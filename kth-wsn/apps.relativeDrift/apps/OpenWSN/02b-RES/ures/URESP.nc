#include "OpenWSN.h"
#include "URES.h"

module URESP {
   provides interface Init as SoftwareInit;
   provides interface ReservationUpdate;
   provides interface DebugPrint;
   uses interface Timer<TMilli> as RemoveFrozenReservationsTimer;
   uses interface Receive;
   uses interface NeighborGet;
   uses interface SimpleSend as SendRES;
   uses interface CellUsageSet;
   uses interface CellUsageGet;
   uses interface Random;
   uses interface EnQueue;
   uses interface GlobalTime;
   uses interface SerialIO;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   ongoingRes_t ongoingRes[NUMONGOINGRES];

   uint8_t debugRow;

   /*-------------------------------- prototypes ----------------------------------------*/

   //FSM
   task void processOngoingRes();
   error_t registerRES(shortnodeid_t neighbor,cellType_t type,bool randomCell,slotOffset_t slotOffset,channelOffset_t channelOffset, bool role);
   error_t localReservation(uint8_t i);
   error_t buildSendREQFREE(uint8_t i);
   error_t buildSendSUCCESSFAIL(uint8_t i);
   void signalReservationUpdateDone(uint8_t i);
   void failReservation(uint8_t i);
   void confirmResIfTodoSuccess(uint8_t i);
   void makeEntryUnused(uint8_t i);

   //helper
   task void taskPrintTable();
   task void taskRemoveFrozenReservations();
   void setReservationTodo(uint8_t i, cellType_t todo);
   void fillRxIndicate(tsch_res_t* data, shortnodeid_t neighbor);
   void fillTxIndicate(tsch_res_t* data, shortnodeid_t neighbor);

   /*-------------------------------- FSM functions -------------------------------------*/

   task void processOngoingRes() {
      uint8_t i;
      i=0;
      while(i<NUMONGOINGRES) {
         if (ongoingRes[i].todo!=UNUSED && ongoingRes[i].todo!=WAIT_FOR_SENT && ongoingRes[i].todo!=WAIT_FOR_RESPONSE) {
            break;
         }
         i++;
      }
      if (i==NUMONGOINGRES) {//nothing to do
         return;
      }
      switch (ongoingRes[i].todo) {
         case LOCAL_RES:
            if (localReservation(i)==SUCCESS) {
               if (ongoingRes[i].role==REQUESTER) {
                  setReservationTodo(i,DISTANT_RES);
               } else {
                  ongoingRes[i].outcome = SUCCESS;
                  setReservationTodo(i,INFORM_REQUESTER);
               }
            } else {
               failReservation(i);
            } 
            break;
         case DISTANT_RES: //I want the replier to reserve the slot I chose
            if (ongoingRes[i].retries==RESRETRIES || buildSendREQFREE(i)==FAIL) {
               failReservation(i);
            } else {
               setReservationTodo(i,WAIT_FOR_SENT);
            }
            break;
         case INFORM_REQUESTER:
            if (ongoingRes[i].role==REQUESTER) {//I need to issue a ReservationUpdate.done
               signalReservationUpdateDone(i);
               makeEntryUnused(i);
            } else { //I need to issue a message informing REQUESTER of outcome of his TXDATA request
               if (buildSendSUCCESSFAIL(i)==SUCCESS) {
                  setReservationTodo(i,WAIT_FOR_SENT);
               } else {
                  failReservation(i);
               }
            }
            break;
         case DISTANT_FAILED_RESTART_LOCAL:
            if ((call CellUsageSet.setUsage(ongoingRes[i].slotOffset,CELLTYPE_OFF,0,0,FALSE))==FAIL) {
               call SerialIO.printError(COMPONENT_URES,ERR_SETUSAGE_OFF_FAILED,
                     (errorparameter_t)ongoingRes[i].slotOffset,
                     (errorparameter_t)ongoingRes[i].channelOffset);
            };
            setReservationTodo(i,LOCAL_RES);
            break;
         default:
            call SerialIO.printError(COMPONENT_URES,
                  ERR_TODO_UNKNOWN,
                  (errorparameter_t)ongoingRes[i].todo,
                  (errorparameter_t)i);
            break;
      }
      post processOngoingRes(); //in case other things to do
   }

   error_t registerRES(shortnodeid_t neighbor, cellType_t type, bool randomCell, slotOffset_t slotOffset, channelOffset_t channelOffset, bool role) {
      uint8_t i;
      /* start error detection */
      if ((call NeighborGet.isNeighbor(neighbor))==FALSE) {
         call SerialIO.printError(COMPONENT_URES,ERR_NEIGHBOR_NOT_REACHABLE_YET,(errorparameter_t)neighbor,0);
         return FAIL;
      }
      if (type!=CELLTYPE_OFF && type!=CELLTYPE_RXDATA && type!=CELLTYPE_TXDATA) {
         call SerialIO.printError(COMPONENT_URES,ERR_REGISTERRES_TYPE_NOT_SUPPORTED,(errorparameter_t)type,0);
         return FAIL;
      }
      /* stop error detection */
      i=0;
      while (i<NUMONGOINGRES) {
         if (ongoingRes[i].todo == UNUSED) {
            setReservationTodo(i,LOCAL_RES);
            ongoingRes[i].type = type;
            ongoingRes[i].neighbor = neighbor;
            ongoingRes[i].retries = 0;
            ongoingRes[i].randomCell = randomCell;
            ongoingRes[i].slotOffset = slotOffset;
            ongoingRes[i].channelOffset = channelOffset;
            ongoingRes[i].role = role;
            ongoingRes[i].timestamp = call GlobalTime.getLocalTime();
            break;
         }
         i++;
      }
      if (i==NUMONGOINGRES) {
         call SerialIO.printError(COMPONENT_URES,ONGOINGRES_TABLE_FULL,NUMONGOINGRES,0);
         return FAIL;
      }
      return SUCCESS;
   }

   error_t localReservation(uint8_t i) {
      error_t local_reservation_ok;
      uint8_t local_reservation_retries;
      local_reservation_ok = FAIL;
      if (ongoingRes[i].randomCell==1) {
         local_reservation_retries = 0;
         while (local_reservation_ok==FAIL && local_reservation_retries<RESRETRIES) {
            local_reservation_retries++;
            ongoingRes[i].slotOffset    = ((call Random.rand16())%(LENGTHCELLFRAME-4))+4;
            ongoingRes[i].channelOffset =  (call Random.rand16())%16;
            local_reservation_ok = (call CellUsageSet.setUsage(ongoingRes[i].slotOffset,
                     CELLTYPE_RESERVED,
                     ongoingRes[i].neighbor,
                     ongoingRes[i].channelOffset,
                     TRUE));
         }
      } else {
         if (ongoingRes[i].type==CELLTYPE_OFF) {
            local_reservation_ok = (call CellUsageSet.setUsageForceOFF(ongoingRes[i].slotOffset,ongoingRes[i].channelOffset));
         } else {
            local_reservation_ok = (call CellUsageSet.setUsage(ongoingRes[i].slotOffset,
                     CELLTYPE_RESERVED,
                     ongoingRes[i].neighbor,
                     ongoingRes[i].channelOffset,
                     FALSE));
         }
      }
      return local_reservation_ok;
   }

   error_t buildSendREQFREE(uint8_t i) {
      bool result;
      ongoingRes[i].pkt = call EnQueue.getFreePacketBuffer();
      if (ongoingRes[i].pkt==NULL) {
         call SerialIO.printError(COMPONENT_URES,ERR_NO_FREE_QUEUEPKT_CELL,0,0);
         return FAIL;
      }
      ((cc2420_header_t*)((ongoingRes[i].pkt)->header))->length = sizeof(tsch_res_t);
      ((cc2420_header_t*)((ongoingRes[i].pkt)->header))->dest = ongoingRes[i].neighbor;
      ((cc2420_header_t*)((ongoingRes[i].pkt)->header))->type = OPEN_RES;
      switch(ongoingRes[i].type) {
         case CELLTYPE_TXDATA:
            ((tsch_res_t*)((ongoingRes[i].pkt)->data))->txCommand = RES_ACTION_REQ_CELL;
            break;
         case CELLTYPE_OFF:
            ((tsch_res_t*)((ongoingRes[i].pkt)->data))->txCommand = RES_ACTION_FREE_CELL;
            break;
         default:
            call EnQueue.freePacketBuffer(ongoingRes[i].pkt);
            call SerialIO.printError(COMPONENT_URES,ERR_RES_TYPE_UNKNOWN,(errorparameter_t)ongoingRes[i].type,0);
            return FAIL;
            break;
      }
      ((tsch_res_t*)((ongoingRes[i].pkt)->data))->txSlotOffset = ongoingRes[i].slotOffset;
      ((tsch_res_t*)((ongoingRes[i].pkt)->data))->txChannelOffset = ongoingRes[i].channelOffset;
      fillRxIndicate( ((tsch_res_t*)((ongoingRes[i].pkt)->data)) ,ongoingRes[i].neighbor);
      result = call SendRES.send(ongoingRes[i].pkt,sizeof(tsch_res_t));
      if (result==FAIL) {
         call EnQueue.freePacketBuffer(ongoingRes[i].pkt);
      }
      return result;
   }

   error_t buildSendSUCCESSFAIL(uint8_t i) {
      bool result;
      ongoingRes[i].pkt = call EnQueue.getFreePacketBuffer();
      if (ongoingRes[i].pkt==NULL) {
         call SerialIO.printError(COMPONENT_URES,ERR_NO_FREE_QUEUEPKT_CELL,0,0);
         return FAIL;
      }
      ((cc2420_header_t*)((ongoingRes[i].pkt)->header))->length = sizeof(tsch_res_t);
      ((cc2420_header_t*)((ongoingRes[i].pkt)->header))->dest = ongoingRes[i].neighbor;
      ((cc2420_header_t*)((ongoingRes[i].pkt)->header))->type = OPEN_RES;
      fillTxIndicate(  ((tsch_res_t*)((ongoingRes[i].pkt)->data)) ,ongoingRes[i].neighbor);
      if (ongoingRes[i].outcome==SUCCESS) {
         ((tsch_res_t*)((ongoingRes[i].pkt)->data))->rxCommand = RES_ACTION_SUCCESS;
      } else {
         ((tsch_res_t*)((ongoingRes[i].pkt)->data))->rxCommand = RES_ACTION_FAIL;
      }
      ((tsch_res_t*)((ongoingRes[i].pkt)->data))->rxSlotOffset = ongoingRes[i].slotOffset;
      ((tsch_res_t*)((ongoingRes[i].pkt)->data))->rxChannelOffset = ongoingRes[i].channelOffset;
      result = call SendRES.send(ongoingRes[i].pkt,sizeof(tsch_res_t));
      if (result==FAIL) {
         call EnQueue.freePacketBuffer(ongoingRes[i].pkt);
      }
      return result;
   }

   void signalReservationUpdateDone(uint8_t i) {
      switch (ongoingRes[i].type) {
         case CELLTYPE_TXDATA:
            signal ReservationUpdate.done(ADD,ongoingRes[i].neighbor,ongoingRes[i].outcome);
            break;
         case CELLTYPE_OFF:
            signal ReservationUpdate.done(REMOVE,ongoingRes[i].neighbor,ongoingRes[i].outcome);
            break;
         default:
            call SerialIO.printError(COMPONENT_URES,ERR_LOCALRES_NOT_TXDATA_OR_OFF,(errorparameter_t)ongoingRes[i].type,0);
            break;
      }
   }

   void failReservation(uint8_t i) {
      call SerialIO.printError(COMPONENT_URES,ERR_FAILRESERVATION,(errorparameter_t)(ongoingRes[i].todo),0);
      if ((call CellUsageGet.getType(ongoingRes[i].slotOffset))==CELLTYPE_RESERVED) {
         if ((call CellUsageSet.setUsage(ongoingRes[i].slotOffset,CELLTYPE_OFF,0,0,FALSE))==FAIL) {
            call SerialIO.printError(COMPONENT_URES,ERR_SETUSAGE_OFF_FAILED,
                  (errorparameter_t)ongoingRes[i].slotOffset,
                  (errorparameter_t)ongoingRes[i].channelOffset);
         }
      }
      ongoingRes[i].outcome = FAIL;
      ongoingRes[i].timestamp = call GlobalTime.getLocalTime();
      setReservationTodo(i,INFORM_REQUESTER);
   }

   void confirmResIfTodoSuccess(uint8_t i) {
      if (ongoingRes[i].outcome==SUCCESS) {
         if ((call CellUsageSet.setUsage(ongoingRes[i].slotOffset,
                     ongoingRes[i].type,
                     ongoingRes[i].neighbor,
                     ongoingRes[i].channelOffset,
                     FALSE))==FAIL) {
            call SerialIO.printError(COMPONENT_URES,ERR_CONFIRMRESERVATION_FAILED,(errorparameter_t)ongoingRes[i].type,(errorparameter_t)ongoingRes[i].neighbor);
         }
      }
   }

   void makeEntryUnused(uint8_t i) {
      setReservationTodo(i,UNUSED);
      ongoingRes[i].type = CELLTYPE_OFF;
      ongoingRes[i].neighbor = 0;
      ongoingRes[i].retries = 0;
      ongoingRes[i].randomCell = 0;
      ongoingRes[i].slotOffset = 0;
      ongoingRes[i].channelOffset = 0;
      ongoingRes[i].role = REPLIER;
      ongoingRes[i].outcome = FAIL;
      ongoingRes[i].timestamp = 0;
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   //Receive
   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
      uint8_t i;
      /* start error detection */
      if (((cc2420_header_t*)msg->header)->type!=OPEN_RES) {
         call SerialIO.printError(COMPONENT_URES,ERR_RXMSG_NOT_RES,(errorparameter_t)((cc2420_header_t*)msg->header)->type,0);
         return msg;
      }
      /* stop error detection */
      call SerialIO.printError(COMPONENT_URES,ERR_RES_RECEIVED,(errorparameter_t)((tsch_res_t*)msg->data)->txCommand,(errorparameter_t)((tsch_res_t*)msg->data)->rxCommand);
      switch (((tsch_res_t*)msg->data)->txCommand) {
         case RES_ACTION_FREE_CELL:
            registerRES(((cc2420_header_t*)msg->header)->src,
                  CELLTYPE_OFF,
                  NORANDOM,
                  ((tsch_res_t*)msg->data)->txSlotOffset,
                  ((tsch_res_t*)msg->data)->txChannelOffset,
                  REPLIER);
            break;
         case RES_ACTION_REQ_CELL:
            call CellUsageSet.removeAllCellsWithNeighbor(((cc2420_header_t*)msg->header)->src,CELLTYPE_RXDATA);
            registerRES(((cc2420_header_t*)msg->header)->src,
                  CELLTYPE_RXDATA,
                  NORANDOM,
                  ((tsch_res_t*)msg->data)->txSlotOffset,
                  ((tsch_res_t*)msg->data)->txChannelOffset,
                  REPLIER);
            break;
         case RES_INDICATION_USED:
            if (  (call CellUsageGet.getType(    ((tsch_res_t*)msg->data)->txSlotOffset)!=CELLTYPE_RXDATA) ||
                  (call CellUsageGet.getNeighbor(((tsch_res_t*)msg->data)->txSlotOffset)!=((cc2420_header_t*)msg->header)->src) ||
                  (call CellUsageGet.getChannel( ((tsch_res_t*)msg->data)->txSlotOffset)!=((tsch_res_t*)msg->data)->txChannelOffset)) {
               call CellUsageSet.removeAllCellsWithNeighbor(((cc2420_header_t*)msg->header)->src,CELLTYPE_RXDATA);
               registerRES(((cc2420_header_t*)msg->header)->src,
                     CELLTYPE_OFF,
                     NORANDOM,
                     ((tsch_res_t*)msg->data)->txSlotOffset,
                     ((tsch_res_t*)msg->data)->txChannelOffset,
                     REQUESTER);
               call SerialIO.printError(COMPONENT_URES,ERR_INCONSISTENT_TXCOMMAND,
                     (errorparameter_t)((cc2420_header_t*)msg->header)->src,
                     (errorparameter_t)((tsch_res_t*)msg->data)->rxSlotOffset);
            }
            break;
         case RES_INDICATION_UNUSED:
            call CellUsageSet.removeAllCellsWithNeighbor(((cc2420_header_t*)msg->header)->src,CELLTYPE_RXDATA);
            break;
         default:
            call SerialIO.printError(COMPONENT_URES,ERR_UNSUPPORTED_TXCOMMAND,((tsch_res_t*)msg->data)->txCommand,0);
            break;
      }
      switch (((tsch_res_t*)msg->data)->rxCommand) {
         case RES_ACTION_FAIL:
         case RES_ACTION_SUCCESS:
            i=0;
            while(i<NUMONGOINGRES) {
               if (     ongoingRes[i].todo          == WAIT_FOR_RESPONSE
                     && ongoingRes[i].neighbor      == ((cc2420_header_t*)msg->header)->src
                     && ongoingRes[i].slotOffset    == ((tsch_res_t*)msg->data)->rxSlotOffset
                     && ongoingRes[i].channelOffset == ((tsch_res_t*)msg->data)->rxChannelOffset
                     && ongoingRes[i].role          == REQUESTER) {
                  if (((tsch_res_t*)msg->data)->rxCommand==RES_ACTION_SUCCESS) {
                     ongoingRes[i].outcome = SUCCESS;
                     confirmResIfTodoSuccess(i);
                     setReservationTodo(i,INFORM_REQUESTER);
                  } else {
                     setReservationTodo(i,DISTANT_FAILED_RESTART_LOCAL);
                  }
                  break;
               }
               i++;
            }
            if (i==NUMONGOINGRES) {
               call SerialIO.printError(COMPONENT_URES,ERR_UNEXPECTED_RES_ACTION_SUCCESS,0,0);
               return msg;
            }
            break;
         case RES_INDICATION_USED:
            if (  (call CellUsageGet.getType(    ((tsch_res_t*)msg->data)->rxSlotOffset)!=CELLTYPE_TXDATA) ||
                  (call CellUsageGet.getNeighbor(((tsch_res_t*)msg->data)->rxSlotOffset)!=((cc2420_header_t*)msg->header)->src) ||
                  (call CellUsageGet.getChannel( ((tsch_res_t*)msg->data)->rxSlotOffset)!=((tsch_res_t*)msg->data)->rxChannelOffset)) {
               call CellUsageSet.removeAllCellsWithNeighbor(((cc2420_header_t*)msg->header)->src,CELLTYPE_TXDATA);
               registerRES(((cc2420_header_t*)msg->header)->src,
                     CELLTYPE_OFF,
                     NORANDOM,
                     ((tsch_res_t*)msg->data)->rxSlotOffset,
                     ((tsch_res_t*)msg->data)->rxChannelOffset,
                     REQUESTER);
               call SerialIO.printError(COMPONENT_URES,ERR_INCONSISTENT_RXCOMMAND,
                     (errorparameter_t)((cc2420_header_t*)msg->header)->src,
                     (errorparameter_t)((tsch_res_t*)msg->data)->rxSlotOffset);
            }
            break;
         case RES_INDICATION_UNUSED:
            call CellUsageSet.removeAllCellsWithNeighbor(((cc2420_header_t*)msg->header)->src,CELLTYPE_TXDATA);
            break;
         default:
            call SerialIO.printError(COMPONENT_URES,ERR_UNSUPPORTED_RXCOMMAND,((tsch_res_t*)msg->data)->rxCommand,0);
            break;
      }
      return msg;
   } 

   //SendRES
   event void SendRES.sendDone(message_t *msg, error_t error) {
      uint8_t i=0;
      while(i<NUMONGOINGRES) {
         if (ongoingRes[i].todo!=UNUSED && msg==ongoingRes[i].pkt) {
            break;
         }
         i++;
      }
      /* start error detection */
      if (i==NUMONGOINGRES) {
         call SerialIO.printError(COMPONENT_URES,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
         return;
      }
      if (ongoingRes[i].todo!=WAIT_FOR_SENT) {
         call SerialIO.printError(COMPONENT_URES,ERR_SENDDONE_WHILE_NOT_TODO_WAIT_FOR_SENT,0,0);
         return;
      }
      if (error==FAIL) { //radio could not send out the message
         failReservation(i);
      }
      /* stop error detection */
      call SerialIO.printError(COMPONENT_URES,ERR_RES_SENT,
            (errorparameter_t)ongoingRes[i].slotOffset,
            (errorparameter_t)((cc2420_metadata_t*)msg->metadata)->ack);
      ongoingRes[i].retries++;
      switch(((cc2420_metadata_t*)msg->metadata)->ack) {
         case TRUE:  //RES successfully sent
            if (ongoingRes[i].role==REQUESTER) { //I sent a request
               setReservationTodo(i,WAIT_FOR_RESPONSE);
            } else {                             //I sent a reply
               confirmResIfTodoSuccess(i);
               makeEntryUnused(i);
            }
            break;
         case FALSE: //RES not successfully sent
            setReservationTodo(i,DISTANT_RES); //triggers resend
            break;
         default:
            call SerialIO.printError(COMPONENT_URES,ERR_UNSUPPORTED_ACK_RESPONSE,0,0);
            break;
      }
   }

   //SoftwareInit
   command error_t SoftwareInit.init() {
      uint8_t i;
      for (i=0;i<NUMONGOINGRES;i++) {
         ongoingRes[i].todo = UNUSED;
      }
      return SUCCESS;
   }

   //ReservationUpdate
   command uint8_t ReservationUpdate.numSlots(shortnodeid_t neighbor) {
      return call CellUsageGet.numCells(CELLTYPE_TXDATA,neighbor);
   }
   command error_t ReservationUpdate.addSlot(shortnodeid_t neighbor) {
      return registerRES(neighbor, CELLTYPE_TXDATA, PICKRANDOM, 0, 0, REQUESTER);
   }
   command error_t ReservationUpdate.removeSlot(shortnodeid_t neighbor) {
      slotChannel_t slotChannel = (call CellUsageGet.getCell(CELLTYPE_TXDATA, neighbor));
      if (slotChannel.exists==FALSE) {
         return FAIL;
      }
      return registerRES(neighbor, CELLTYPE_OFF, NORANDOM, slotChannel.slotOffset, slotChannel.channelOffset, REQUESTER);
   }
   command error_t ReservationUpdate.removeAllOngoingResToNeighbor(shortnodeid_t neighbor) {
      error_t return_value=FAIL;
      uint8_t i;
      for (i=0;i<NUMONGOINGRES;i++) {
         if (ongoingRes[i].todo!=UNUSED && ongoingRes[i].neighbor==neighbor) {
            failReservation(i);
            return_value=SUCCESS;
         }
      }
      return return_value;
   }

   //Timer
   event void RemoveFrozenReservationsTimer.fired() {
      post taskRemoveFrozenReservations();
   }

   //DebugPrint
   command void DebugPrint.print() {
      post taskPrintTable();
   }

   /*-------------------------------- helper functions ----------------------------------*/

   task void taskPrintTable() {
      debugOngoingRes_t temp;
      debugRow = (debugRow+1)%NUMONGOINGRES;
      temp.row            = debugRow;
      temp.ongoingRes     = ongoingRes[debugRow];
      call SerialIO.printStatus(STATUS_RESERVATIONP_ONGOINGRES,(uint8_t*)&temp,sizeof(debugOngoingRes_t));
   }

   task void taskRemoveFrozenReservations() {
      timervalue_t now = call GlobalTime.getLocalTime();
      uint8_t i;
      for (i=0;i<NUMONGOINGRES;i++) {
         switch (ongoingRes[i].todo) {
            case UNUSED:
               break;
            case LOCAL_RES:
               //should be immediate, delay as worst case on task execution
               if ((now-ongoingRes[i].timestamp)>30000) {
                  call SerialIO.printError(COMPONENT_URES,ERR_DETECTED_FROZEN,
                        (errorparameter_t)ongoingRes[i].type,
                        (errorparameter_t)ongoingRes[i].neighbor);
                  failReservation(i);
               }
               break;
            case DISTANT_RES:
               //should be immediate, delay as worst case on task execution
               if ((now-ongoingRes[i].timestamp)>30000) {
                  call SerialIO.printError(COMPONENT_URES,ERR_DETECTED_FROZEN,
                        (errorparameter_t)ongoingRes[i].type,
                        (errorparameter_t)ongoingRes[i].neighbor);
                  failReservation(i);
               }
               break;
            case WAIT_FOR_SENT:
               //delay includes retransmission, not just radio access
               if ((now-ongoingRes[i].timestamp)>DELAY_FROZEN_RESERVATIONS_32KHZ) {
                  call SerialIO.printError(COMPONENT_URES,ERR_DETECTED_FROZEN,
                        (errorparameter_t)ongoingRes[i].type,
                        (errorparameter_t)ongoingRes[i].neighbor);
                  failReservation(i);
               }
               break;
            case WAIT_FOR_RESPONSE:
               if ((now-ongoingRes[i].timestamp)>DELAY_FROZEN_RESERVATIONS_32KHZ) {
                  call SerialIO.printError(COMPONENT_URES,ERR_DETECTED_FROZEN,
                        (errorparameter_t)ongoingRes[i].type,
                        (errorparameter_t)ongoingRes[i].neighbor);
                  failReservation(i);
               }
               break;
            case DISTANT_FAILED_RESTART_LOCAL:
               //should be immediate, delay as worst case on task execution
               if ((now-ongoingRes[i].timestamp)>30000) {
                  call SerialIO.printError(COMPONENT_URES,ERR_DETECTED_FROZEN,
                        (errorparameter_t)ongoingRes[i].type,
                        (errorparameter_t)ongoingRes[i].neighbor);
                  failReservation(i);
               }
               break;
            case INFORM_REQUESTER:
               //should be immediate, delay as worst case on task execution
               if ((now-ongoingRes[i].timestamp)>30000) {
                  call SerialIO.printError(COMPONENT_URES,ERR_DETECTED_FROZEN,
                        (errorparameter_t)ongoingRes[i].type,
                        (errorparameter_t)ongoingRes[i].neighbor);
                  failReservation(i);
               }
               break;
            default:
               break;
         }
      }
   }

   void setReservationTodo(uint8_t i, cellType_t todo) {
      ongoingRes[i].todo = todo;
      post processOngoingRes();
      call RemoveFrozenReservationsTimer.startOneShot(DELAY_FROZEN_RESERVATIONS_1KHZ);
   }

   void fillRxIndicate(tsch_res_t* data, shortnodeid_t neighbor) {
      slotChannel_t rxDataSlotChannel;
      uint8_t i;
      //there is a RXDATA slot to that neigbhor?
      rxDataSlotChannel = call CellUsageGet.getCell(CELLTYPE_RXDATA,neighbor);
      if (rxDataSlotChannel.exists==TRUE) {
         //if yes, send that
         data->rxCommand       = RES_INDICATION_USED;
         data->rxSlotOffset    = rxDataSlotChannel.slotOffset;
         data->rxChannelOffset = rxDataSlotChannel.channelOffset;
         return;
      }
      //there is no RXDATA slot, but is there a RESERVED slot?
      rxDataSlotChannel = call CellUsageGet.getCell(CELLTYPE_RESERVED,neighbor);
      if (rxDataSlotChannel.exists==TRUE) {
         //does that RESERVED slot correspond to a RXDATA reservation to that neigbhor?
         i=0;
         while (i<NUMONGOINGRES) {
            if (  ongoingRes[i].type          == CELLTYPE_RXDATA &&
                  ongoingRes[i].neighbor      == neighbor &&
                  ongoingRes[i].slotOffset    == rxDataSlotChannel.slotOffset &&
                  ongoingRes[i].channelOffset == rxDataSlotChannel.channelOffset) {
               data->rxCommand       = RES_INDICATION_USED;
               data->rxSlotOffset    = rxDataSlotChannel.slotOffset;
               data->rxChannelOffset = rxDataSlotChannel.channelOffset;
               return;
            }
            i++;
         }
      }
      data->rxCommand       = RES_INDICATION_UNUSED;
      data->rxSlotOffset    = 0;
      data->rxChannelOffset = 0;
   }

   void fillTxIndicate(tsch_res_t* data, shortnodeid_t neighbor) {
      slotChannel_t txDataSlotChannel;
      uint8_t i;
      //there is a TXDATA slot to that neigbhor?
      txDataSlotChannel = call CellUsageGet.getCell(CELLTYPE_TXDATA,neighbor);
      if (txDataSlotChannel.exists==TRUE) {
         //if yes, send that
         data->txCommand       = RES_INDICATION_USED;
         data->txSlotOffset    = txDataSlotChannel.slotOffset;
         data->txChannelOffset = txDataSlotChannel.channelOffset;
         return;
      }
      //there is no TXDATA slot, but is there a RESERVED slot?
      txDataSlotChannel = call CellUsageGet.getCell(CELLTYPE_RESERVED,neighbor);
      if (txDataSlotChannel.exists==TRUE) {
         //does that RESERVED slot correspond to a TXDATA reservation to that neigbhor?
         i=0;
         while (i<NUMONGOINGRES) {
            if (  ongoingRes[i].type          == CELLTYPE_TXDATA &&
                  ongoingRes[i].neighbor      == neighbor &&
                  ongoingRes[i].slotOffset    == txDataSlotChannel.slotOffset &&
                  ongoingRes[i].channelOffset == txDataSlotChannel.channelOffset) {
               data->txCommand       = RES_INDICATION_USED;
               data->txSlotOffset    = txDataSlotChannel.slotOffset;
               data->txChannelOffset = txDataSlotChannel.channelOffset;
               return;
            }
            i++;
         }
      }
      data->txCommand       = RES_INDICATION_UNUSED;
      data->txSlotOffset    = 0;
      data->txChannelOffset = 0;
   }
}
