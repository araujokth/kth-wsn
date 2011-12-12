#include "LATIN.h"
#include "Neighbors.h"
#include "IEEE802154E.h"

module NeighborsP {
   provides interface Init as SoftwareInit;
   //down the stack
   uses interface OpenSend as OpenSendKAToLower;
   //up the stack
   provides interface OpenReceive as OpenReceiveADVFromLower;
   //misc
   provides interface NeighborGet;
   provides interface DebugPrint;
   provides interface NeighborStats;
   uses interface GlobalTime;
   uses interface GlobalSync;
   uses interface OpenQueue;
   uses interface Malloc;
//   uses interface OpenSerial;
   uses interface IDManager;
   uses interface Random;
   uses interface PacketFunctions;
}

implementation {

   /*-------------------------------- variables -----------------------------------------*/

   neighborEntry_t    neighbors[MAXNUMNEIGHBORS];
   dagrank_t          myDAGrank;
   open_addr_t        KAdestination;
   bool               busySendKA;
   uint8_t            debugRow;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskPrintTable();
   task void taskHandleLosingNeighbors();
   task void taskUpdateMyDAGrankAndNeighborPreference();
   task void taskKeepAliveSync();

   void registerNewNeighbor(open_addr_t* neighborID);
   void buildAndSendKA(open_addr_t* dest);
   bool isNeighbor(open_addr_t* neighbor);
   void removeNeighbor(uint8_t neighborIndex);
   bool isThisRowMatching(open_addr_t* address, uint8_t rowNumber);

   /*-------------------------------- ADV ---------------------------------------------*/

   command void OpenReceiveADVFromLower.receive(OpenQueueEntry_t* msg) {
      uint8_t i;
      msg->owner = COMPONENT_NEIGHBORS;
      if (isNeighbor(&(msg->l2_nextORpreviousHop))==TRUE) {
         for (i=0;i<MAXNUMNEIGHBORS;i++) {
            if (isThisRowMatching(&(msg->l2_nextORpreviousHop),i)) {
               atomic neighbors[i].DAGrank = ((IEEE802154E_ADV_t*)(msg->payload))->DAGrank;
               break;
            }
         }
      } else {
         registerNewNeighbor(&(msg->l2_nextORpreviousHop));
      }
      call Malloc.freePacketBuffer(msg);
   }

   task void taskUpdateMyDAGrankAndNeighborPreference() {
      uint8_t   i;
      uint8_t   temp_linkCost;
      uint16_t  temp_myTentativeDAGrank; //has to be 16bit, so that the sum can be larger than 255
      bool      temp_condition;
      uint8_t   temp_preferredParentRow=0;
      bool      temp_preferredParentExists=FALSE;
      if ((call IDManager.getIsDAGroot())==FALSE) {
         myDAGrank=255;
         i=0;
         while(i<MAXNUMNEIGHBORS) {
            atomic neighbors[i].parentPreference=0;
            atomic temp_condition = neighbors[i].used==TRUE && neighbors[i].stableNeighbor==TRUE;
            if (temp_condition) {
               atomic {
                  if (neighbors[i].numTxACK==0) {
                     temp_linkCost=40; //TODO: evaluate using RSSI?
                  } else {
                     temp_linkCost=(uint8_t)((((float)neighbors[i].numTx)/((float)neighbors[i].numTxACK))*10.0);
                  }
               }
               atomic temp_myTentativeDAGrank=neighbors[i].DAGrank+temp_linkCost;
               if (temp_myTentativeDAGrank<myDAGrank && temp_myTentativeDAGrank<255) {
                  myDAGrank=temp_myTentativeDAGrank;
                  temp_preferredParentExists=TRUE;
                  temp_preferredParentRow=i;
               }
               /*poipoi manual routing
                 switch ((call IDManager.getMyID(ADDR_16B))->addr_16b[1]) {
                 case 0x03:
                 if (neighbors[i].addr_16b.addr_16b[1]==0x07) {
                 myDAGrank=neighbors[i].DAGrank+temp_linkCost;
                 temp_preferredParentExists=TRUE;
                 temp_preferredParentRow=i;
                 }
                 break;
                 case 0x07:
                 if (neighbors[i].addr_16b.addr_16b[1]==0x01) {
                 myDAGrank=neighbors[i].DAGrank+temp_linkCost;
                 temp_preferredParentExists=TRUE;
                 temp_preferredParentRow=i;
                 }
                 break;
                 default:
                 break;
                 }*/
            }
            i++;
         }
         if (temp_preferredParentExists) {
            atomic neighbors[temp_preferredParentRow].parentPreference=MAXPREFERENCE;
         }
      } else {
         myDAGrank=0;
      }
   }

   /*-------------------------------- KA ----------------------------------------------*/

   event void GlobalSync.losingSync() { 
      post taskKeepAliveSync();
   }

   task void taskKeepAliveSync() {
      open_addr_t timeParent;
      call NeighborGet.getPreferredParent(&timeParent,ADDR_128B);
      if (timeParent.type!=ADDR_NONE){
         buildAndSendKA(&timeParent);
      }
   }

   void buildAndSendKA(open_addr_t* dest) {
      /*poipoiOpenQueueEntry_t*  KApkt;
        if (busySendKA==TRUE) {
        return;
        } else {
        busySendKA=TRUE;
        KApkt = call Malloc.getFreePacketBuffer();
        if (KApkt==NULL) {
        call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_NO_FREE_PACKET_BUFFER,(errorparameter_t)0,(errorparameter_t)0);
        return;
        }
        KApkt->creator = COMPONENT_NEIGHBORS;
        KApkt->owner = COMPONENT_NEIGHBORS;
        ((cc2420_header_t*)KApkt->packet.header)->length=sizeof(tsch_ka_t);
        ((cc2420_header_t*)KApkt->packet.header)->dest=dest;
        ((cc2420_header_t*)KApkt->packet.header)->type=OPEN_KA;
        if (call OpenSendKAToLower.send(KApkt)==FAIL) {
        call Malloc.freePacketBuffer(KApkt);
        busySendKA=FALSE;
        }
        }*/
   }

   event void OpenSendKAToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_NEIGHBORS;
      if (msg->creator!=COMPONENT_NEIGHBORS) {
//         call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      if (busySendKA==FALSE) {
//         call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_SENDDONE_WHILE_NOT_BUSY,0,0);
      }
      call Malloc.freePacketBuffer(msg);
      atomic busySendKA=FALSE;
   }

   /*-------------------------------- NeighborStats -------------------------------------*/

   async command void NeighborStats.indicateRx(open_addr_t* src, uint16_t rssi) {
      uint8_t i=0;
      while (i<MAXNUMNEIGHBORS) {
         if (isThisRowMatching(src,i)) {
            atomic {
               neighbors[i].numRx++;
               neighbors[i].linkQuality=rssi;
               neighbors[i].timestamp=call GlobalTime.getLocalTime();
               if (neighbors[i].stableNeighbor==FALSE) {
                  if (neighbors[i].linkQuality>BADNEIGHBORMAXPOWER || neighbors[i].linkQuality<129) {
                     neighbors[i].switchStabilityCounter++;
                     if (neighbors[i].switchStabilityCounter>=SWITCHSTABILITYTHRESHOLD) {
                        neighbors[i].switchStabilityCounter=0;
                        neighbors[i].stableNeighbor=TRUE;
                     }
                  } else {
                     neighbors[i].switchStabilityCounter=0;
                  }
               } else if (neighbors[i].stableNeighbor==TRUE) {
                  if (neighbors[i].linkQuality<GOODNEIGHBORMINPOWER && neighbors[i].linkQuality>128) {
                     neighbors[i].switchStabilityCounter++;
                     if (neighbors[i].switchStabilityCounter>=SWITCHSTABILITYTHRESHOLD) {
                        neighbors[i].switchStabilityCounter=0;
                        neighbors[i].stableNeighbor=FALSE;
                     }
                  } else {
                     neighbors[i].switchStabilityCounter=0;
                  }
               }
            }
            return;
         }
         i++;   
      }
      registerNewNeighbor(src);
   }
   async command void NeighborStats.indicateTx(open_addr_t* dest, bool was_acked) {
      uint8_t i=0;
      if (call PacketFunctions.isBroadcastMulticast(dest)) {//means I just sent an ADV
         post taskUpdateMyDAGrankAndNeighborPreference();
         post taskHandleLosingNeighbors();
      } else {
         for (i=0;i<MAXNUMNEIGHBORS;i++) {
            if (isThisRowMatching(dest,i)) {
               atomic {
                  if (neighbors[i].numTx==255) {
                     neighbors[i].numTx/=2;
                     neighbors[i].numTxACK/=2;
                  }
                  neighbors[i].numTx++;
                  if (was_acked==TRUE) {
                     neighbors[i].numTxACK++;
                     neighbors[i].timestamp=call GlobalTime.getLocalTime();
                  }
               }
               return;
            }
         }
         registerNewNeighbor(dest);
      }
   }

   task void taskHandleLosingNeighbors() {
      timervalue_t timeSinceLastHeard;
      bool         temp_condition;
      open_addr_t* temp_neighborid;
      uint8_t      i=0;
      while (i<MAXNUMNEIGHBORS){
         atomic temp_condition = (neighbors[i].used==TRUE);
         if (temp_condition) {
            atomic timeSinceLastHeard = (call GlobalTime.getLocalTime())-neighbors[i].timestamp;
            atomic temp_neighborid = &(neighbors[i].addr_16b);
            if (timeSinceLastHeard>DELAY_LOST_NEIGHBOR_32KHZ) {
//               call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_LOST_NEIGHBOR,
//                     (errorparameter_t)(temp_neighborid->addr_16b[0]<<8|temp_neighborid->addr_16b[1]),
//                     (errorparameter_t)0);
               //call OpenQueue.removeAllPacketsToNeighbor(temp_neighborid);poipoi
               removeNeighbor(i);
               break;
            } else if (timeSinceLastHeard>DELAY_LOSING_NEIGHBOR_32KHZ) {
//               call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_LOSING_NEIGHBOR,
//                     (errorparameter_t)(temp_neighborid->addr_16b[0]<<8|temp_neighborid->addr_16b[1]),
//                     (errorparameter_t)0);
               //memcpy(&KAdestination,temp_neighborid,sizeof(open_addr_t));
               //buildAndSendKA(&KAdestination);poipoi
               break;
            }
         }
         i++;
      }
   }

   void removeNeighbor(uint8_t neighborIndex) {
      atomic {
         neighbors[neighborIndex].used                      = FALSE;
         neighbors[neighborIndex].parentPreference          = 0;
         neighbors[neighborIndex].stableNeighbor            = FALSE;
         neighbors[neighborIndex].switchStabilityCounter    = 0;
         neighbors[neighborIndex].addr_16b.type             = ADDR_NONE;
         neighbors[neighborIndex].addr_64b.type             = ADDR_NONE;
         neighbors[neighborIndex].addr_128b.type            = ADDR_NONE;
         neighbors[neighborIndex].DAGrank                   = 255;
         neighbors[neighborIndex].linkQuality               = 0;
         neighbors[neighborIndex].numRx                     = 0;
         neighbors[neighborIndex].numTx                     = 0;
         neighbors[neighborIndex].numTxACK                  = 0;
         neighbors[neighborIndex].timestamp                 = 0;
      }
   }

   /*-------------------------------- DebugPrint ----------------------------------------*/

   command void DebugPrint.print() {
      post taskPrintTable();
   }

   task void taskPrintTable() {
      debugNeighborEntry_t temp;
      debugRow=(debugRow+1)%MAXNUMNEIGHBORS;
      temp.row=debugRow;
      atomic temp.neighborEntry=neighbors[debugRow];
//      call OpenSerial.printStatus(STATUS_NEIGHBORSP_NEIGHBORS,(uint8_t*)&temp,sizeof(debugNeighborEntry_t));
   }

   /*-------------------------------- misc ----------------------------------------------*/

   command bool NeighborGet.isStableNeighbor(open_addr_t* address) {
      uint8_t i=0;
      bool temp_stableNeighbor;
      for (i=0;i<MAXNUMNEIGHBORS;i++) {
         atomic temp_stableNeighbor = neighbors[i].stableNeighbor;
         if (isThisRowMatching(address,i) && temp_stableNeighbor==TRUE) {
            return TRUE;
         }
      }
      return FALSE;
   }

   command error_t SoftwareInit.init() {
      uint8_t i;
      busySendKA=FALSE;
      for (i=0;i<MAXNUMNEIGHBORS;i++){
         atomic neighbors[i].used=FALSE;
      }
      if ((call IDManager.getIsDAGroot())==TRUE) {
         atomic myDAGrank=0;
      } else {
         atomic myDAGrank=255;
      }
      return SUCCESS;
   }

   command dagrank_t NeighborGet.getMyDAGrank() {
      return myDAGrank;
   }
   command uint8_t NeighborGet.getNumNeighbors(){
      uint8_t i;
      uint8_t returnvalue=0;
      for (i=0;i<MAXNUMNEIGHBORS;i++) {
         atomic {
            if (neighbors[i].used==TRUE) {
               returnvalue++;
            }
         }
      }
      return returnvalue;
   }
   command void NeighborGet.getPreferredParent(open_addr_t* addressToWrite, uint8_t addr_type) {
      //following commented out section is equivalent to setting a default gw
      /*
         open_addr_t    nextHop;
         nextHop.type = ADDR_64B;
         nextHop.addr_64b[0]=0x00;
         nextHop.addr_64b[1]=0x00;
         nextHop.addr_64b[2]=0x00;
         nextHop.addr_64b[3]=0x00;
         nextHop.addr_64b[4]=0x00;
         nextHop.addr_64b[5]=0x00;
         nextHop.addr_64b[6]=0x00;
         nextHop.addr_64b[7]=0x01;
         memcpy(addressToWrite,&nextHop,sizeof(open_addr_t));*/
      uint8_t i;
      bool temp_condition;
      addressToWrite->type=ADDR_NONE;
      for (i=0; i<MAXNUMNEIGHBORS; i++) {
         atomic temp_condition = (neighbors[i].used==TRUE && neighbors[i].parentPreference==MAXPREFERENCE);
         if (temp_condition) {
            switch(addr_type) {
               case ADDR_16B:
                  memcpy(addressToWrite,&(neighbors[i].addr_16b),sizeof(open_addr_t));
                  break;
               case ADDR_64B:
                  memcpy(addressToWrite,&(neighbors[i].addr_64b),sizeof(open_addr_t));
                  break;
               case ADDR_128B:
                  memcpy(addressToWrite,&(neighbors[i].addr_128b),sizeof(open_addr_t));
                  break;
               default:
//                  call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
//                        (errorparameter_t)addr_type,
//                        (errorparameter_t)0);
                  break;
            }
            return;
         }
      }
   }

   event void GlobalSync.lostSync() { 
   }

   /*-------------------------------- helper functions ----------------------------------*/

   bool isNeighbor(open_addr_t* neighbor) {
      uint8_t i=0;
      for (i=0;i<MAXNUMNEIGHBORS;i++) {
         if (isThisRowMatching(neighbor,i)) {
            return TRUE;
         }
      }
      return FALSE;
   }

   void registerNewNeighbor(open_addr_t* address) {
      open_addr_t temp_prefix;
      open_addr_t temp_addr16b;
      open_addr_t temp_addr64b;
      open_addr_t temp_addr128b;
      bool temp_used;
      uint8_t  i;
      if (address->type!=ADDR_16B && address->type!=ADDR_64B && address->type!=ADDR_128B) {
//         call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
//               (errorparameter_t)address->type,
//               (errorparameter_t)1);
         return;
      }
      if (isNeighbor(address)==FALSE) {
         i=0;
         while(i<MAXNUMNEIGHBORS) {
            atomic temp_used = neighbors[i].used;
            if (temp_used==FALSE) {
               atomic {
                  neighbors[i].used                   = TRUE;
                  neighbors[i].parentPreference       = 0;
                  neighbors[i].stableNeighbor         = FALSE;
                  neighbors[i].switchStabilityCounter = 0;
                  neighbors[i].addr_16b.type          = ADDR_NONE;
                  neighbors[i].addr_64b.type          = ADDR_NONE;
                  neighbors[i].addr_128b.type         = ADDR_NONE;
                  switch (address->type) {
                     case ADDR_16B:
                        call PacketFunctions.mac16bToMac64b(address,&temp_addr64b);
                        call PacketFunctions.mac64bToIp128b(
                              call IDManager.getMyID(ADDR_PREFIX),
                              &temp_addr64b,
                              &temp_addr128b);
                        memcpy(&neighbors[i].addr_16b,  address,        sizeof(open_addr_t));
                        memcpy(&neighbors[i].addr_64b,  &temp_addr64b,  sizeof(open_addr_t));
                        memcpy(&neighbors[i].addr_128b, &temp_addr128b, sizeof(open_addr_t));
                        break;
                     case ADDR_64B:
                        call PacketFunctions.mac64bToMac16b(address,&temp_addr16b);
                        call PacketFunctions.mac64bToIp128b(
                              call IDManager.getMyID(ADDR_PREFIX),
                              address,
                              &temp_addr128b);
                        memcpy(&neighbors[i].addr_16b,  &temp_addr16b,  sizeof(open_addr_t));
                        memcpy(&neighbors[i].addr_64b,  address,        sizeof(open_addr_t));
                        memcpy(&neighbors[i].addr_128b, &temp_addr128b, sizeof(open_addr_t));
                        break;
                     case ADDR_128B:
                        call PacketFunctions.ip128bToMac64b(
                              address,
                              &temp_prefix,
                              &temp_addr64b);
                        call PacketFunctions.mac64bToMac16b(&temp_addr64b,&temp_addr16b);
                        memcpy(&neighbors[i].addr_16b,  &temp_addr16b,  sizeof(open_addr_t));
                        memcpy(&neighbors[i].addr_64b,  &temp_addr64b,  sizeof(open_addr_t));
                        memcpy(&neighbors[i].addr_128b, address,        sizeof(open_addr_t));
                        break;
                  }
                  neighbors[i].DAGrank                = 255;
                  neighbors[i].linkQuality            = 0;
                  neighbors[i].numRx                  = 0;
                  neighbors[i].numTx                  = 0;
                  neighbors[i].numTxACK               = 0;
                  neighbors[i].timestamp              = call GlobalTime.getLocalTime();
               }
               break;
            }
            i++;
         }
         if (i==MAXNUMNEIGHBORS) {
//            call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_NEIGHBORS_FULL,MAXNUMNEIGHBORS,0);
            return;
         }
      }
   }

   bool isThisRowMatching(open_addr_t* address, uint8_t rowNumber) {
      switch (address->type) {
         case ADDR_16B:
            atomic return neighbors[rowNumber].used &&
               call PacketFunctions.sameAddress(address,&neighbors[rowNumber].addr_16b);
         case ADDR_64B:
            atomic return neighbors[rowNumber].used &&
               call PacketFunctions.sameAddress(address,&neighbors[rowNumber].addr_64b);
         case ADDR_128B:
            atomic return neighbors[rowNumber].used &&
               call PacketFunctions.sameAddress(address,&neighbors[rowNumber].addr_128b);
            break;
         default:
//            call OpenSerial.printError(COMPONENT_NEIGHBORS,ERR_WRONG_ADDR_TYPE,
//                  (errorparameter_t)address->type,
//                  (errorparameter_t)2);
            return FALSE;
      }
   };
}
