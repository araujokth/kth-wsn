#include "OpenWSN.h"
#include "IEEE802154.h"

module NoRESP {
   provides interface Init as SoftwareInit;
   //down the stack
   provides interface OpenSend as OpenSendFromUpper;
   provides interface OpenSend as OpenSendFromBridge;
   provides interface OpenSend as OpenSendKAFromNeighbors;
   uses     interface OpenSend as OpenSendToLower;
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower;
   uses     interface OpenReceive as OpenReceiveADVToNeighbor;
   uses     interface OpenReceive as OpenReceiveToUpper;
   uses     interface OpenReceive as OpenReceiveToBridge;
   //misc
   uses interface NeighborGet;                     //used for getMyDAGrank
   uses interface Malloc;
   uses interface OpenSerial;
   uses interface IDManager;
   provides interface DebugPrint;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskPrintDebug();
   error_t sendToLower(OpenQueueEntry_t *msg);

   /*-------------------------------- helper functions ----------------------------------*/
   
   task void taskPrintDebug() {
      uint16_t output=0;
      output = call NeighborGet.getMyDAGrank();
      call OpenSerial.printStatus(STATUS_ADVERTISEP_DAGRANK,(uint8_t*)&output,1);
   }
   /*-------------------------------- interfaces ----------------------------------------*/

   //SoftwareInit
   command error_t SoftwareInit.init() {
      return SUCCESS;
   }

   //OpenSendFromUpper
   command error_t OpenSendFromUpper.send(OpenQueueEntry_t *msg) {
      msg->owner = COMPONENT_RES;
      if (call IDManager.getIsBridge()==TRUE) {
         call OpenSerial.printError(COMPONENT_RES,ERR_BRIDGE_MISMATCH,0,0);
         return FAIL;
      }
      return sendToLower(msg);
   }

   //OpenSendFromBridge
   command error_t OpenSendFromBridge.send(OpenQueueEntry_t *msg) {
      msg->owner = COMPONENT_RES;
      if (call IDManager.getIsBridge()==FALSE) {
         call OpenSerial.printError(COMPONENT_RES,ERR_BRIDGE_MISMATCH,1,0);
         return FAIL;
      }
      return sendToLower(msg);
   }

   error_t sendToLower(OpenQueueEntry_t *msg) {
      msg->l2_frameType = IEEE154_TYPE_DATA;
      return call OpenSendToLower.send(msg);
   }

   //OpenSendKAFromNeighbors
   command error_t OpenSendKAFromNeighbors.send(OpenQueueEntry_t* msg) {
      msg->owner = COMPONENT_RES;
      call Malloc.freePacketBuffer(msg);
      return FAIL;
   }

   //OpenSendToLower
   event void OpenSendToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_RES;
      //this is a packet from upper layers
      if (call IDManager.getIsBridge()==TRUE) {
         signal OpenSendFromBridge.sendDone(msg,error);
      } else {
         signal OpenSendFromUpper.sendDone(msg,error);
      }
   }

   //OpenReceiveFromLower
   command void OpenReceiveFromLower.receive(OpenQueueEntry_t* msg) {
      msg->owner = COMPONENT_RES;
      switch (msg->l2_frameType) {
         case IEEE154_TYPE_DATA:
            if (call IDManager.getIsBridge()==TRUE) {
               call OpenReceiveToBridge.receive(msg);     //out to the OpenLBR stack
            } else {
               call OpenReceiveToUpper.receive(msg);      //up the internal stack
            }
            break;
         default:
            call Malloc.freePacketBuffer(msg);
            call OpenSerial.printError(COMPONENT_RES,ERR_MSG_UNKNOWN_TYPE,msg->l2_frameType,0);
            break;
      }
   }

   //DebugPrint
   command void DebugPrint.print(){
      post taskPrintDebug();
   }
}
