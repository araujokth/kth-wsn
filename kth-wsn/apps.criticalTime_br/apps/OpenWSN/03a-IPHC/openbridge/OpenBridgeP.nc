#include "OpenWSN.h"

module OpenBridgeP {
   //down the stack
   provides interface Trigger as TriggerOpenBridge;
   uses     interface OpenSend as OpenSendToLower;
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower;
   //misc
   uses     interface OpenSerial;  //also to exchange the data
   uses     interface IDManager;
   uses     interface Malloc;
   uses     interface PacketFunctions;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- prototypes ----------------------------------------*/

   /*-------------------------------- interfaces ----------------------------------------*/

   /*-------------------------------- helper functions ----------------------------------*/

   //Trigger
   command void TriggerOpenBridge.trigger() {
      uint8_t number_bytes_from_input_buffer;
      uint8_t input_buffer[136];//worst case: 8B of next hop + 128B of data
      OpenQueueEntry_t* pkt;
      uint8_t numDataBytes;
      numDataBytes = call OpenSerial.getNumDataBytes();
      //get command from OpenSerial (16B IPv6 destination address, 2B destination port)
      number_bytes_from_input_buffer = call OpenSerial.getInputBuffer(&(input_buffer[0]),numDataBytes);
      if (call IDManager.getIsBridge()==TRUE && numDataBytes>0) {
         pkt = call Malloc.getFreePacketBuffer();
         if (pkt==NULL) {
            call OpenSerial.printError(COMPONENT_OPENBRIDGE,ERR_NO_FREE_PACKET_BUFFER,
                  (errorparameter_t)0,
                  (errorparameter_t)0);
            return;
         }
         //admin
         pkt->creator  = COMPONENT_OPENBRIDGE;
         pkt->owner    = COMPONENT_OPENBRIDGE;
         //l2
         pkt->l2_nextORpreviousHop.type = ADDR_64B;
         memcpy(&(pkt->l2_nextORpreviousHop.addr_64b[0]),&(input_buffer[0]),8);
         //payload
         call PacketFunctions.reserveHeaderSize(pkt,numDataBytes-8);
         memcpy(pkt->payload,&(input_buffer[8]),numDataBytes-8);
         //send
         if ((call OpenSendToLower.send(pkt))==FAIL) {
            call Malloc.freePacketBuffer(pkt);
         }
      }
   }

   //OpenSendToLower
   event void OpenSendToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_OPENBRIDGE;
      if (msg->creator!=COMPONENT_OPENBRIDGE) {
         call OpenSerial.printError(COMPONENT_OPENBRIDGE,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,
               (errorparameter_t)0,
               (errorparameter_t)0);
      }
      call Malloc.freePacketBuffer(msg);
   }

   //OpenReceiveFromLower
   command void OpenReceiveFromLower.receive(OpenQueueEntry_t* msg) {
      call OpenSerial.printData((uint8_t*)(msg->payload),msg->length);
      call Malloc.freePacketBuffer(msg);
   }

   /*-------------------------------- helper functions ----------------------------------*/
}
