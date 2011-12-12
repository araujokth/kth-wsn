#include "LATIN.h"

module AppUdpInjectP {
   //admin
   provides interface Init as SoftwareInit;
   //data flow
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   uses interface PacketFunctions;
   provides interface Trigger as TriggerUDPInject;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   OpenQueueEntry_t* pkt;
   bool              sending;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskSend();

   /*-------------------------------- receive sequence ----------------------------------*/

   command void TriggerUDPInject.trigger() {
      post taskSend();
   }

   task void taskSend() {
      uint8_t number_bytes_from_input_buffer;
      uint8_t input_buffer[18];
      //get command from OpenSerial (16B IPv6 destination address, 2B destination port)
      number_bytes_from_input_buffer = call OpenSerial.getInputBuffer(&(input_buffer[0]),sizeof(input_buffer));
      if (number_bytes_from_input_buffer!=sizeof(input_buffer)) {
         call OpenSerial.printError(COMPONENT_APPUDPINJECT,ERR_INPUTBUFFER_LENGTH,
               (errorparameter_t)number_bytes_from_input_buffer,
               (errorparameter_t)0);
         return;
      };
      //prepare packet
      pkt = call Malloc.getFreePacketBuffer();
      if (pkt==NULL) {
         call OpenSerial.printError(COMPONENT_APPUDPINJECT,ERR_NO_FREE_PACKET_BUFFER,(errorparameter_t)0,(errorparameter_t)0);
         return;
      }
      pkt->creator                     = COMPONENT_APPUDPINJECT;
      pkt->owner                       = COMPONENT_APPUDPINJECT;
      pkt->l4_protocol                 = IANA_UDP;
      pkt->l4_sourcePortORicmpv6Type   = WKP_UDP_INJECT;
      pkt->l4_destination_port         = call PacketFunctions.ntohs(&(input_buffer[16]));
      pkt->l3_destinationORsource.type = ADDR_128B;
      memcpy(&(pkt->l3_destinationORsource.addr_128b[0]),&(input_buffer[0]),16);
      call PacketFunctions.reserveHeaderSize(pkt,6);
      ((uint8_t*)pkt->payload)[0]      = 'p';
      ((uint8_t*)pkt->payload)[1]      = 'o';
      ((uint8_t*)pkt->payload)[2]      = 'i';
      ((uint8_t*)pkt->payload)[3]      = 'p';
      ((uint8_t*)pkt->payload)[4]      = 'o';
      ((uint8_t*)pkt->payload)[5]      = 'i';
      //send packet
      if ((call OpenSendToLower.send[WKP_UDP_INJECT](pkt))==FAIL) {
         call Malloc.freePacketBuffer(pkt);
      }
      return;
   }

   default command error_t OpenSendToLower.send[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_APPUDPINJECT,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,0);
      return FAIL;
   }

   event void OpenSendToLower.sendDone[uint16_t localPortNumber](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_APPUDPINJECT;
      if (msg->creator!=COMPONENT_APPUDPINJECT) {
         call OpenSerial.printError(COMPONENT_APPUDPINJECT,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      call Malloc.freePacketBuffer(msg);
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      return SUCCESS;
   }
}
