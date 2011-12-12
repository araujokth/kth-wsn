#include "OpenWSN.h"

module AppTcpInjectP {
   //admin
   provides interface Init as SoftwareInit;
   //data flow
   uses interface TCPControl[uint16_t localPortNumber];
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   uses interface PacketFunctions;
   provides interface Trigger as TriggerTCPInject;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   OpenQueueEntry_t* pkt;
   bool              sending;
   open_addr_t       hisAddress;
   uint16_t          hisPort;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskConnect();
   task void taskSend();
   task void taskClose();

   /*-------------------------------- receive sequence ----------------------------------*/

   command void TriggerTCPInject.trigger() {
      uint8_t number_bytes_from_input_buffer;
      uint8_t input_buffer[18];
      //get command from OpenSerial (16B IPv6 destination address, 2B destination port)
      number_bytes_from_input_buffer = call OpenSerial.getInputBuffer(&(input_buffer[0]),sizeof(input_buffer));
      if (number_bytes_from_input_buffer!=sizeof(input_buffer)) {
         call OpenSerial.printError(COMPONENT_APPTCPINJECT,ERR_INPUTBUFFER_LENGTH,
               (errorparameter_t)number_bytes_from_input_buffer,
               (errorparameter_t)0);
         return;
      };
      hisAddress.type = ADDR_128B;
      memcpy(&(hisAddress.addr_128b[0]),&(input_buffer[0]),16);
      hisPort = call PacketFunctions.ntohs(&(input_buffer[16]));
      //connect
      post taskConnect();
   }

   task void taskConnect() {
      call TCPControl.connect[WKP_TCP_INJECT](&hisAddress,hisPort);
   }

   event void TCPControl.connectDone[uint16_t localPortNumber](error_t error)
   {
      if (error==SUCCESS) {
         post taskSend();
      }
   }

   task void taskSend() {
      pkt = call Malloc.getFreePacketBuffer();
      if (pkt==NULL) {
         call OpenSerial.printError(COMPONENT_APPTCPINJECT,ERR_NO_FREE_PACKET_BUFFER,(errorparameter_t)0,(errorparameter_t)0);
         return;
      }
      pkt->creator                      = COMPONENT_APPTCPINJECT;
      pkt->owner                        = COMPONENT_APPTCPINJECT;
      pkt->l4_protocol                  = IANA_UDP;
      pkt->l4_sourcePortORicmpv6Type    = WKP_TCP_INJECT;
      pkt->l4_destination_port          = hisPort;
      memcpy(&(pkt->l3_destinationORsource),&hisAddress,sizeof(open_addr_t));
      call PacketFunctions.reserveHeaderSize(pkt,6);
      ((uint8_t*)pkt->payload)[0] = 'p';
      ((uint8_t*)pkt->payload)[1] = 'o';
      ((uint8_t*)pkt->payload)[2] = 'i';
      ((uint8_t*)pkt->payload)[3] = 'p';
      ((uint8_t*)pkt->payload)[4] = 'o';
      ((uint8_t*)pkt->payload)[5] = 'i';
      if ((call OpenSendToLower.send[WKP_TCP_INJECT](pkt))==FAIL) {
         call Malloc.freePacketBuffer(pkt);
      }
      return;
   }

   event void OpenSendToLower.sendDone[uint16_t localPortNumber](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_APPTCPINJECT;
      if (msg->creator!=COMPONENT_APPTCPINJECT) {
         call OpenSerial.printError(COMPONENT_APPTCPINJECT,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      post taskClose();
      call Malloc.freePacketBuffer(msg);
   }

   task void taskClose() {
      call TCPControl.close[WKP_TCP_INJECT]();
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      return SUCCESS;
   }

   /*-------------------------------- defaults ------------------------------------------*/

   default command error_t TCPControl.connect[uint16_t localPortNumber](open_addr_t* dest, uint16_t temp_hisPort) {
      call OpenSerial.printError(COMPONENT_APPTCPINJECT,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,0);
      return FAIL;
   }

   default command error_t OpenSendToLower.send[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_APPTCPINJECT,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,1);
      return FAIL;
   }

   default command error_t TCPControl.close[uint16_t localPortNumber]() {
      call OpenSerial.printError(COMPONENT_APPTCPINJECT,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,2);
      return FAIL;
   }

   event bool TCPControl.shouldIlisten[uint16_t localPortNumber](){
      return FALSE;
   }

}
