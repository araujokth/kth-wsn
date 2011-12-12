#include "OpenWSN.h"

module AppUdpEchoP {
   //admin
   provides interface Init as SoftwareInit;
   //data flow
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- receive sequence ----------------------------------*/

   command void OpenReceiveFromLower.receive[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      msg->owner   = COMPONENT_APPUDPECHO;
      //reply with the same OpenQueueEntry_t
      msg->creator               = COMPONENT_APPUDPECHO;
      msg->l4_protocol           = IANA_UDP;
      msg->l4_destination_port   = msg->l4_sourcePortORicmpv6Type;
      msg->l4_sourcePortORicmpv6Type        = localPortNumber;
      if ((call OpenSendToLower.send[localPortNumber](msg))==FAIL) {
         call Malloc.freePacketBuffer(msg);
      }
   }
   default command error_t OpenSendToLower.send[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_APPUDPECHO,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,0);
      return FAIL;
   }
   event void OpenSendToLower.sendDone[uint16_t localPortNumber](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_APPUDPECHO;
      if (msg->creator!=COMPONENT_APPUDPECHO) {
         call OpenSerial.printError(COMPONENT_APPUDPECHO,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      call Malloc.freePacketBuffer(msg);
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      return SUCCESS;
   }
}
