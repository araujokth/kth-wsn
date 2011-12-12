#include "LATIN.h"
#include "UserButton.h"
#include "IPHC.h" //for IPv6 next header

module AppTcpEchoP {
   //admin
   provides interface Init as SoftwareInit;
   //data flow
   uses interface TCPControl[uint16_t localPortNumber];
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   uses interface PacketFunctions;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskCloseTCPSession();

   /*-------------------------------- http receive sequence -----------------------------*/

   event bool TCPControl.shouldIlisten[uint16_t localPortNumber](){
      return TRUE;
   }

   command void OpenReceiveFromLower.receive[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      msg->owner   = COMPONENT_APPTCPECHO;
      //reply with the same OpenQueueEntry_t
      msg->creator               = COMPONENT_APPTCPECHO;
      msg->l4_protocol           = IANA_TCP;
      msg->l4_destination_port   = msg->l4_sourcePortORicmpv6Type;
      msg->l4_sourcePortORicmpv6Type        = localPortNumber;
      if ((call OpenSendToLower.send[localPortNumber](msg))==FAIL) {
         call Malloc.freePacketBuffer(msg);
      }
   }
   default command error_t OpenSendToLower.send[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_APPTCPECHO,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,0);
      return FAIL;
   }

   event void OpenSendToLower.sendDone[uint16_t localPortNumber](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_APPTCPECHO;
      if (msg->creator!=COMPONENT_APPTCPECHO) {
         call OpenSerial.printError(COMPONENT_APPTCPECHO,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      post taskCloseTCPSession();
      call Malloc.freePacketBuffer(msg);
   }

   task void taskCloseTCPSession() {
      //close TCP session, but keep listening
      call TCPControl.close[WKP_TCP_ECHO]();
   }

   default command error_t TCPControl.close[uint16_t localPortNumber]() {
      call OpenSerial.printError(COMPONENT_APPTCPECHO,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,1);
      return FAIL;
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      return SUCCESS;
   }

   event void TCPControl.connectDone[uint16_t localPortNumber](error_t error) {
   }
}
