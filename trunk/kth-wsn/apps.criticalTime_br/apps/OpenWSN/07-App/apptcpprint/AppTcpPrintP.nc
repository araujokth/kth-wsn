#include "OpenWSN.h"

module AppTcpPrintP {
   //admin
   provides interface Init as SoftwareInit;
   //data flow
   uses interface TCPControl[uint16_t localPortNumber];
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskCloseTCPSession();

   /*-------------------------------- receive sequence ----------------------------------*/

   event bool TCPControl.shouldIlisten[uint16_t localPortNumber](){
      return TRUE;
   }

   command void OpenReceiveFromLower.receive[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printData((uint8_t*)(msg->payload),msg->length);
      post taskCloseTCPSession();
      call Malloc.freePacketBuffer(msg);
   }

   task void taskCloseTCPSession() {
      //close TCP session, but keep listening
      call TCPControl.close[WKP_TCP_DISCARD]();
   }

   default command error_t TCPControl.close[uint16_t localPortNumber]() {
      call OpenSerial.printError(COMPONENT_APPTCPPRINT,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,1);
      return FAIL;
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      return SUCCESS;
   }

   event void TCPControl.connectDone[uint16_t localPortNumber](error_t error) {
   }
}
