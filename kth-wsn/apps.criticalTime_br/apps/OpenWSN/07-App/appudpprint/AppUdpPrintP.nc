#include "OpenWSN.h"

module AppUdpPrintP {
   //admin
   provides interface Init as SoftwareInit;
   //data flow
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- receive sequence ----------------------------------*/

   command void OpenReceiveFromLower.receive[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printData((uint8_t*)(msg->payload),msg->length);
      call Malloc.freePacketBuffer(msg);
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      return SUCCESS;
   }
}
