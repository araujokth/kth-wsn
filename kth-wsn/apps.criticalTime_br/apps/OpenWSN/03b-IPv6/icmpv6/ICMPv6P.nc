#include "OpenWSN.h"
#include "ICMPv6.h"

module ICMPv6P {
   provides interface Init as SoftwareInit;
   //down the stack
   provides interface OpenSend as OpenSendFromUpper[uint8_t icmpv6_type];
   uses interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   uses interface OpenReceive as OpenReceiveToUpper[uint8_t icmpv6_type];
   //misc
   uses interface PacketFunctions;
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface DebugPrint;
}

implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskPrintDebug();

   /*-------------------------------- helper functions ----------------------------------*/

   task void taskPrintDebug() {
      //TBC
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   //SoftwareInit
   command error_t SoftwareInit.init() {
      return SUCCESS;
   }

   //DebugPrint
   command void DebugPrint.print() {
      post taskPrintDebug();
   }

   //OpenSendFromUpper
   command error_t OpenSendFromUpper.send[uint8_t icmpv6_type](OpenQueueEntry_t* msg) {
      msg->owner    = COMPONENT_ICMPv6;
      //send
      return call OpenSendToLower.send[IANA_ICMPv6](msg);
   }

   //OpenSendToLower
   event void OpenSendToLower.sendDone[uint8_t iana_number](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_ICMPv6;
      signal OpenSendFromUpper.sendDone[msg->l4_sourcePortORicmpv6Type](msg, error);
   }

   //OpenReceiveFromLower
   command void OpenReceiveFromLower.receive[uint8_t iana_number](OpenQueueEntry_t* msg) {
      msg->owner = COMPONENT_ICMPv6;
      msg->l4_sourcePortORicmpv6Type = ((ICMPv6_ht*)(msg->payload))->type;
      call OpenReceiveToUpper.receive[msg->l4_sourcePortORicmpv6Type](msg);
   }

   /*-------------------------------- defaults ------------------------------------------*/

   default command error_t OpenSendToLower.send[uint8_t iana_number](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_ICMPv6,ERR_WRONG_TRAN_PROTOCOL,iana_number,0);
      return FAIL;
   }

   default event void OpenSendFromUpper.sendDone[uint8_t icmpv6_type](OpenQueueEntry_t* msg, error_t error) {
      call OpenSerial.printError(COMPONENT_ICMPv6,ERR_UNSUPPORTED_ICMPV6_TYPE,icmpv6_type,0);
      call Malloc.freePacketBuffer(msg);
   }

   default command void OpenReceiveToUpper.receive[uint8_t icmpv6_type](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_ICMPv6,ERR_UNSUPPORTED_ICMPV6_TYPE,icmpv6_type,1);
      call Malloc.freePacketBuffer(msg);
   }
}
