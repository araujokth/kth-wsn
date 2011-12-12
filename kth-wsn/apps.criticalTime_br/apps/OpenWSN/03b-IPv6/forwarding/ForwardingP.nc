#include "OpenWSN.h"
#include "IPHC.h"

module ForwardingP {
   provides interface Init as SoftwareInit;
   //down the stack
   provides interface OpenSend as OpenSendFromUpper[uint8_t iana_number];
   uses     interface OpenSend as OpenSendToLower;
   //up the stack
   provides interface OpenReceiveIp;
   uses     interface OpenReceive as OpenReceiveToUpper[uint8_t iana_number];
   //misc
   uses interface PacketFunctions;
   uses interface Malloc;
   uses interface IDManager;
   uses interface NeighborGet;
   uses interface OpenSerial;
   provides interface DebugPrint;
}

implementation {
   /*-------------------------------- variables -----------------------------------------*/

   OpenQueueEntry_t* signalSendDoneMsg;
   error_t           signalSendDoneError;
   bool              signalSendDoneBusy;
   uint16_t icmpv6_seq=0;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void signalSendDoneTask();
   task void taskPrintDebug();
   void getNextHop(open_addr_t* destination, open_addr_t* addressToWrite);
   error_t rplsend(OpenQueueEntry_t *msg);

   /*-------------------------------- helper functions ----------------------------------*/

   task void signalSendDoneTask() {
      signal OpenSendFromUpper.sendDone[signalSendDoneMsg->l4_protocol](signalSendDoneMsg,signalSendDoneError);
      signalSendDoneBusy=FALSE;
   }

   task void taskPrintDebug() {
      //TBC
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   //SoftwareInit
   command error_t SoftwareInit.init() {
      signalSendDoneBusy=FALSE;
      return SUCCESS;
   }

   //OpenSendFromUpper
   command error_t OpenSendFromUpper.send[uint8_t iana_number](OpenQueueEntry_t *msg) {
      msg->owner = COMPONENT_FORWARDING;
      return rplsend(msg);
   }

   error_t rplsend(OpenQueueEntry_t *msg) {
      getNextHop(&(msg->l3_destinationORsource),&(msg->l2_nextORpreviousHop));
      if (msg->l2_nextORpreviousHop.type==ADDR_NONE) {
         call OpenSerial.printError(COMPONENT_FORWARDING,ERR_NO_NEXTHOP,0,0);
         return FAIL;
      }
      return call OpenSendToLower.send(msg);
   }

   void getNextHop(open_addr_t* destination128b, open_addr_t* addressToWrite64b) {
      open_addr_t temp_prefix64btoWrite;
      if (call NeighborGet.isStableNeighbor(destination128b)) {           //destination is 1-hop neighbor
         call PacketFunctions.ip128bToMac64b(destination128b,&temp_prefix64btoWrite,addressToWrite64b);
      } else {
         call NeighborGet.getPreferredParent(addressToWrite64b,ADDR_64B); //destination is remote
      }
   }

   //OpenSendToLower
   event void OpenSendToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_FORWARDING;
      if (signalSendDoneBusy==TRUE) {
         call OpenSerial.printError(COMPONENT_FORWARDING,ERR_BUSY_SENDING,0,0);
      }
      if (msg->creator==COMPONENT_CC2420RECEIVE) {//that was a packet I had relayed
         call Malloc.freePacketBuffer(msg);
      } else {//that was a packet coming from above
         signalSendDoneBusy  = TRUE;
         signalSendDoneMsg   = msg;
         signalSendDoneError = error;
         post signalSendDoneTask();
      }
   }

   //DebugPrint
   command void DebugPrint.print() {
      post taskPrintDebug();
   }

   //OpenReceiveIp
   command void OpenReceiveIp.receive(OpenQueueEntry_t* msg, ipv6_header_iht ipv6_header) {
      msg->owner = COMPONENT_FORWARDING;
      msg->l4_protocol = ipv6_header.next_header;
      if ( call IDManager.isMyAddress(&ipv6_header.dest) || call PacketFunctions.isBroadcastMulticast(&ipv6_header.dest)) {//for me
         memcpy(&(msg->l3_destinationORsource),&ipv6_header.src,sizeof(open_addr_t));
         call OpenReceiveToUpper.receive[msg->l4_protocol](msg);
      } else { //relay
         memcpy(&(msg->l3_destinationORsource),&ipv6_header.dest,sizeof(open_addr_t));//because initially contains source
         //resend as if from upper layer
         if (rplsend(msg)==FAIL) {
            call Malloc.freePacketBuffer(msg);
         }
      }
   }

   /*-------------------------------- defaults ------------------------------------------*/

   default event void OpenSendFromUpper.sendDone[uint8_t iana_number](OpenQueueEntry_t* msg, error_t error) {
      call OpenSerial.printError(COMPONENT_FORWARDING,ERR_WRONG_TRAN_PROTOCOL,iana_number,0);
      call Malloc.freePacketBuffer(msg);
   }

   default command void OpenReceiveToUpper.receive[uint8_t iana_number](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_FORWARDING,ERR_WRONG_TRAN_PROTOCOL,iana_number,0);
      call Malloc.freePacketBuffer(msg);
   }

}
