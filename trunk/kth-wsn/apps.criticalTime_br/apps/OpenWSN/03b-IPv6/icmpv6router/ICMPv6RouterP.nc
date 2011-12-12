#include "OpenWSN.h"
#include "ICMPv6.h"

module ICMPv6RouterP {
   provides interface Init as SoftwareInit;
   //down the stack
   uses interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   //misc
   uses interface PacketFunctions;
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger as TriggerICMPv6Router;
   provides interface DebugPrint;
}

implementation {
   /*-------------------------------- variables -----------------------------------------*/

   bool busySending;
   open_addr_t hisAddress;
   uint16_t icmpv6_seq=0;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskPrintDebug();
   task void taskSendRouterSolicitation();

   /*-------------------------------- helper functions ----------------------------------*/

   task void taskPrintDebug() {
      //TBC
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   //SoftwareInit
   command error_t SoftwareInit.init() {
      busySending = FALSE;
      return SUCCESS;
   }

   //TriggerICMPv6Router
   command void TriggerICMPv6Router.trigger() {
      uint8_t number_bytes_from_input_buffer;
      uint8_t input_buffer[16];
      //get command from OpenSerial (16B IPv6 destination address)
      number_bytes_from_input_buffer = call OpenSerial.getInputBuffer(&(input_buffer[0]),sizeof(input_buffer));
      if (number_bytes_from_input_buffer!=sizeof(input_buffer)) {
         call OpenSerial.printError(COMPONENT_ICMPv6ECHO,ERR_INPUTBUFFER_LENGTH,
               (errorparameter_t)number_bytes_from_input_buffer,
               (errorparameter_t)0);
         return;
      };
      hisAddress.type  = ADDR_128B;
      memcpy(&(hisAddress.addr_128b[0]),&(input_buffer[0]),16);
      //send
      if (busySending==TRUE) {
         call OpenSerial.printError(COMPONENT_ICMPv6ROUTER,ERR_BUSY_SENDING,0,0);
      } else {
         busySending = TRUE;
         post taskSendRouterSolicitation();
      }
   }
   task void taskSendRouterSolicitation() {
      OpenQueueEntry_t* msg;
      msg = call Malloc.getFreePacketBuffer();
      if (msg==NULL) {
         call OpenSerial.printError(COMPONENT_ICMPv6ROUTER,ERR_NO_FREE_PACKET_BUFFER,(errorparameter_t)0,(errorparameter_t)0);
         return;
      }
      //admin
      msg->creator                               = COMPONENT_ICMPv6ROUTER;
      msg->owner                                 = COMPONENT_ICMPv6ROUTER;
      //l4
      msg->l4_protocol                           = IANA_ICMPv6;
      msg->l4_sourcePortORicmpv6Type             = IANA_ICMPv6_RS;
      //l3
      msg->l3_destinationORsource.type           = ADDR_128B;
      msg->l3_destinationORsource.addr_128b[0]   = 0xff;
      msg->l3_destinationORsource.addr_128b[1]   = 0x02;
      msg->l3_destinationORsource.addr_128b[2]   = 0x00;
      msg->l3_destinationORsource.addr_128b[3]   = 0x00;
      msg->l3_destinationORsource.addr_128b[4]   = 0x00;
      msg->l3_destinationORsource.addr_128b[5]   = 0x00;
      msg->l3_destinationORsource.addr_128b[6]   = 0x00;
      msg->l3_destinationORsource.addr_128b[7]   = 0x00;
      msg->l3_destinationORsource.addr_128b[8]   = 0x00;
      msg->l3_destinationORsource.addr_128b[9]   = 0x00;
      msg->l3_destinationORsource.addr_128b[10]  = 0x00;
      msg->l3_destinationORsource.addr_128b[11]  = 0x00;
      msg->l3_destinationORsource.addr_128b[12]  = 0x00;
      msg->l3_destinationORsource.addr_128b[13]  = 0x00;
      msg->l3_destinationORsource.addr_128b[14]  = 0x00;
      msg->l3_destinationORsource.addr_128b[15]  = 0x02;
      //ICMPv6 header
      call PacketFunctions.reserveHeaderSize(msg,sizeof(ICMPv6_ht));
      ((ICMPv6_ht*)(msg->payload))->type         = msg->l4_sourcePortORicmpv6Type;
      ((ICMPv6_ht*)(msg->payload))->code         = 0;
      call PacketFunctions.htons(0x1234       ,(uint8_t*)&((ICMPv6_ht*)(msg->payload))->identifier);
      call PacketFunctions.htons(icmpv6_seq++ ,(uint8_t*)&((ICMPv6_ht*)(msg->payload))->sequence_number); 
      call PacketFunctions.calculateChecksum(msg,(uint8_t*)&(((ICMPv6_ht*)(msg->payload))->checksum));//call last
      //send
      if (call OpenSendToLower.send[msg->l4_sourcePortORicmpv6Type](msg)!=SUCCESS) {
         busySending = FALSE;
         call Malloc.freePacketBuffer(msg);
      }
   }

   //OpenSendToLower
   event void OpenSendToLower.sendDone[uint8_t iana_number](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_ICMPv6ROUTER;
      if (msg->creator!=COMPONENT_ICMPv6ROUTER) {//that was a packet I had not created
         call OpenSerial.printError(COMPONENT_ICMPv6ROUTER,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      call Malloc.freePacketBuffer(msg);
      busySending = FALSE;
   }

   //DebugPrint
   command void DebugPrint.print() {
      post taskPrintDebug();
   }

   //OpenReceiveFromLower
   command void OpenReceiveFromLower.receive[uint8_t icmpv6_type](OpenQueueEntry_t* msg) {
      open_addr_t temp_prefix;
      msg->owner = COMPONENT_ICMPv6ROUTER;
      //toss ICMPv6 header
      call PacketFunctions.tossHeader(msg,sizeof(ICMPv6_RA_ht));
      //record prefix
      if (  ((ICMPv6_64bprefix_option_ht*)(msg->payload))->option_type   == IANA_ICMPv6_RA_PREFIX_INFORMATION &&
            ((ICMPv6_64bprefix_option_ht*)(msg->payload))->option_length == 4                                 &&
            ((ICMPv6_64bprefix_option_ht*)(msg->payload))->prefix_length == 64                                &&
            msg->length>=sizeof(ICMPv6_64bprefix_option_ht) ) {
         temp_prefix.type = ADDR_PREFIX;
         memcpy(  &(temp_prefix.prefix[0]),
               &(((ICMPv6_64bprefix_option_ht*)(msg->payload))->prefix[0]),
               8);
         call IDManager.setMyID(&temp_prefix);
      }
      //toss packet
      call Malloc.freePacketBuffer(msg);
   }

   /*-------------------------------- defaults ------------------------------------------*/

   default command error_t OpenSendToLower.send[uint8_t icmpv6_type](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_ICMPv6ROUTER,ERR_UNSUPPORTED_ICMPV6_TYPE,
            (errorparameter_t)icmpv6_type,
            (errorparameter_t)1);
      return FAIL;
   }
}
