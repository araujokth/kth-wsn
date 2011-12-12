#include "LATIN.h"
#include "IPHC.h"
#include "udp.h"

module UDPP {
   //down the stack
   provides interface OpenSend as OpenSendFromUpper[uint16_t localPortNumber];
   uses     interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   uses     interface OpenReceive as OpenReceiveToUpper[uint16_t localPortNumber];
   //misc
   uses     interface PacketFunctions;
   uses     interface IDManager;
   uses     interface OpenSerial;
   uses     interface Malloc;
}
implementation {

   //as per http://tools.ietf.org/html/rfc768

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t OpenSendFromUpper.send[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      msg->owner       = COMPONENT_UDP;
      msg->l4_protocol = IANA_UDP;
      msg->l4_payload  = msg->payload;
      msg->l4_length   = msg->length;
      call PacketFunctions.reserveHeaderSize(msg,sizeof(udp_ht));
      ((udp_ht*)msg->payload)->port_src  = msg->l4_sourcePortORicmpv6Type;
      ((udp_ht*)msg->payload)->port_dest = msg->l4_destination_port;
      ((udp_ht*)msg->payload)->length    = msg->length;
      call PacketFunctions.calculateChecksum(msg,(uint8_t*)&(((udp_ht*)msg->payload)->checksum));
      return call OpenSendToLower.send[IANA_UDP](msg);
   }

   event void OpenSendToLower.sendDone[uint8_t iana_number](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_UDP;
      signal OpenSendFromUpper.sendDone[msg->l4_sourcePortORicmpv6Type](msg,error);
   }

   command void OpenReceiveFromLower.receive[uint8_t iana_number](OpenQueueEntry_t* msg) {
      msg->owner                = COMPONENT_UDP;
      msg->l4_sourcePortORicmpv6Type       = ((udp_ht*)msg->payload)->port_src;
      msg->l4_destination_port  = ((udp_ht*)msg->payload)->port_dest;
      call PacketFunctions.tossHeader(msg,sizeof(udp_ht));
      call OpenReceiveToUpper.receive[msg->l4_destination_port](msg);
   }

   /*-------------------------------- defaults ------------------------------------------*/

   default event void OpenSendFromUpper.sendDone[uint16_t localPortNumber](OpenQueueEntry_t* msg, error_t error) {
      call OpenSerial.printError(COMPONENT_UDP,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,0);
   }
   default command void OpenReceiveToUpper.receive[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_UDP,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,1);
      call Malloc.freePacketBuffer(msg);
   }
   default command error_t OpenSendToLower.send[uint8_t iana_number](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_UDP,ERR_WRONG_TRAN_PROTOCOL,iana_number,0);
      return FAIL;
   }
}
