#include "LATIN.h"
#include "IEEE802154.h"
//#include "IPHC.h"

module PacketFunctionsP {
   provides interface PacketFunctions;
//   uses interface OpenSerial;
   uses interface IDManager;
   uses interface Leds;
}
implementation {

   /*------------------------ prototypes ----------------------------------------------*/

   void onesComplementSum(uint8_t* global_sum, uint8_t* ptr, int length);

   /*------------------------ address translation -------------------------------------*/

   //assuming an ip128b is a concatenation of prefix64b followed by a mac64b
   async command void PacketFunctions.ip128bToMac64b(
         open_addr_t* ip128b,
         open_addr_t* prefix64btoWrite,
         open_addr_t* mac64btoWrite) {
      if (ip128b->type!=ADDR_128B) {
//         call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
//               (errorparameter_t)ip128b->type,
//               (errorparameter_t)0);
         mac64btoWrite->type=ADDR_NONE;
         return;
      }
      prefix64btoWrite->type=ADDR_PREFIX;
      memcpy(prefix64btoWrite->prefix, &(ip128b->addr_128b[0]), 8);
      mac64btoWrite->type=ADDR_64B;
      memcpy(mac64btoWrite->addr_64b , &(ip128b->addr_128b[8]), 8);
   }
   async command void PacketFunctions.mac64bToIp128b(
         open_addr_t* prefix64b,
         open_addr_t* mac64b,
         open_addr_t* ip128bToWrite) {
      if (prefix64b->type!=ADDR_PREFIX || mac64b->type!=ADDR_64B) {
//         call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
//               (errorparameter_t)prefix64b->type,
//               (errorparameter_t)1);
         ip128bToWrite->type=ADDR_NONE;
         return;
      }
      ip128bToWrite->type=ADDR_128B;
      memcpy(&(ip128bToWrite->addr_128b[0]), &(prefix64b->prefix[0]), 8);
      memcpy(&(ip128bToWrite->addr_128b[8]), &(mac64b->addr_64b[0]),  8);
   }

   //assuming an mac16b is lower 2B of mac64b
   async command void PacketFunctions.mac64bToMac16b(open_addr_t* mac64b, open_addr_t* mac16btoWrite) {
      if (mac64b->type!=ADDR_64B) {
//         call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
//               (errorparameter_t)mac64b->type,
//               (errorparameter_t)2);
         mac16btoWrite->type=ADDR_NONE;
         return;
      }
      mac16btoWrite->type = ADDR_16B;
      mac16btoWrite->addr_16b[0] = mac64b->addr_64b[6];
      mac16btoWrite->addr_16b[1] = mac64b->addr_64b[7];
   }
   async command void PacketFunctions.mac16bToMac64b(open_addr_t* mac16b, open_addr_t* mac64btoWrite) {
      if (mac16b->type!=ADDR_16B) {
//         call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
//               (errorparameter_t)mac16b->type,
//               (errorparameter_t)3);
         mac64btoWrite->type=ADDR_NONE;
         return;
      }
      mac64btoWrite->type = ADDR_64B;
      mac64btoWrite->addr_64b[0] = 0;
      mac64btoWrite->addr_64b[1] = 0;
      mac64btoWrite->addr_64b[2] = 0;
      mac64btoWrite->addr_64b[3] = 0;
      mac64btoWrite->addr_64b[4] = 0;
      mac64btoWrite->addr_64b[5] = 0;
      mac64btoWrite->addr_64b[6] = mac16b->addr_16b[0];
      mac64btoWrite->addr_64b[7] = mac16b->addr_16b[1];
   }

   /*------------------------ address recognition -------------------------------------*/

   async command bool PacketFunctions.isBroadcastMulticast(open_addr_t* address) {
      uint8_t i;
      uint8_t address_length;
      //IPv6 multicast
      if (address->type==ADDR_128B) {
         if (address->addr_128b[0]==0xff) {
            return TRUE;
         } else {
            return FALSE;
         }
      }
      //15.4 broadcast
      switch (address->type) {
         case ADDR_16B:
            address_length = 2;
            break;
         case ADDR_64B:
            address_length = 8;
            break;
         default:
//            call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
//                  (errorparameter_t)address->type,
//                  (errorparameter_t)4);

            return FALSE;
      }
      for (i=0;i<address_length;i++) {
         if (address->addr_128b[i]!=0xFF) {

        	 return FALSE;
         }
      }
      return TRUE;
   }

   async command bool PacketFunctions.sameAddress(open_addr_t* address_1, open_addr_t* address_2) {
      uint8_t address_length;
      if (address_1->type!=address_2->type) {
         return FALSE;
      }
      switch (address_1->type) {
         case ADDR_16B:
         case ADDR_PANID:
            address_length = 2;
            break;
         case ADDR_64B:
         case ADDR_PREFIX:
            address_length = 8;
            break;
         case ADDR_128B:
            address_length = 16;
            break;
         default:
//            call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
//                  (errorparameter_t)address_1->type,
//                  (errorparameter_t)5);
            return FALSE;
      }
      if (memcmp((void*)address_1->addr_128b,(void*)address_2->addr_128b,address_length)==0) {
         return TRUE;
      }
      return FALSE;
   }

   /*------------------------ read/write addresses ------------------------------------*/

   async command void PacketFunctions.readAddress(uint8_t* payload, uint8_t type, open_addr_t* writeToAddress, bool littleEndian) {
      uint8_t i;
      uint8_t address_length;
      writeToAddress->type = type;
      switch (type) {
         case ADDR_16B:
         case ADDR_PANID:
            address_length = 2;
            break;
         case ADDR_64B:
         case ADDR_PREFIX:
            address_length = 8;
            break;
         case ADDR_128B:
            address_length = 16;
            break;
         default:
//            call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
//                  (errorparameter_t)type,
//                  (errorparameter_t)6);
            return;
      }
      for (i=0;i<address_length;i++) {
         if (littleEndian) {
            writeToAddress->addr_128b[address_length-1-i] = *(payload+i);
         } else {
            writeToAddress->addr_128b[i]   = *(payload+i);
         }
      }
   }

   async command void PacketFunctions.writeAddress(OpenQueueEntry_t* msg, open_addr_t* address, bool littleEndian) {
      uint8_t i;
      uint8_t address_length;
      switch (address->type) {
         case ADDR_16B:
         case ADDR_PANID:
            address_length = 2;
            break;
         case ADDR_64B:
         case ADDR_PREFIX:
            address_length = 8;
            break;
         case ADDR_128B:
            address_length = 16;
            break;
         default:
//            call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_WRONG_ADDR_TYPE,
//                  (errorparameter_t)address->type,
//                  (errorparameter_t)7);
            return;
      }
      for (i=0;i<address_length;i++) {
         msg->payload      -= sizeof(uint8_t);
         msg->length       += sizeof(uint8_t);
         if (littleEndian) {
            *((uint8_t*)(msg->payload)) = address->addr_128b[i];
         } else {
            *((uint8_t*)(msg->payload)) = address->addr_128b[address_length-1-i];
         }
      }
   }

   /*------------------------ reserving/tossing headers -------------------------------*/

   async command void PacketFunctions.reserveHeaderSize(OpenQueueEntry_t* pkt, uint8_t header_length) {
      pkt->payload -= header_length;
      pkt->length  += header_length;
      if ( (uint8_t*)(pkt->payload) < (uint8_t*)(pkt->packet) ) {
//         call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_HEADER_TOO_LONG,
//               (errorparameter_t)0,
//               (errorparameter_t)pkt->length);
      }
   }
   async command void PacketFunctions.tossHeader(OpenQueueEntry_t* pkt, uint8_t header_length) {
      pkt->payload += header_length;
      pkt->length  -= header_length;
      if ( (uint8_t*)(pkt->payload) > (uint8_t*)(pkt->packet+126) ) {
//         call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_HEADER_TOO_LONG,
//               (errorparameter_t)1,
//               (errorparameter_t)pkt->length);
      }
   }

   async command void PacketFunctions.reserveFooterSize(OpenQueueEntry_t* pkt, uint8_t header_length) {
      pkt->length  += header_length;
      if (pkt->length>127) {
//         call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_HEADER_TOO_LONG,
//               (errorparameter_t)2,
//               (errorparameter_t)pkt->length);
      }
   }
   async command void PacketFunctions.tossFooter(OpenQueueEntry_t* pkt, uint8_t header_length) {
      pkt->length  -= header_length;
      if (pkt->length>128) {//wraps around, so a negative value will be >128
//         call OpenSerial.printError(COMPONENT_PACKETFUNCTIONS,ERR_HEADER_TOO_LONG,
//               (errorparameter_t)3,
//               (errorparameter_t)pkt->length);
      }
   }

   /*------------------------ checksum calculation ------------------------------------*/
//fabien added this line because it was giving me the error: not implemented.
   async command void PacketFunctions.calculateChecksum(OpenQueueEntry_t* msg, uint8_t* checksum_ptr){}


   //see http://www-net.cs.umass.edu/kurose/transport/UDP.html, or http://tools.ietf.org/html/rfc1071
   async command uint16_t PacketFunctions.onesComplementSum(uint8_t* ptr, int length, uint16_t previous_partial_checksum) {
      
   uint32_t tempSum = 0; //this is a working variable
   tempSum |= previous_partial_checksum; //store the previous value in the 32 bit register


   while (length > 1){
      tempSum += (uint16_t) ptr++;
      length -= 2;
   }

   //if the number of bytes is odd this will be satisfied:
   if (length > 0)
      tempSum += (uint16_t) ptr;


   //go from 32 to 16 bits:
   while (tempSum>>16)
      tempSum = (tempSum & 0xffff) + (tempSum >> 16);

   previous_partial_checksum = ~tempSum; //this now holds the new checksum
   return previous_partial_checksum;
   }

   /*------------------------ endianness ----------------------------------------------*/

   async command void PacketFunctions.htons( uint16_t val, uint8_t* dest ) {
      dest[0] = (val & 0xff00) >> 8;
      dest[1] = (val & 0x00ff);
   }
   async command uint16_t PacketFunctions.ntohs( uint8_t* src ) {
      return (((uint16_t) src[0]) << 8) |
         (((uint16_t) src[1])
         );
   }
   async command void PacketFunctions.htonl( uint32_t val, uint8_t* dest ) {
      dest[0] = (val & 0xff000000) >> 24;
      dest[1] = (val & 0x00ff0000) >> 16;
      dest[2] = (val & 0x0000ff00) >> 8;
      dest[3] = (val & 0x000000ff);
   }
   async command uint32_t PacketFunctions.ntohl( uint8_t* src ) {
      return (((uint32_t) src[0]) << 24) |
         (((uint32_t) src[1]) << 16)     |
         (((uint32_t) src[2]) << 8)      |
         (((uint32_t) src[3])
         );
   }
}
