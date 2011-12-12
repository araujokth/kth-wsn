#include "OpenWSN.h"

interface PacketFunctions {
   //address translation
   async command void     ip128bToMac64b(open_addr_t* ip128b, open_addr_t* prefix64btoWrite, open_addr_t* mac64btoWrite);
   async command void     mac64bToIp128b(open_addr_t* prefix64b, open_addr_t* mac64b, open_addr_t* ip128bToWrite);
   async command void     mac64bToMac16b(open_addr_t* mac64b, open_addr_t* mac16btoWrite);
   async command void     mac16bToMac64b(open_addr_t* mac16b, open_addr_t* mac64btoWrite);
   //address recognition
   async command bool     isBroadcastMulticast(open_addr_t* address);
   async command bool     sameAddress(open_addr_t* address_1, open_addr_t* address_2);
   //read/write addresses
   async command void     readAddress(uint8_t* payload, uint8_t type, open_addr_t* writeToAddress, bool littleEndian);
   async command void     writeAddress(OpenQueueEntry_t* msg, open_addr_t* address, bool littleEndian);
   //reserving/tossing headers
   async command void     reserveHeaderSize(OpenQueueEntry_t* pkt, uint8_t header_length);
   async command void     tossHeader(OpenQueueEntry_t* pkt, uint8_t header_length);
   async command void     reserveFooterSize(OpenQueueEntry_t* pkt, uint8_t header_length);
   async command void     tossFooter(OpenQueueEntry_t* pkt, uint8_t header_length);
   //checksum calculation
   async command void     calculateChecksum(OpenQueueEntry_t* msg, uint8_t* checksum_ptr);
   //endianness
   async command void     htons(uint16_t val, uint8_t* dest);
   async command uint16_t ntohs(uint8_t *src);
   async command void     htonl(uint32_t val, uint8_t *dest);
   async command uint32_t ntohl(uint8_t *src);
   async command uint16_t onesComplementSum(uint8_t* ptr, int length, uint16_t previous_partial_checksum);
}
