#ifndef ICMP_H
#define ICMP_H

typedef nx_struct ICMPv6_ht {
   nxle_uint8_t     type;
   nxle_uint8_t     code;
   nxle_uint16_t    checksum;
   nxle_uint16_t    identifier;
   nxle_uint16_t    sequence_number;
} ICMPv6_ht;

typedef nx_struct ICMPv6_RA_ht {
   nxle_uint8_t     type;
   nxle_uint8_t     code;
   nxle_uint16_t    checksum;
   nxle_uint8_t     hop_limit;
   nxle_uint8_t     flags;
   nxle_uint16_t    router_lifetime;
   nxle_uint32_t    reachable_time;
   nxle_uint32_t    retransmission_timer;
} ICMPv6_RA_ht;

typedef nx_struct ICMPv6_64bprefix_option_ht {
   nxle_uint8_t     option_type;
   nxle_uint8_t     option_length;
   nxle_uint8_t     prefix_length;
   nxle_uint8_t     flags;
   nxle_uint32_t    valid_lifetime;
   nxle_uint32_t    preferred_lifetime;
   nxle_uint32_t    unused;
   nxle_uint8_t     prefix[16];
} ICMPv6_64bprefix_option_ht;

#endif
