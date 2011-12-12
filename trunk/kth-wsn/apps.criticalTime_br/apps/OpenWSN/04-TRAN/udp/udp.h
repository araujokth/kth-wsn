#ifndef UDP_H
#define UDP_H

typedef nx_struct udp_ht {
   nx_uint16_t port_src;
   nx_uint16_t port_dest;
   nx_uint16_t length;
   nx_uint16_t checksum;
} udp_ht;

enum UDP_enums {
   UDP_ID        = 3,
   UDP_CHECKSUM  = 2,
   UDP_PORTS     = 0,
};

enum UDP_ID_enums {
   UDP_ID_DEFAULT = 0x1E,
};

enum UDP_CHECKSUM_enums {
   UDP_CHECKSUM_INLINE  = 0,
   UDP_CHECKSUM_ELIDED  = 1,
};

enum UDP_PORTS_enums {
   UDP_PORTS_16b_SRC_16b_DEST_INLINE = 0,
   UDP_PORTS_16b_SRC_8b_DEST_INLINE  = 1,
   UDP_PORTS_8b_SRC_16b_DEST_INLINE  = 2,
   UDP_PORTS_4b_SRC_4b_DEST_INLINE   = 3,
};

#endif
