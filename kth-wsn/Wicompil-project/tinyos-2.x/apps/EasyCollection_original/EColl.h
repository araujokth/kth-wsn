#ifndef ECOLL_H
#define ECOLL_H

#include <AM.h>

typedef nx_struct ECollMessage {
  nx_am_addr_t source;
  //nx_uint16_t seqno;
  //nx_am_addr_t parent;
  //nx_uint16_t metric;
  //nx_uint16_t data;
  //nx_uint8_t hopcount;
} ECollMessage;


enum {
    MSGLEN = 2,
    MAXMSG = 9,
};

#endif
