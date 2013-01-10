#ifndef ECOLL_H
#define ECOLL_H

#include <AM.h>

typedef nx_struct ECollMessage {
  nx_am_addr_t seqno;
} ECollMessage;


enum {
    MSGLEN = 2,
    MAXMSG = 9,
};

#endif
