#ifndef  APP_PROFILE_H
#define  APP_PROFILE_H

#include <AM.h>

typedef nx_struct TDMA_frame_msg {
  nx_am_addr_t Node_ID;
  nx_uint16_t val;
} TDMA_frame_msg;


enum {
  RADIO_CHANNEL = 0x10,
  PAN_ID = 0x1234,
  COORDINATOR_ADDRESS = 0x0000,
  BEACON_ORDER = 6,
  SUPERFRAME_ORDER = 6,
  TX_POWER_BEACON = 0,
  TX_POWER = 0, // in dBm
};


#endif
