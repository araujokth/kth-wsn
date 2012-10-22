#ifndef __APP_PROFILE_H
#define __APP_PROFILE_H

#include <AM.h>

typedef nx_struct TDMA_frame_msg {
	
  nx_uint8_t	      options;
  nx_uint8_t          thl;
  nx_uint16_t         etx;
  nx_am_addr_t        origin;
  nx_uint8_t          originSeqNo;
  nx_uint8_t		  type;
  nx_am_addr_t 		  Node_ID;
  nx_uint16_t 		  val;
} TDMA_frame_msg;

enum {
  RADIO_CHANNEL = 0x10,
  PAN_ID = 0x1234,
  //COORDINATOR_ADDRESS = 0x0001,
  BEACON_ORDER = 6,
  SUPERFRAME_ORDER = 4,
  TX_POWER_BEACON = 0,
  TX_POWER = 0, // in dBm
};
#endif
