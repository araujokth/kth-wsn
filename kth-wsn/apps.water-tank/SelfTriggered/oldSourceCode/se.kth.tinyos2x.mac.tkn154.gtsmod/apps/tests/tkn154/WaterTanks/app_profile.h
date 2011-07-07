
#ifndef __APP_PROFILE_H
#define __APP_PROFILE_H

enum {
  RADIO_CHANNEL = 0x14,
  PAN_ID = 0x1234,
  //BASE_NODEID=10,
  //INTERMEDIATE_NODEID=11,
  INTERMEDIATE_NODEID=0x00,
  COORDINATOR_ADDRESS = INTERMEDIATE_NODEID,
  BASE_NODEID = COORDINATOR_ADDRESS,
  BEACON_ORDER = 6,
  SUPERFRAME_ORDER = 5,
  TX_POWER = 0, // in dBm

  AM_MYMSG = 10, //Radio channel number
  //BUFFER_SIZE=10,
  BUFFER_SIZE=1, 
  FIRST_PLUG=1,
  SECOND_PLUG=2,

};

//structure of current meter message
typedef nx_struct MyMsg{// max 28 bytes
   nx_uint8_t other;
   nx_uint8_t srcId;
   nx_uint8_t trgtId;
   nx_uint16_t data[BUFFER_SIZE*2];
   nx_float integrator;
   nx_float next_time;
}MyMsg;

#endif
