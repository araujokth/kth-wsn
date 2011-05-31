
#ifndef TESTSERIALCOMM_H
#define TESTSERIALCOMM_H

enum {
  AM_TESTSERIALCOMM = 6,
  TIMER_PERIOD_MILLI = 250,

  PAN_ID = 0x1234,

  ACTUATOR_ADRESS = 0x02,
  SENSOR_ADRESS = 0x01,

  TX_POWER = 0,
  SUPERFRAME_ORDER = 15,
  BEACON_ORDER = 15,
  RADIO_CHANNEL = 0x10,
};

typedef nx_struct TestSerialCommMsg {
	nx_uint16_t dummy;
	nx_uint16_t counter;
} TestSerialCommMsg;

#endif
