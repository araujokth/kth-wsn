
#ifndef TESTSERIALCOMM_H
#define TESTSERIALCOMM_H

enum {
  AM_TESTSERIALCOMMMSG = 6,
  TIMER_PERIOD_MILLI = 250,
};

typedef nx_struct TestSerialCommMsg {
	nx_uint16_t dummy;
	nx_uint16_t counter;
} TestSerialCommMsg;

#endif
