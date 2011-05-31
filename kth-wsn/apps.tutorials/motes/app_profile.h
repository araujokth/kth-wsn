// $Id: TestSerialComm.h,v 1.4 2006-12-12 18:22:52 vlahan Exp $

#ifndef TESTSERIALCOMM_H
#define TESTSERIALCOMM_H

enum {
  AM_TESTSERIALCOMM = 6,
  TIMER_PERIOD_MILLI = 250,
};

typedef nx_struct TestSerialCommMsg {
	nx_uint16_t dummy;
	nx_uint16_t counter;
} TestSerialCommMsg;

#endif
