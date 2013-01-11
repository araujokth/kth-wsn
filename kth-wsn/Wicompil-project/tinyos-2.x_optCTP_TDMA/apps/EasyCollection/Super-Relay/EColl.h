#ifndef ECOLL_H
#define ECOLL_H

#include <AM.h>

/*********************************************************************
		* Define reference values for LAB 3
*********************************************************************/

#define WT_CALIBRATION 87.3

enum{
PHASE1_DURATION=90000,
MAX_LEVEL=15,
SAMPLING_PERIOD=1000,
};

typedef nx_struct ECollMessage {
  nx_uint16_t  seqno;
  nx_uint16_t  data[2] ; // Sensors values from upper and Lower tank
} ECollMessage;

typedef nx_struct ControllertoSF {
	nx_uint16_t TankLevel[2];  
	nx_uint16_t  voltage;     
	nx_uint16_t  counter;

} ControllertoSF;

enum {
    MSGLEN = 6,
    MAXMSG = 5,
    AM_MYMSG = 10, 
    COORDINATOR_ADDRESS = 0x0001,
    MAXRETS = 5,		
};

bool isRelay = FALSE;
bool isController=TRUE;
#endif