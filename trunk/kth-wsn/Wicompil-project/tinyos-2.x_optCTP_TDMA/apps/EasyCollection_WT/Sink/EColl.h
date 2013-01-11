#ifndef ECOLL_H
#define ECOLL_H

#include <AM.h>

/*********************************************************************
		* Define reference values for LAB 3
*********************************************************************/

#define WT_CALIBRATION 87.3
#define OP_POINT 5		// Operating point(1st phase of the experiment)
#define REFERENCE 8	

enum{
PHASE0_DURATION=30000,
PHASE1_DURATION=60000,
PHASE2_DURATION=60000,
MAX_LEVEL=25,
AM_TESTSERIALCOMMMSG = 6,
SAMPLING_PERIOD=500,
SENSE_INTERVAL=1000,
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
    MSGLEN = sizeof(ECollMessage),
    MAXMSG = 10,
    AM_MYMSG = 10, 
    COORDINATOR_ADDRESS = 0x0001,
    MAXRETS = 5,		
};
bool PRINT=FALSE;
bool isRelay = FALSE;
bool isController=FALSE;
#endif
