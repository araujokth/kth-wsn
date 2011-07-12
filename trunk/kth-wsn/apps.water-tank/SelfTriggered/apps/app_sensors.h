// $Id: BlinkToRadio.h,v 1.4 2006/12/12 18:22:52 vlahan Exp $


#ifndef APP_SENSORS_H
#define APP_SENSORS_H

#define SIGMA  0.05

enum {
	AM_SUPERFRAMECONFIGMSG = 13,
	AM_WTINFOMSG = 14,
	AM_WTSENSORVALUESMSG = 15,
	AM_ACTUATIONVALUESMSG = 16,
	AM_CONTROLMSG = 17,

	NUMBER_WT_ET = 1,
	NUMBER_WT_ST = 0,
	NUMBER_WT = NUMBER_WT_ET + NUMBER_WT_ST,

	GTS_SLOTS_ALLOCATED = 22,

	EXTRA_SENSORS = 1,
	NUMBER_TANKS = 2,

	WT_INITIAL_REFERENCE_TIME = 120000, // 80 s
	UART_QUEUE_LEN = 12,

	RADIO_CHANNEL = 25,
	PAN_ID = 4660,

	BEACON_ORDER = 6, // We need time to configure the new superframe
	SUPERFRAME_ORDER = 6, // We cannot modify otherwise we don't have time to compute the control output

	COORDINATOR_ADDRESS = 0x00,

	TX_POWER = 0, // in dBm
	TX_POWER_COORDINATOR = 0, // in dBm

	TIMER_PREC = 32768U,

	SLIDING_WINDOW = 500,

	// Controller information or Command information that we send
	// in the ControlMsg
	PERIODIC_CONTROLLER = 1,
	SELFTRIGGERED_CONTROLLER = 2,
	MODELING_CONTROLLER = 3,
	CONSTANT_CONTROLLER = 4,
	REFERENCE_CONTROLLER = 5,
	RESET_INTEGRAL = 6,
	EVENT_TRIGGERED_CONTROLLER = 7,

};

typedef nx_struct PerformanceParams {

	nx_uint16_t pckTotal;
	nx_uint16_t pckSuccess;
	nx_uint16_t delay;

}PerformanceParams;

typedef nx_struct ActuationValuesMsg {
	nx_int16_t u; //actuation
	nx_uint8_t wtId;

	PerformanceParams performValues;

}ActuationValuesMsg;

typedef struct WTSensorValuesMsg {
	nx_uint16_t tankLevel[NUMBER_TANKS]; //value for the tk1 and tk2
	nx_float integrator;

	PerformanceParams performValues;
} WTSensorValuesMsg;

//structure of the message used by the BS to communicate with the PC in the WaterTanks system
typedef nx_struct WTInfoMsg {
	nx_int16_t y[NUMBER_WT][NUMBER_TANKS]; // Tank systems y[i] -> Tanks levels
	nx_int16_t y_initial[NUMBER_WT][NUMBER_TANKS]; // Initial Tank systems y[i] -> Tanks levels

	nx_int16_t extraSensors[EXTRA_SENSORS]; // Extra sensor vector

	nx_int16_t u[NUMBER_WT]; // Voltage applied to pump on tank system i
	nx_int16_t u_initial[NUMBER_WT]; // Initial voltage applied to pump on tank system i

	nx_uint32_t time; //Time when CFP started

	nx_float integral[NUMBER_WT]; // Integral of y[i][2] tracking error. lower tank
	nx_float integral_initial[NUMBER_WT]; // Initial integral of y[i][2] tracking error. lower tank

	nx_float ref; //Time when CFP started
}WTInfoMsg;

//structure of the message used by the PC to communicate with the BS in the WaterTanks system
typedef nx_struct SuperframeConfigMsg {
	nx_uint8_t BI; //Beacon interval
	nx_uint8_t timeslots[GTS_SLOTS_ALLOCATED]; // If there is an error trying to compile the java classes,
	nx_float ref; //send the reference from the computer to the mote
	// substitute the constants manually
}SuperframeConfigMsg;

// we only create this message to identify easily the state
typedef nx_struct ControlMsg {
	nx_uint8_t cmd; // we coudl control the change of controller from the PC

	nx_float u[NUMBER_WT]; //actuation
}ControlMsg;

typedef nx_struct Control2MoteMsg {
	nx_float ref;
}Control2MoteMsg;

typedef struct InitConfigureController {
	nx_float k1[NUMBER_WT][3];
	nx_float k2[NUMBER_WT][3];

	nx_float initReferenceVolts[NUMBER_WT];
	nx_float finalUpperTankLevel[NUMBER_WT];
	nx_float finalIntegral[NUMBER_WT];

	nx_float initReferenceCm;
	nx_float ref;
} InitConfigureController;

void initController(InitConfigureController* ctlr) {
	// Initialize all the controllers


	//		ctlr->k[0][0] = -0.9523;
	//		ctlr->k[0][1] = -0.0395;
	//		ctlr->k[0][2] = -0.0543;

	//		ctlr->k[1][0] = -1.0499;
	//		ctlr->k[1][1] = -0.0761;
	//		ctlr->k[1][2] = -0.0612;
	ctlr->k1[0][0] = -1.0499;
	ctlr->k1[0][1] = -0.0761;
	ctlr->k1[0][2] = -0.0612;

	ctlr->k2[0][0] = -0.2150;
	ctlr->k2[0][1] = -0.1639;
	ctlr->k2[0][2] = -0.0077;

	// Set the initial voltage
	// to have 5cm in the lower tank
	ctlr->initReferenceVolts[0] = 4.363;
	//ctlr->initReferenceVolts[0] = 5.34;
	//		ctlr->initReferenceVolts[1] = 4.363;

	ctlr->finalUpperTankLevel[0] = 10.93;
	//	ctlr->finalUpperTankLevel[0] = 10;
	//ctlr->finalUpperTankLevel[1] = 10.93;

	ctlr->finalIntegral[0] = -120.3;
	//ctlr->finalIntegral[0] = -120.3;
	//ctlr->finalIntegral[1] = -87.4;s

	ctlr->initReferenceCm = 5;
	ctlr->ref = 10;
}

#endif
