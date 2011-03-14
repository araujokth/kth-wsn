/*
 * Copyright (c) 2010, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice, this list
 * 	  of conditions and the following disclaimer.
 *
 * 	- Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *	  materials provided with the distribution.
 *
 * 	- Neither the name of the KTH Royal Institute of Technology nor the names of its
 *    contributors may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @author Joao Faria <jfff@kth.se>
 *
 * @version  $Revision: 1.0 Date: 2010/11/03 $
 * @modified 2011/02/01
 */
#ifndef APP_SENSORS_H
#define APP_SENSORS_H

enum {

	DEFAULT_RATE = 200,
	DEFAULT_LEVEL = 10,

	// Matrix limits && slots number
	NUMBER_WT = 8, //number of slots
	UPDATE_INTERVAL = 2,

	// Msg identifiers
	AM_SENSORVALUES = 10,
	AM_ACTUATIONMATRIXMSG = 11,
	AM_SENSORMATRIXMSG = 12,

	SLIDING_WINDOW = 1000,

	UART_QUEUE_LEN = 12,

	//Not broadcast, just 2 try
	BROADCAST_DESTINATION = 0x01,

	RADIO_CHANNEL = 0x12,
	PAN_ID = 0x1234,

	BEACON_ORDER = 6,
	SUPERFRAME_ORDER = 6,

	COORDINATOR_ADDRESS = 0x0,
	TX_POWER = 0, // in dBm
	TX_POWER_COORDINATOR = 0, // in dBm

	TIMER_PREC = 32768U,

	BUFFER_SIZE = 1,

};

typedef nx_struct ActuationMatrixMsg {
	nx_int16_t u[NUMBER_WT]; //actuation
}ActuationMatrixMsg;

typedef nx_struct SensorMatrixMsg {
	nx_uint16_t tankLevel1[NUMBER_WT];
	nx_uint16_t tankLevel2[NUMBER_WT];
}SensorMatrixMsg;

typedef nx_struct EncMsg2SensorsAct {
	nx_int16_t u; //actuation
	nx_uint8_t wtId;
}EncMsg2SensorsAct;

typedef struct PerformanceParams {
	nx_uint16_t pckTotal;
	nx_uint16_t pckSuccess;
	nx_uint16_t delay;
} PerformanceParams;

typedef struct SensorValues {
	nx_uint16_t tankLevel[2]; //value for the tk1 and tk2
} SensorValues;

typedef struct SensingMsg {
	SensorValues data;
	PerformanceParams performValues;
} SensingMsg;

#endif
