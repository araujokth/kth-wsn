/*
 * Copyright (c) 2011, KTH Royal Institute of Technology
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
 * @author Joao Faria <jfff@kth.se>
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @author David Andreu <daval@kth.se>
 *  
 * @version  $Revision: 1.2 Date: 2011/09/28 $ 
 */

#include "AM.h"
#include "Serial.h"
#include "app_sensors.h"
#include "app_wt_calibration.h"
#include "TKN154.h"
#include "UserButton.h"
#include "math.h"

module BaseStation154P {
	uses {
		interface Boot;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface MLME_GTS;

		interface IEEE154Frame as Frame;
		interface IEEE154BeaconFrame as BeaconFrame;
		interface IEEE154TxBeaconPayload;
		interface SuperframeStructure as BeaconSuperframe;

		interface Leds;
		interface Packet;

		interface SplitControl as SerialControl;
		interface AMSend as UartSend;
		interface Receive as UartReceive[am_id_t id];
		interface AMPacket as UartAMPacket;

		interface Get<ieee154_GTSdb_t*> as SetGtsCoordinatorDb;
		interface GtsUtility;

		interface Notify<bool> as IsEndSuperframe;
		interface GetNow<bool> as IsGtsOngoing;

		interface LocalTime<TSymbolIEEE802154>;
		interface Timer<TMilli> as InitController; // Timer to set the constant value to the voltage once we restart the experiment

		interface PinDebug;

		interface Notify<button_state_t> as UserButton;

	}provides {
		interface Notify<bool> as GtsSpecUpdated;
	}
}
implementation {

	// Messages to transmit
	message_t m_frameToMote;
	message_t m_frameToPC;
	message_t m_frameControl2Mote;

	// Struct with all the controller information
	InitConfigureController CONTROLLER;

	// Pointers to read/write the information of the packets
	ActuationValuesMsg* actuationMsg;
	WTInfoMsg * wtInfo;
	SuperframeConfigMsg * superframeConfigMsg;
	ControlMsg * controlMsg;
	Control2MoteMsg control2MoteMsg;

	// Size for each message
	uint8_t m_payloadLenToMote = sizeof(ActuationValuesMsg);
	uint8_t m_payloadLenFromMote = sizeof(WTSensorValuesMsg);
	uint8_t m_payloadLenControl2Mote = sizeof(Control2MoteMsg);
	uint8_t m_payloadLenToPC = sizeof(WTInfoMsg);
	uint8_t m_payloadLenFromPC = sizeof(SuperframeConfigMsg);

	ieee154_PANDescriptor_t m_PANDescriptor;

	// To manage the Serial comm
	message_t uartQueueBufs[UART_QUEUE_LEN];
	message_t * ONE_NOK uartQueue[UART_QUEUE_LEN];
	uint8_t uartIn, uartOut;
	uint8_t bo;
	bool uartBusy, uartFull;

	uint8_t m_state, prev_state;
	uint8_t m_phase;
	uint8_t m_counter;

	// How many slots do we want to set initially
	uint8_t numSlots = 18;

	enum {
		INITIALIZATION = 0x01, //wiat for the beacon and sysnc with the ADC
		ACTIVE = 0x2, // running
		CALIBRATION = 0x4,
		CONTROL = 0x6,
	};

	enum {
		PHASE_1 = 0x01,
		PHASE_2 = 0x02,
	};

	// To control the time between phases
	uint16_t numBeaconsTxToSteady;

	// To store the initial output for phase 2
	float outf0[NUMBER_WT];
	float x0[NUMBER_WT][2];

	// P E R F O R M A N C E   E V A L U A T I O N
	PerformanceParams sendPerfParams; // to compute the performance before sending
	uint32_t timestamp2Delay;

	//Restart Integral
	uint8_t completeReset[NUMBER_WT];
	bool resetIntegrals;
	WTInfoMsg prevWTInfo;
	nx_float k[NUMBER_WT][3];
	nx_float K[NUMBER_WT][2];
	nx_float V[NUMBER_WT][2];
	nx_float r_tk[NUMBER_WT][2];

	task void uartSendTask();
	void setAddressingFields(uint16_t address, message_t * frame);

	void periodicGtsChange();
	void matlabChange( );
	void matlabOrderChange( );
	void defaultConfiguration();

	void printfFloat(float k5);
	void startResetIntegrals();

	task void sendActuatorPacketGTS();
	task void sendActuatorPacketCAP();

	event void Boot.booted() {
		uint8_t i = 0;

		m_counter = 0;
		atomic {
			m_state = INITIALIZATION;
			prev_state = m_state;
		}
		m_phase = PHASE_1;
		numBeaconsTxToSteady=0;
		for (i = 0; i < UART_QUEUE_LEN; i++)
		uartQueue[i] = &uartQueueBufs[i];

		uartIn = uartOut = 0;
		uartBusy = FALSE;
		uartFull = TRUE;

		//call SerialControl.start();
		call MLME_RESET.request(TRUE);

		actuationMsg = (ActuationValuesMsg*)(call Packet.getPayload(&m_frameToMote,m_payloadLenToMote ));
		wtInfo = (WTInfoMsg*)(call Packet.getPayload(&m_frameToPC,m_payloadLenToPC ));

		sendPerfParams.pckTotal = 0;
		sendPerfParams.pckSuccess = 0;

		startResetIntegrals();

		// It could be really usefull to compute CONTROLLER.k[i][j] and V_REF automatically in Matlab
		// and print them here, them all the process will be automatic
		initController(&CONTROLLER);

		// Set the reference
		control2MoteMsg.ref = CONTROLLER.initReferenceCm;

		memcpy(k[0], CONTROLLER.k1[0], sizeof(nx_float)*3*NUMBER_WT );
		memcpy(K[0], CONTROLLER.K[0], sizeof(nx_float)*2*NUMBER_WT );
		memcpy(V[0], CONTROLLER.V[0], sizeof(nx_float)*2*NUMBER_WT );
		memcpy(r_tk[0], CONTROLLER.r_tk[0], sizeof(nx_float)*2*NUMBER_WT );

		call UserButton.enable();

	}

	void startResetIntegrals() {
		uint8_t i=0;
		resetIntegrals = TRUE;
		for (i = 0; i < NUMBER_WT; i++)
		completeReset[i] = 0;
	}
	event void InitController.fired() {

		m_phase = PHASE_2;

		// Change reference
		control2MoteMsg.ref = CONTROLLER.ref;

		// Reset the integrals
		startResetIntegrals();

		// Copy the controller
		memcpy(k[0], CONTROLLER.k2[0], sizeof(nx_float)*3*NUMBER_WT );

		// Change the reference value. At this time, the matlab program will start running. Before
		// it was waiting for the steady state (or reference change)
		call IEEE154TxBeaconPayload.setBeaconPayload(&control2MoteMsg, m_payloadLenControl2Mote);
	}

	event void UserButton.notify( button_state_t state ) {
		if ( state == BUTTON_PRESSED ) {
		} else if ( state == BUTTON_RELEASED ) {
			//			defaultConfiguration();
			uint8_t i=0, res;
			ieee154_GTSdb_t* GTSdb;
			GTSdb = call SetGtsCoordinatorDb.get();
			/***  INITIALIZE GTS SETTINGS  ***/
			GTSdb->numGtsSlots = 0;

			for (i = 0; i < numSlots; i++) {
				// IEEE154_aNumSuperframeSlots = 32
				call GtsUtility.addGtsEntry(GTSdb, 1, 32 - i + 1, 1, GTS_TX_ONLY_REQUEST);
			}

			numSlots++;
			signal GtsSpecUpdated.notify(TRUE);
		}
	}

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status != IEEE154_SUCCESS)
		return;

		call MLME_SET.phyTransmitPower(TX_POWER_COORDINATOR);
		call MLME_SET.macShortAddress(COORDINATOR_ADDRESS);
		call MLME_SET.macCoordShortAddress(COORDINATOR_ADDRESS);
		call MLME_SET.macAssociationPermit(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);
		call MLME_SET.macGTSPermit(TRUE);

		call MLME_START.request(
				PAN_ID, // PANId
				RADIO_CHANNEL, // LogicalChannel
				0, // ChannelPage,
				0, // StartTime,
				BEACON_ORDER, // BeaconOrder
				SUPERFRAME_ORDER, // SuperframeOrder
				TRUE, // PANCoordinator
				FALSE, // BatteryLifeExtension
				FALSE, // CoordRealignment
				0, // CoordRealignSecurity,
				0 // BeaconSecurity
		);

		// We send the reference to the motes
		call IEEE154TxBeaconPayload.setBeaconPayload(&control2MoteMsg.ref, m_payloadLenControl2Mote);
	}

	void setAddressingFields(uint16_t address, message_t *frame)
	{
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = address; // destination

		call Frame.setAddressingFields(
				frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);
	}

	event void MLME_GTS.confirm (
			uint8_t GtsCharacteristics,
			ieee154_status_t status) {}

	event void MLME_GTS.indication (
			uint16_t DeviceAddress,
			uint8_t GtsCharacteristics,
			ieee154_security_t *security) {}

	event void MLME_START.confirm(ieee154_status_t status) {}

	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	) {
		// reset the packet counters
		if (sendPerfParams.pckTotal >= SLIDING_WINDOW) {
			sendPerfParams.pckTotal = 0;
			sendPerfParams.pckSuccess = 0;
			return;
		}
		if (status == IEEE154_SUCCESS) {
			sendPerfParams.pckSuccess ++;
		}

		sendPerfParams.delay = (uint16_t) timestamp - timestamp2Delay;
	}

	/***************************************************************************************
	 * I E E E  8 0 2 . 1 5 . 4    P A C K E T S 
	 ***************************************************************************************/
	event message_t* MCPS_DATA.indication (message_t* frame)
	{

		ieee154_address_t deviceAddr;
		WTSensorValuesMsg *m_device;
		nx_float outf;
		uint8_t wtId;
		nx_float tmp_integral;

		if (call Frame.getPayloadLength(frame) == m_payloadLenFromMote &&
				call Frame.getFrameType(frame) == FRAMETYPE_DATA) {

			call Frame.getSrcAddr(frame, &deviceAddr);
			atomic {

				m_device = (WTSensorValuesMsg*)(call Packet.getPayload(frame,m_payloadLenFromMote));

				// Is a WaterTank or a extar sensor?
				if (deviceAddr.shortAddress < (NUMBER_WT*2+1)) {

					//					call Leds.led1Toggle();

					wtId = (deviceAddr.shortAddress + 1 )/2;
					setAddressingFields(wtId*2, &m_frameToMote);

					// We have received a controlMsg during the Inactive period to restart the integral
					if (resetIntegrals) {
						completeReset[wtId-1] = 1;
						prevWTInfo.integral_initial[wtId-1] = m_device->integrator;
						prevWTInfo.u_initial[wtId-1] = wtInfo->u[wtId-1];
					}
					// Remove the initial value
					// With this tmp_integral we can analise when the tanks reach the steady state instead of waiting for the timer
					// When the current tmp_integral is equal to the last tmp_integral we are done. (Better with a certain window)
					tmp_integral = m_device->integrator - prevWTInfo.integral_initial[wtId-1];

					switch(m_state) {
						case ACTIVE:

						if (m_phase == PHASE_1) {
//							call Leds.led1On();
							outf = prevWTInfo.u_initial[wtId-1]
							+ (273.0 * (((((nx_float) m_device->tankLevel[0])/WT_CALIBRATION) - control2MoteMsg.ref) * k[wtId-1][0]
											+ ((((nx_float) m_device->tankLevel[1])/WT_CALIBRATION) - control2MoteMsg.ref) * k[wtId-1][1]
											+ ( tmp_integral ) * k[wtId-1][2]));

							outf0[wtId-1]=outf;
							x0[wtId-1][0]=(m_device->tankLevel[0])/WT_CALIBRATION;
							x0[wtId-1][1]=(m_device->tankLevel[1])/WT_CALIBRATION;
							
						} else if (m_phase == PHASE_2) {
							
							outf = outf0[wtId-1] + (273.0 * (((((nx_float) m_device->tankLevel[0])/WT_CALIBRATION)-x0[wtId-1][0]) * K[wtId-1][0]
											+ ((((nx_float) m_device->tankLevel[1])/WT_CALIBRATION)-x0[wtId-1][1]) * K[wtId-1][1]
											+ (V[wtId-1][0])*r_tk[wtId-1][0]+ (V[wtId-1][1])*r_tk[wtId-1][1]));
						}

						break;
						case CALIBRATION:

						// The control voltage is given by the Matlab
						outf = (nx_float) (273 * controlMsg->u[wtId-1]);
						break;

						case INITIALIZATION:

						// Drive the tanks to the steady state 
						outf = (nx_float) (273.0 * CONTROLLER.initReferenceVolts[wtId-1]);

						prevWTInfo.u_initial[wtId-1] = (uint16_t) outf;

						//prevWTInfo.integral_initial[wtId-1] = m_device->integrator;
						prevWTInfo.y_initial[wtId-1][0] = m_device->tankLevel[0];
						prevWTInfo.y_initial[wtId-1][1] = m_device->tankLevel[1];
						break;
					}

					if(outf < 0) outf = 0;
					else if(outf > 4095) outf = 4095;

					wtInfo->y[wtId-1][0] = m_device->tankLevel[0];
					wtInfo->y[wtId-1][1] = m_device->tankLevel[1];
					wtInfo->u[wtId-1] = (uint16_t) outf;
					wtInfo->integral[wtId-1] = m_device->integrator - prevWTInfo.integral_initial[wtId-1];

					actuationMsg->u = (uint16_t) outf;
					actuationMsg->wtId = wtId;

					setAddressingFields(wtId*2, &m_frameToMote);

				
					if (m_device->isGTSpacket)
					{
						call Leds.led0Toggle();
						post sendActuatorPacketGTS();
					}else{
						call Leds.led1Toggle();
						post sendActuatorPacketCAP();
					}
					
				} else {
					// They are just extra sensors
					// We use to same WTSensorValuesMsg message to simplify
					wtInfo->extraSensors[deviceAddr.shortAddress - (2*NUMBER_WT + 1) ] = m_device->tankLevel[0];
				}

			}
		}
		return frame;
	}

	task void sendActuatorPacketGTS() {
		// We send the communication performance information in the payload of the packet
		// If we have the sniffer connect, it will store the information

		// set the timestamp and copy the new performance data
		memcpy(&(actuationMsg->performValues), &sendPerfParams, sizeof(PerformanceParams) );
		//set the new timestamp value
		timestamp2Delay = call LocalTime.get();

		sendPerfParams.pckTotal ++;

		if (call MCPS_DATA.request ( &m_frameToMote, // frame,
						m_payloadLenToMote, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK | TX_OPTIONS_GTS // TxOptions,
				) != IEEE154_SUCCESS) {
//			call Leds.led0On(); //fail!
		}
	}
	task void sendActuatorPacketCAP() {
		// We send the communication performance information in the payload of the packet
		// If we have the sniffer connect, it will store the information

		// set the timestamp and copy the new performance data
		memcpy(&(actuationMsg->performValues), &sendPerfParams, sizeof(PerformanceParams) );
		//set the new timestamp value
		timestamp2Delay = call LocalTime.get();

		sendPerfParams.pckTotal ++;

		if (call MCPS_DATA.request ( &m_frameToMote, // frame,
						m_payloadLenToMote, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
//			call Leds.led0On(); //fail!
		}
	}
	

	/************************************************************************************
	 *  SERIAL FUNCTIONS
	 ************************************************************************************/
	task void uartSendTask() {
		// Send to the serial port
		message_t* msg;

		atomic {
			if (uartIn == uartOut && !uartFull) {
				uartBusy = FALSE;
				return;
			}
		}

		msg = uartQueue[uartOut];

		if (call UartSend.send(AM_BROADCAST_ADDR, uartQueue[uartOut], m_payloadLenToPC ) == SUCCESS) {
			//call Leds.led1Toggle();
		} else {
			//call Leds.led0Toggle();
			post uartSendTask();
		}
	}

	event void SerialControl.startDone(error_t error) {
		if (error == SUCCESS) {
			uartFull = FALSE;
		}

	}

	event void SerialControl.stopDone(error_t error) {}

	event void UartSend.sendDone(message_t* msg, error_t error) {
		if (error != SUCCESS) {
			//call Leds.led0Toggle();
		} else
		atomic
		if (msg == uartQueue[uartOut])
		{
			if (++uartOut >= UART_QUEUE_LEN)
			uartOut = 0;
			if (uartFull)
			uartFull = FALSE;
		}
		post uartSendTask();
	}

	/************************************************************************************
	 *  CFP EVENTS/NOTIFIES FUNCTIONS
	 ************************************************************************************/
	event void IsEndSuperframe.notify( bool val ) {
		uint8_t i = 0;

		// call PinDebug.ADC2set();

		// Before we check if all the integral are reset
		// Lets assume that we we check MANUALLY that all the tanks have been reset
		// otherwise we need to run all the tanks every time we want to test something
		if (resetIntegrals) {
			for (i=0; i < NUMBER_WT; i++ ) if (completeReset[i] == 0) break;

			if ( i == NUMBER_WT) {
				//controlMsg->cmd = CONSTANT_CONTROLLER; // It doesn't matter, this value is only important
				resetIntegrals = FALSE;
				// if we are restarting the integral or when we receive the ControlMsg
				m_state = prev_state; // Restore the previous state
			}
		}

		//send packet with the time to the computer
		wtInfo->time = call BeaconSuperframe.sfStartTime();

		for (i=0; i < NUMBER_WT; i++ ) {

			wtInfo->y_initial[i][0] = prevWTInfo.y_initial[i][0];
			wtInfo->y_initial[i][1] = prevWTInfo.y_initial[i][1];
			wtInfo->integral_initial[i] = prevWTInfo.integral_initial[i];
			wtInfo->u_initial[i] = prevWTInfo.u_initial[i];
			wtInfo->ref = control2MoteMsg.ref;

		}

		atomic {
			if (!uartFull)
			{
				memcpy(uartQueue[uartIn], &m_frameToPC, sizeof(message_t));
				uartIn = (uartIn + 1) % UART_QUEUE_LEN;

				if (!uartBusy)
				{
					//call Leds.led1Toggle();
					post uartSendTask();
					uartBusy = TRUE;
				}
			}
			else {
				//call Leds.led0Toggle();
			}
		}

		return;
	}
	/************************************************************************************
	 *  BEACON FUNCTIONS
	 ************************************************************************************/
	event void IEEE154TxBeaconPayload.aboutToTransmit() {}

	event void IEEE154TxBeaconPayload.setBeaconPayloadDone(void *beaconPayload, uint8_t length) {

	}

	event void IEEE154TxBeaconPayload.modifyBeaconPayloadDone(uint8_t offset, void *buffer, uint8_t bufferLength) {

	}

	event void IEEE154TxBeaconPayload.beaconTransmitted()
	{
		ieee154_macBSN_t beaconSequenceNumber = call MLME_GET.macBSN();

		if (m_state==ACTIVE) {
			numBeaconsTxToSteady=numBeaconsTxToSteady+1;
			if (numBeaconsTxToSteady == 100) {
				m_phase = PHASE_2;

				// Change reference
				control2MoteMsg.ref = CONTROLLER.ref;

				// Reset the integrals
				startResetIntegrals();

				// Copy the controller
				memcpy(k[0], CONTROLLER.k2[0], sizeof(nx_float)*3*NUMBER_WT );

				// Change the reference value. At this time, the matlab program will start running. Before
				// it was waiting for the steady state (or reference change)
				call IEEE154TxBeaconPayload.setBeaconPayload(&control2MoteMsg, m_payloadLenControl2Mote);

			} else if (numBeaconsTxToSteady>=450) {
				// End the experiment
//				call Leds.led1On();
				control2MoteMsg.ref = 20;
				call IEEE154TxBeaconPayload.setBeaconPayload(&control2MoteMsg, m_payloadLenControl2Mote);
			}
		}
		if (beaconSequenceNumber & 1) {
			call Leds.led2On();
		}
		else {
			call Leds.led2Off();

		}

	}
	/***************************************************************************************
	 * C H A N G E   G T S  D E S C R I P T O R
	 ***************************************************************************************/

	event message_t *UartReceive.receive[am_id_t id](message_t *msg,
			void *payload,
			uint8_t len) {
		// Fired when we receive a message from the PC

		if (call UartAMPacket.type(msg) == AM_CONTROLMSG) {
			uint8_t i;
			controlMsg = call Packet.getPayload(msg, m_payloadLenToMote);
			defaultConfiguration();
			memcpy(k[0], CONTROLLER.k1[0], sizeof(nx_float)*3*NUMBER_WT );

			if (controlMsg->cmd == RESET_INTEGRAL) {
				prev_state = m_state;
				m_state = CONTROL;
				startResetIntegrals();
//				control2MoteMsg.ref = CONTROLLER.ref;

				call IEEE154TxBeaconPayload.setBeaconPayload(&control2MoteMsg, m_payloadLenControl2Mote);

			} else if (controlMsg->cmd == MODELING_CONTROLLER || controlMsg->cmd == CONSTANT_CONTROLLER) {
				// In the calibration state, is the Matlab who computes the control voltage.
				// Everytime one experiment finishes, the file with the model information is saved in the 
				// folder mat/model
				prev_state = m_state;
				m_state = CALIBRATION;

			} else if (controlMsg->cmd == REFERENCE_CONTROLLER) {
				resetIntegrals = TRUE; // Reset the integrals

				m_state = ACTIVE;
				prev_state = m_state;

//				control2MoteMsg.ref = CONTROLLER.ref;
				call IEEE154TxBeaconPayload.setBeaconPayload(&control2MoteMsg, m_payloadLenControl2Mote);

			} else {

				//m_state = INITIALIZATION;
				m_state = ACTIVE;
				prev_state = m_state;
				startResetIntegrals();

				// Switch to initial mode
				control2MoteMsg.ref = CONTROLLER.initReferenceCm;
				call IEEE154TxBeaconPayload.setBeaconPayload(&control2MoteMsg, m_payloadLenControl2Mote);

			}

		} else if (call UartAMPacket.type(msg) == AM_SUPERFRAMECONFIGMSG && m_state == ACTIVE) {

			superframeConfigMsg = call Packet.getPayload(msg, m_payloadLenFromPC);

			// If we receive a packet with BI=0xFF it indicate that we start the experiment
			if (superframeConfigMsg->BI == 0xff) {
				// We initiliaze m_counter = 0. So if we only enter to this function once,
				// only initilizes the GTS Descriptor
				defaultConfiguration();
			} else {
				matlabChange();
				//defaultConfiguration();
				//periodicGtsChange(); // Periodic change to debug the program
			}

			call PinDebug.ADC2clr();
		}
		return msg;

	}

	void defaultConfiguration() {
		// This function will generate a GTS database where all the Water tanks are transmitting information
		// if we have enought space
		uint8_t i = 0;
		ieee154_GTSdb_t* GTSdb;
		GTSdb = call SetGtsCoordinatorDb.get();
		/***  INITIALIZE GTS SETTINGS  ***/
		GTSdb->numGtsSlots = 0;

		for (i = 0; i < NUMBER_WT; i++) {
			call GtsUtility.addGtsEntry(GTSdb, 2*i+2, IEEE154_aNumSuperframeSlots - (2*i+1), 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 2*i+1, IEEE154_aNumSuperframeSlots - (2*i+2), 1, GTS_TX_ONLY_REQUEST);
		}
		for (i=2*i; i < GTS_SLOTS_ALLOCATED; i++)
		call GtsUtility.addGtsEntry(GTSdb, i+1, IEEE154_aNumSuperframeSlots - (i+1), 1, GTS_TX_ONLY_REQUEST);

		signal GtsSpecUpdated.notify(TRUE);

	}

	void periodicGtsChange() {
		// It will generate a null GTS database or a default GTS database depending on the counter
		// It is useful to check if the GTS database is working properly
		ieee154_GTSdb_t* GTSdb;
		uint8_t i;

		/***  INITIALIZE GTS SETTINGS  ***/
		GTSdb = call SetGtsCoordinatorDb.get();

		GTSdb->numGtsSlots = 0;
		if (m_counter++ == 1) {
			m_counter = 0;
			GTSdb = call SetGtsCoordinatorDb.get();
			GTSdb->numGtsSlots = 0;

			for (i=0; i < GTS_SLOTS_ALLOCATED; i++)
			call GtsUtility.addGtsEntry(GTSdb, i+20, IEEE154_aNumSuperframeSlots - (i+1), 1, GTS_TX_ONLY_REQUEST);

			signal GtsSpecUpdated.notify(TRUE);

		} else {
			defaultConfiguration();
		}

	}

	void matlabChange() {
		// This function creates the GTS database based on the slots vector sent by the PC
		// The timeslots vector will contain the ID for the mote that needs to transmit in the slot.
		// For example: timeslot = [0 0 0 1 0 0 0 3]
		// 		Mote with ID = 1 transmits in the slot IEEE154_aNumSuperframeSlots - 4
		// 		Mote with ID = 3 transmits in the slot IEEE154_aNumSuperframeSlots - 8
		ieee154_GTSdb_t* GTSdb;
		uint8_t i = 0;
		uint8_t *pSlots;

		pSlots = superframeConfigMsg->timeslots;
		GTSdb = call SetGtsCoordinatorDb.get();

		GTSdb->numGtsSlots = 0;

		// Go through all the timeslots field. Where we store the id that needs to transmit in the currect slot
		for (i = 0; i < GTS_SLOTS_ALLOCATED; i++) {
			if (pSlots[i] == 0)
			break;
			// Is it a water tank?
			else if (pSlots[i] <= (2 * NUMBER_WT)) {
				call GtsUtility.addGtsEntry(GTSdb, pSlots[i+1], IEEE154_aNumSuperframeSlots - (i+1), 1, GTS_RX_ONLY_REQUEST);
				call GtsUtility.addGtsEntry(GTSdb, pSlots[i], IEEE154_aNumSuperframeSlots - (i+2), 1, GTS_TX_ONLY_REQUEST);
				i++; //jump to the next water tank

				// Is it a sensor?
			} else if (pSlots[i] > (2 * NUMBER_WT)) {
				call GtsUtility.addGtsEntry(GTSdb, pSlots[i], IEEE154_aNumSuperframeSlots - (i+1), 1, GTS_TX_ONLY_REQUEST);
			}

		}

		signal GtsSpecUpdated.notify(TRUE);
	}

	/***************************************************************************************
	 * DEFAULTS spec updated commands
	 ***************************************************************************************/
	command error_t GtsSpecUpdated.enable() {return FAIL;}
	command error_t GtsSpecUpdated.disable() {return FAIL;}
	default event void GtsSpecUpdated.notify( bool val ) {return;}

	/***************************************************************************************
	 * O T H E R   T O O L S
	 ***************************************************************************************/
	void printfFloat(float toBePrinted) {
		uint32_t fi, f0, f1, f2;
		char c;
		float f = toBePrinted;

		if (f < 0) {
			c = '-';
			f = -f;
		} else {
			c = ' ';
		}

		// integer portion.
		fi = (uint32_t) f;

		// decimal portion...get index for up to 3 decimal places.
		f = f - ((float) fi);
		f0 = f * 10;
		f0 %= 10;
		f1 = f * 100;
		f1 %= 10;
		f2 = f * 1000;
		f2 %= 10;
		printf("%c%ld.%d%d%d", c, fi, (uint8_t) f0, (uint8_t) f1, (uint8_t) f2);
	}

}
