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

#include "AM.h"
#include "Serial.h"
#include "TKN154.h"
#include "printf.h"

#include "app_sensors_nbe.h"
#include "app_wt_calibration.h"

#include "UserButton.h"

#include "math.h"

module ControllerC {
	uses {
		interface Boot;
		interface Leds;

		interface Packet;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;
		interface Alarm<TSymbolIEEE802154,uint32_t> as SlotAlarm;
		interface Timer<TMilli> as TimerChange;

#ifndef TKN154_BEACON_DISABLED
		interface IEEE154TxBeaconPayload;
#endif
		// communication with the PC
		interface SplitControl as SerialControl;
		interface AMSend as UartSend;
		interface Receive as UartReceive;
		interface AMPacket as UartAMPacket;

		interface Notify<button_state_t> as UserButton;
	}
}
implementation {

	message_t m_frame;
	EncMsg2SensorsAct *m_actuator;

	bool m_wasScanSuccessful;

	uint8_t m_payloadLen = sizeof(SensingMsg);
	uint8_t m_payloadLenSend = sizeof(EncMsg2SensorsAct);
	uint16_t m_payloadLenPC = sizeof(ActuationMatrixMsg);
	void setAddressingFields(uint16_t address);
	ieee154_PANDescriptor_t m_PANDescriptor;

	// SERIAL COMMUNICATION
	message_t uartQueueBufs[UART_QUEUE_LEN];
	message_t * ONE_NOK uartQueue[UART_QUEUE_LEN];
	uint8_t uartIn, uartOut;
	bool uartBusy, uartFull;
	task void uartSendTask();

	// TDMA Management
	uint32_t m_sfStartRef;
	uint32_t m_capDuration;
	uint32_t m_slotDuration;

	nx_float x_int[NUMBER_WT];
	nx_float x_ref_vector[7];
	nx_float* x_ref;
	uint8_t level;
	
	uint8_t m_state;
	enum {
		INITIALIZATION = 0x01, //wiat for the beacon and sysnc with the ADC
		RUNNING = 0x2, // runing
	};

	void setAddressingFields(uint16_t address);
	void printfFloat(float toBePrinted);

	/**----------------------------------------------*/
	event void Boot.booted() {
		uint8_t i = 0;

#if defined(WT_CALIBRATION_PROCESS) || defined(WT_USERBUTTON)
		m_state = INITIALIZATION;
		call UserButton.enable();
#else
		m_state = RUNNING;
#endif

#ifdef WT_CALIBRATION_PROCESS
		printf("\nC A L I B R A T I O N   M O D E:\n\n");
		printf("1. Disconnect the motor (unplugged From Load cable) \n");
		printf("2. Set both tanks to level ~0cm by adjusting the OFFSET resistor\n");
		printf("3. Fill the tank 1 (upper tank) to 25cm\n");
		printf("4. Set tank 1 to level ~25cm by adjusting the GAIN resistor\n");
		printf("5. Transfer the water to tank 2\n");
		printf("6. Set tank 1 to level ~25cm by adjusting the GAIN resistor\n");
		printf("---------------------------------------------------\n");
		printf("PRESS THE USER BUTTON TO CONTINUE\n");

		printfflush();
#endif

		m_actuator = (EncMsg2SensorsAct*)(call Packet.getPayload(&m_frame,m_payloadLenSend));
		m_actuator->u = 0;
		m_actuator->wtId = 0;

		call Leds.led0On();
		call Leds.led1On();
		call Leds.led2On();

		for (i = 0; i < UART_QUEUE_LEN; i++)
		uartQueue[i] = &uartQueueBufs[i];

		uartIn = uartOut = 0;
		uartBusy = FALSE;
		uartFull = TRUE;

		// set the beaon variables
		m_capDuration = (((uint32_t) 1 << SUPERFRAME_ORDER) * (uint32_t) IEEE154_aBaseSuperframeDuration);
		m_slotDuration = m_capDuration / NUMBER_WT;

		x_ref_vector[0] = 5.0;
		x_ref_vector[1] = 10.0;
		x_ref_vector[2] = 15.0;
		x_ref_vector[3] = 20.0;
		x_ref_vector[4] = 15.0;
		x_ref_vector[5] = 10.0;
		x_ref_vector[6] = 2.0;
		x_ref = x_ref_vector;
		level = 0;
		
		call SerialControl.start();
		call MLME_RESET.request(TRUE);
	}

	event void UserButton.notify( button_state_t state ) {
		if ( state == BUTTON_PRESSED ) {
			printf("---------------------------------------------------\n");
		} else if ( state == BUTTON_RELEASED ) {
			atomic m_state = RUNNING;
			call TimerChange.startPeriodic((uint32_t) DEFAULT_RATE*SLIDING_WINDOW/10);

		}

	}
	event void TimerChange.fired() {
		call Leds.led2Toggle();
		level = (level+1) % 7;
		
		x_ref = x_ref_vector + level;
	}
	/*********************************************************************
	 * TDMA functions
	 *********************************************************************/
	async event void SlotAlarm.fired() {}

	/*********************************************************************
	 * 802.15.4 functions
	 *********************************************************************/
	void startApp()
	{
		call Leds.led0Off();
		call Leds.led1Off();
		call Leds.led2Off();

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(COORDINATOR_ADDRESS);

		call MLME_SET.macAssociationPermit(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);

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
		setAddressingFields(0x10); //Random adress we will send to the selected actuator
	}

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status != IEEE154_SUCCESS) return;
		startApp();
	}

	void setAddressingFields(uint16_t address)
	{
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = address; // destination

		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);

	}

	event void MLME_START.confirm(ieee154_status_t status) {}

	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame)
	{
		ieee154_address_t deviceAddr;
		SensingMsg *m_device;
		nx_float K[3], outf;
		
		K[0] = -0.6002; //
		K[1] = -0.0021; //
		K[2] = -0.1; //


		// We assume that the packets with length equal to EncMsg
		// are a EncMsg
		if (call Frame.getPayloadLength(frame) == m_payloadLen &&
				call Frame.getFrameType(frame) == FRAMETYPE_DATA && m_state == RUNNING) {
			call Leds.led1Toggle();

			call Frame.getSrcAddr(frame, &deviceAddr);
			atomic {
				m_device = (SensingMsg*)(call Packet.getPayload(frame,m_payloadLen));
				m_actuator->wtId = (deviceAddr.shortAddress + 1 )/2;

				x_int[m_actuator->wtId-1] += ((nx_float) DEFAULT_RATE/1000 *
						((((nx_float) (m_device->data).tankLevel[0])/WT_CALIBRATION) - *x_ref));

				// 1v in the pump is approx. 273 units in the DAC
				outf = (273.0 * (((((nx_float) (m_device->data).tankLevel[0] )/WT_CALIBRATION)) * K[0] +
								((((nx_float) (m_device->data).tankLevel[1] )/WT_CALIBRATION)) * K[1] +
								x_int[m_actuator->wtId-1] * K[2]));

				if(outf < 0)outf = 0;
				else if(outf > 4095) outf = 4095;

				m_actuator->u = (uint16_t) outf;

#if defined(WT_CALIBRATION_PROCESS) || defined(WT_DEBUG)
				printf("[WT %u - %u] ", m_actuator->wtId, (m_device->performValues).pckTotal);
				printf("x_ref= ");
				printfFloat(*x_ref);
				printf(" cm ; x1=");
				printfFloat((nx_float) (m_device->data).tankLevel[0]/WT_CALIBRATION);
				printf(" cm ; x2=");
				printfFloat((nx_float) (m_device->data).tankLevel[1]/WT_CALIBRATION);
				printf(" cm ; xi=");
				printfFloat(x_int[m_actuator->wtId-1]);
				printf(" cm ; out=");
				printfFloat((0.0006151*outf + 0.0009710)*6);
				printf("V \n");
				printfflush();
#endif
				setAddressingFields(m_actuator->wtId*2);

				if (call MCPS_DATA.request (
								&m_frame, // frame,
								sizeof(EncMsg2SensorsAct), // payloadLength,
								0, // msduHandle,
								TX_OPTIONS_ACK // TxOptions,
						) != IEEE154_SUCCESS) {
					//call Leds.led2Toggle(); //fail!
				}
				else {
					call Leds.led1Toggle();
				}
			}
		}
		return frame;
	}
	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	) {
		if (status != IEEE154_SUCCESS) {
			call Leds.led0Toggle();
		}
	}

#ifndef TKN154_BEACON_DISABLED
	event void IEEE154TxBeaconPayload.aboutToTransmit() {}

	event void IEEE154TxBeaconPayload.setBeaconPayloadDone(void *beaconPayload, uint8_t length) {}

	event void IEEE154TxBeaconPayload.modifyBeaconPayloadDone(uint8_t offset, void *buffer, uint8_t bufferLength) {}

	event void IEEE154TxBeaconPayload.beaconTransmitted()
	{
		ieee154_macBSN_t beaconSequenceNumber = call MLME_GET.macBSN();
		m_sfStartRef = call SlotAlarm.getNow();
		if (beaconSequenceNumber & 1)
		call Leds.led2On();
		else
		call Leds.led2Off();
	}
#endif
	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/
	/************************************************************************************
	 *  DEVICE -----15.4----- BASE STATION -------EncMsg------- PC
	 ************************************************************************************/

	//Send to the serial port
	task void uartSendTask() {
		message_t* msg;

		atomic {
			if (uartIn == uartOut && !uartFull) {
				uartBusy = FALSE;
				return;
			}
		}

		msg = uartQueue[uartOut];

		if (call UartSend.send(AM_BROADCAST_ADDR, uartQueue[uartOut], m_payloadLenPC ) == SUCCESS) {
			//call Leds.led1Toggle();
		} else {
			call Leds.led0Toggle();
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
			call Leds.led0Toggle();
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
	 *  PC -----EncMsg----- BASE SATITON -------15.4------- DEVICE
	 ************************************************************************************/
	event message_t *UartReceive.receive(message_t *msg,
			void *payload,
			uint8_t len) {
		message_t* ret = msg;

		call Leds.led0Toggle();

		setAddressingFields(BROADCAST_DESTINATION);

		if (call MCPS_DATA.request ( &m_frame, // frame,
						len, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led0On(); //fail!
		} else {
			call Leds.led0Off();
			//call Leds.led1Toggle();
		}
		return ret;
	}

	/************************************************************************************
	 * TOOLS
	 ************************************************************************************/
	void printfFloat(float toBePrinted) {
		uint32_t fi, f0, f1, f2;
		char c;
		float f = toBePrinted;

		if (f<0) {
			c = '-'; f = -f;
		} else {
			c = ' ';
		}

		// integer portion.
		fi = (uint32_t) f;

		// decimal portion...get index for up to 3 decimal places.
		f = f - ((float) fi);
		f0 = f*10; f0 %= 10;
		f1 = f*100; f1 %= 10;
		f2 = f*1000; f2 %= 10;
		printf("%c%ld.%d%d%d", c, fi, (uint8_t) f0, (uint8_t) f1,
				(uint8_t) f2);
	}
}