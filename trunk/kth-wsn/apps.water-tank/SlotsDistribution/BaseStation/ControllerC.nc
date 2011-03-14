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
		interface Alarm<TSymbolIEEE802154,uint32_t> as EndCapPeriod;

		interface IEEE154TxBeaconPayload;
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
	message_t m_frame2Pc;

	EncMsg2SensorsAct *m_sensor;

	bool m_wasScanSuccessful;

	uint8_t m_payloadLen = sizeof(SensingMsg);
	uint8_t m_payloadLenSend = sizeof(EncMsg2SensorsAct);
	uint16_t m_payloadLenActuation = sizeof(ActuationMatrixMsg);
	uint16_t m_payloadLenSensor = sizeof(SensorMatrixMsg);

	void setAddressingFields(uint16_t address);
	ieee154_PANDescriptor_t m_PANDescriptor;

	// WATER TANKS MANAGEMENT
	ActuationMatrixMsg actuationMatrix;
	SensorMatrixMsg* sensorMatrix;

	// TDMA MANAGEMENT
	uint32_t m_sfStartRef;
	uint32_t m_capDuration;
	uint32_t m_slotDuration;

	uint8_t m_state;
	enum {

		INITIALIZATION = 0x01, //wiat for the beacon and sysnc with the ADC
		IDLE = 0x2,
		RUNNING = 0x4, // runing
	};

	void setAddressingFields(uint16_t address);
	void printfFloat(float toBePrinted);
	uint8_t getSlotNumber();
	task void updateIntervalRequest();

	/**----------------------------------------------*/
	event void Boot.booted() {
		uint8_t i = 0;

#if defined(WT_CALIBRATION) || defined(WT_USERBUTTON)
		m_state = INITIALIZATION;
		call UserButton.enable();
#else
		m_state = IDLE;
#endif

#ifdef WT_CALIBRATION
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
		sensorMatrix = (SensorMatrixMsg* )(call Packet.getPayload(&m_frame2Pc, m_payloadLenSensor));

		m_sensor = (EncMsg2SensorsAct*)(call Packet.getPayload(&m_frame,m_payloadLenSend));
		m_sensor->u = 0;
		m_sensor->wtId = 0;

		call Leds.led0On();
		call Leds.led1On();

		// set the beacon variables
		m_capDuration = (((uint32_t) 1 << SUPERFRAME_ORDER) * (uint32_t) IEEE154_aBaseSuperframeDuration);
		m_slotDuration = (uint32_t) m_capDuration / (NUMBER_WT*2+1);
		m_sfStartRef = 0;

		//reset all the motes
		for (i=0; i < NUMBER_WT; i++) actuationMatrix.u[i] = 0;

		call SerialControl.start();
		call MLME_RESET.request(TRUE);
	}

	event void UserButton.notify( button_state_t state ) {
		if ( state == BUTTON_PRESSED ) {
		} else if ( state == BUTTON_RELEASED ) {
			atomic m_state = RUNNING;
		}
	}

	/*********************************************************************
	 * 802.15.4 functions
	 *********************************************************************/
	void startApp()
	{
		call Leds.led0Off();
		call Leds.led1Off();

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(COORDINATOR_ADDRESS);

		call MLME_SET.macAssociationPermit(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);
		call MLME_SET.macMaxFrameRetries(0);

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

		uint8_t wtId = 0;
		// We assume that the packets with length equal to EncMsg
		// are a EncMsg
		if (call Frame.getPayloadLength(frame) == m_payloadLen &&
				call Frame.getFrameType(frame) == FRAMETYPE_DATA && m_state == RUNNING) {
			//call Leds.led1Toggle();

			call Frame.getSrcAddr(frame, &deviceAddr);
			wtId = (deviceAddr.shortAddress + 1)/2;

			atomic {
				m_device = (SensingMsg*)(call Packet.getPayload(frame,m_payloadLen));

				sensorMatrix->tankLevel1[wtId-1] = (m_device->data).tankLevel[0];
				sensorMatrix->tankLevel2[wtId-1] = (m_device->data).tankLevel[1];
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
		} else if ( m_state == INITIALIZATION) m_state = IDLE;

	}

	event void IEEE154TxBeaconPayload.aboutToTransmit() {}

	event void IEEE154TxBeaconPayload.setBeaconPayloadDone(void *beaconPayload, uint8_t length) {}

	event void IEEE154TxBeaconPayload.modifyBeaconPayloadDone(uint8_t offset, void *buffer, uint8_t bufferLength) {}

	event void IEEE154TxBeaconPayload.beaconTransmitted()
	{
		ieee154_macBSN_t beaconSequenceNumber = call MLME_GET.macBSN();
		m_sfStartRef = call SlotAlarm.getNow();
		//Set when we need to transmit
		call SlotAlarm.startAt(m_sfStartRef, m_slotDuration);
		call EndCapPeriod.startAt(m_sfStartRef, m_capDuration);

		if (beaconSequenceNumber & 1)
		call Leds.led2On();
		else
		call Leds.led2Off();
	}
	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/
	/*********************************************************************
	 * TDMA functions
	 *********************************************************************/
	async event void EndCapPeriod.fired() {
		// end of the superframe. Time to send the matrices
		post updateIntervalRequest();
		call SlotAlarm.stop();
	}

	async event void SlotAlarm.fired() {
		if (m_state == RUNNING || m_state == INITIALIZATION) {
			uint8_t slotNumber = getSlotNumber();

			m_sensor->wtId = (slotNumber+1)/2;

			if (m_sensor->wtId <= NUMBER_WT) {

				m_sensor->u = actuationMatrix.u[m_sensor->wtId-1];

				setAddressingFields(slotNumber+1);

				if (call MCPS_DATA.request (
								&m_frame, // frame,
								sizeof(EncMsg2SensorsAct), // payloadLength,
								0, // msduHandle,
								TX_OPTIONS_ACK // TxOptions,
						) != IEEE154_SUCCESS) {
					//call Leds.led2Toggle(); //fail!
				}
				else {
					//call Leds.led1Toggle();
				}
				call SlotAlarm.startAt(call SlotAlarm.getAlarm(), 2*m_slotDuration); //Every two slots
			}else{
			}
		}
	}

	uint8_t getSlotNumber() {
		return (uint32_t) (call SlotAlarm.getNow() - m_sfStartRef) / m_slotDuration;
	}

	/************************************************************************************
	 *  SERIAL COMM
	 ************************************************************************************/

	// Send the new sensor values to the PC
	task void updateIntervalRequest() {

		if (call UartSend.send(AM_BROADCAST_ADDR, &m_frame2Pc, m_payloadLenSensor ) == SUCCESS) {
			call Leds.led1Toggle();
		} else {
			post updateIntervalRequest();
		}
	}

	event void SerialControl.startDone(error_t error) {}
	event void SerialControl.stopDone(error_t error) {}
	event void UartSend.sendDone(message_t* msg, error_t error) {}

	/************************************************************************************
	 *  PC -----EncMsg----- BASE SATITON -------15.4------- DEVICE
	 ************************************************************************************/
	event message_t *UartReceive.receive(message_t *msg,
			void *payload,
			uint8_t len) {
		if (len == m_payloadLenActuation) {
			memcpy(&actuationMatrix, call Packet.getPayload(msg, m_payloadLenSensor), m_payloadLenSensor);
		}
		return msg;
	}

	/************************************************************************************
	 * TOOLS
	 ************************************************************************************/
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