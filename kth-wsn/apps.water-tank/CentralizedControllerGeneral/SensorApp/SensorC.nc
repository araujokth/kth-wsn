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
 * @version  $Revision: 1.1 Date: 2011/03/14 $ 
 */

#include "app_sensors_nbe.h"
#include "app_wt_calibration.h"

#include "TKN154_PHY.h"
#include <Timer.h>

module SensorC {
	uses {
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as TimerSend;

		interface Resource;
		interface Msp430Adc12MultiChannel as MultiChannel;

		interface Packet;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;

		interface LocalTime<TSymbolIEEE802154>;

		interface SplitControl as SerialControl;
		interface AMSend as UartSend;
		interface Receive as UartReceive;
		interface AMPacket as UartAMPacket;

#ifndef TKN154_BEACON_DISABLED
		interface MLME_SCAN;
		interface MLME_SYNC;
		interface MLME_BEACON_NOTIFY;
		interface MLME_SYNC_LOSS;
		interface IEEE154BeaconFrame as BeaconFrame;

#endif

	}
	provides interface AdcConfigure<const msp430adc12_channel_config_t*>;

}
implementation {
	uint16_t buffer[3];

	uint16_t rate = DEFAULT_RATE;

	message_t m_frame;
	uint8_t m_payloadLen = sizeof(SensingMsg);
	SensingMsg *sendpkt;
	SensorValues sensorValues;

	message_t m_frame2Serial;
	//	uint8_t m_payloadLenSerial = sizeof(SensorValues);
	uint8_t m_payloadLenSerial = sizeof(SensingMsg);
	SensingMsg *sendpkt2Serial;

	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	// S E R I A L
	message_t uartQueueBufs[UART_QUEUE_LEN];
	message_t * ONE_NOK uartQueue[UART_QUEUE_LEN];
	uint8_t uartIn, uartOut;
	bool uartBusy, uartFull;
	task void uartSendTask();

	// P E R F O R M A N C E   E V A L U A T I O N
	PerformanceParams sendPerfParams; // to compute the performance before sending

#ifndef TKN154_BEACON_DISABLED
	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;
#endif

	task void getData();
	task void sendPacket();
	void startApp();

	/**----------------------------------------------*/
	event void Boot.booted() {
		uint8_t i = 0;
		sendpkt = (SensingMsg*) (call Packet.getPayload(&m_frame, m_payloadLen));

		sendPerfParams.pckTotal = 0;
		sendPerfParams.pckSuccess = 0;

		sensorValues.tankLevel[0] = 0;
		sensorValues.tankLevel[1] = 0;

		for (i = 0; i < UART_QUEUE_LEN; i++)
		uartQueue[i] = &uartQueueBufs[i];

		uartIn = uartOut = 0;
		uartBusy = FALSE;
		uartFull = FALSE;

		call SerialControl.start();
		call MLME_RESET.request(TRUE);
	}

	/*********************************************************************
	 * 802.15.4 functions
	 *********************************************************************/
	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status == IEEE154_SUCCESS)
		startApp();
	}
	void startApp()
	{
#ifndef TKN154_BEACON_DISABLED
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = COORDINATOR_ADDRESS; // destination

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);

		// scan only the channel where we expect the coordinator
		channelMask = ((uint32_t) 1) << RADIO_CHANNEL;

		// we want all received beacons to be signalled 
		// through the MLME_BEACON_NOTIFY interface, i.e.
		// we set the macAutoRequest attribute to FALSE
		call MLME_SET.macAutoRequest(FALSE);
		m_wasScanSuccessful = FALSE;
		call MLME_SCAN.request (
				PASSIVE_SCAN, // ScanType
				channelMask, // ScanChannels
				scanDuration, // ScanDuration
				0x00, // ChannelPage
				0, // EnergyDetectListNumEntries
				NULL, // EnergyDetectList
				0, // PANDescriptorListNumEntries
				NULL, // PANDescriptorList
				0 // security
		);

#else
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = COORDINATOR_ADDRESS; // destination

		call MLME_SET.macRxOnWhenIdle(TRUE);
		call MLME_SET.macShortAddress(TOS_NODE_ID);

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
		
		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);

		call Resource.request();

#endif
	}

	event void MLME_START.confirm(ieee154_status_t status) {}

#ifndef TKN154_BEACON_DISABLED
	event message_t* MLME_BEACON_NOTIFY.indication (message_t* frame)
	{
		// received a beacon frame
		ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();
		ieee154_macBSN_t beaconSequenceNumber = call BeaconFrame.getBSN(frame);

		if (!m_wasScanSuccessful) {
			// received a beacon during channel scanning
			if (call BeaconFrame.parsePANDescriptor(
							frame, RADIO_CHANNEL, page, &m_PANDescriptor) == SUCCESS) {
				// let's see if the beacon is from our coordinator...
				if (m_PANDescriptor.CoordAddrMode == ADDR_MODE_SHORT_ADDRESS &&
						m_PANDescriptor.CoordPANId == PAN_ID &&
						m_PANDescriptor.CoordAddress.shortAddress == COORDINATOR_ADDRESS) {
					// yes! wait until SCAN is finished, then syncronize to the beacons
					m_wasScanSuccessful = TRUE;
				}
			}
		} else {
			// received a beacon during synchronization, toggle LED2
			if (beaconSequenceNumber & 1)
			call Leds.led2On();
			else
			call Leds.led2Off();
		}

		return frame;
	}

	event void MLME_SCAN.confirm (
			ieee154_status_t status,
			uint8_t ScanType,
			uint8_t ChannelPage,
			uint32_t UnscannedChannels,
			uint8_t EnergyDetectListNumEntries,
			int8_t* EnergyDetectList,
			uint8_t PANDescriptorListNumEntries,
			ieee154_PANDescriptor_t* PANDescriptorList
	)
	{

		if (m_wasScanSuccessful) {

			// we received a beacon from the coordinator before
			call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
			call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
			call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);

			call Frame.setAddressingFields(
					&m_frame,
					ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
					ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
					m_PANDescriptor.CoordPANId, // DstPANId,
					&m_PANDescriptor.CoordAddress, // DstAddr,
					NULL // security
			);

			call Resource.request();
		} else
		startApp();
	}

	event void MLME_SYNC_LOSS.indication(
			ieee154_status_t lossReason,
			uint16_t PANId,
			uint8_t LogicalChannel,
			uint8_t ChannelPage,
			ieee154_security_t *security)
	{
		m_wasScanSuccessful = FALSE;
		call Leds.led1Off();
		call Leds.led2Off();
		call TimerSend.stop();

		startApp();
	}
#endif

	/*************************** Send functions *************************/
	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	)
	{
		// reset the packet counters
		if (sendPerfParams.pckTotal >= SLIDING_WINDOW) {
			sendPerfParams.pckTotal = 0;
			sendPerfParams.pckSuccess = 0;
			return;
		}
		if (status == IEEE154_SUCCESS) {
			sendPerfParams.pckSuccess ++;
			call Leds.led1Toggle();
		} else
			call Leds.led0Toggle();

		sendPerfParams.delay = timestamp - sendPerfParams.delay;

	}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {return frame;}
	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/
	task void sendErrorPacket() {

		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = TOS_NODE_ID+1; // destination

		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security

		);
		if (call MCPS_DATA.request (
						&m_frame, // frame,
						m_payloadLen, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led2Toggle(); //fail!
		}
		else {
			//call Leds.led1Toggle();
		}
	}
	task void sendPacket() {
		atomic {
			memcpy(&(sendpkt->data), &sensorValues, sizeof(SensorValues) );
		}

		// set the timestamp and copy the new performance data
		memcpy(&(sendpkt->performValues), &sendPerfParams, sizeof(PerformanceParams) );

		//set the new timestamp value
		sendPerfParams.delay = call LocalTime.get();

		sendPerfParams.pckTotal ++;

		if (!uartFull) {
			uartQueue[uartIn] = &m_frame;
			uartIn = (uartIn + 1) % UART_QUEUE_LEN;

			if (!uartBusy)
			{
				post uartSendTask();
				uartBusy = TRUE;
			}
		}

		if (call MCPS_DATA.request (
						&m_frame, // frame,
						m_payloadLen, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led2Toggle(); //fail!
		}
		else {
			//call Leds.led1Toggle();
		}

		if (sensorValues.tankLevel[0] > WT_LIMIT*WT_CALIBRATION) {
			post sendErrorPacket();
		}
	}
	event void TimerSend.fired() {
		post getData();
	}

	/*********************************************************************
	 * Multi Channel functions
	 *********************************************************************/
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}

	task void getData()
	{
		call MultiChannel.getData();
	}

	event void Resource.granted()
	{
		atomic {
			adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}};

			if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) != SUCCESS) {
				call Leds.led0On();
			}
		}

		call TimerSend.startPeriodic(rate);
	}

	async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = COORDINATOR_ADDRESS; // destination

		sensorValues.tankLevel[0] = buf[1];
		sensorValues.tankLevel[1] = buf[2];

		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security

		);

		post sendPacket();
	}
	/*********************************************************************
	 * END Multi Channel functions
	 *********************************************************************/

	/*********************************************************************
	 * Serial functions
	 *********************************************************************/
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

	event message_t *UartReceive.receive(message_t *msg,
			void *payload,
			uint8_t len) {
		return msg;
	}

	task void uartSendTask() {
		message_t* msg;

		atomic {
			if (uartIn == uartOut && !uartFull) {
				uartBusy = FALSE;
				return;
			}
		}

		msg = uartQueue[uartOut];

		if (call UartSend.send(AM_BROADCAST_ADDR, uartQueue[uartOut], m_payloadLenSerial ) == SUCCESS) {
			//call Leds.led1Toggle();
		} else {
			call Leds.led0Toggle();
			post uartSendTask();
		}
	}
	/*********************************************************************
	 * End Serial functions
	 *********************************************************************/
}