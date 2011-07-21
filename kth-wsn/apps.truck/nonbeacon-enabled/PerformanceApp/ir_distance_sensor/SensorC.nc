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
 * @author Aziz Khakulov <khakulov@kth.se>
 * @author 
 * 
 * @version  $Revision: 1.0 Date: 2010/11/03 $ 
 * @modified 2011/02/01 
 */

#include "app_profile.h"
#include "TKN154_PHY.h"
#include <Timer.h>

module SensorC {
	uses {
		interface Boot;
		interface Leds;

		interface Resource;
		interface Msp430Adc12MultiChannel as MultiChannel;

		interface Packet;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;
		interface Timer<TMilli> as TimerSamples;

		interface MLME_SCAN;
		interface MLME_SYNC;
		interface MLME_BEACON_NOTIFY;
		interface MLME_SYNC_LOSS;
		interface IEEE154BeaconFrame as BeaconFrame;
		interface LocalTime<TSymbolIEEE802154>;
	}
	provides interface AdcConfigure<const msp430adc12_channel_config_t*>;

}
implementation {
	message_t m_frame;
	SensorValues sensorValues;
	uint8_t m_payloadLenSerial = sizeof(SensorValues);
	uint8_t m_payloadLen;
	SensingMsg *payloadRegion;
	ieee154_address_t deviceShortAddress;

	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_1_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;

	task void getData();
	task void sendPacket();
	void startApp();
	uint16_t buffer[4];

	/**----------------------------------------------*/

	// P E R F O R M A N C E   E V A L U A T I O N
	PerformanceParams sendPerfParams; // to compute the performance before sending
	uint32_t timestamp2Delay;

	/**----------------------------------------------*/

	event void Boot.booted() {
		m_payloadLen = sizeof(SensingMsg);
		payloadRegion = (SensingMsg*) (call Packet.getPayload(&m_frame, m_payloadLen));
		sendPerfParams.pckTotal = 0;
		sendPerfParams.pckSuccess = 0;
		sensorValues.src = TOS_NODE_ID;
		call MLME_RESET.request(TRUE);
	}
	/**----------------------------------------------*/
	event void TimerSamples.fired() {
		post getData();
	}
	/**----------------------------------------------*/
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}
	/*********************************************************************
	 * 802.15.4 functions
	 *********************************************************************/
	event void MLME_RESET.confirm(ieee154_status_t status) {

		if (status != IEEE154_SUCCESS) return;

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);
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

		startApp();
	}
	void startApp()
	{
#ifndef TKN154_BEACON_DISABLED
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;
#endif
		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);
		call MLME_SET.macRxOnWhenIdle(FALSE);
		call MLME_SET.macMaxCSMABackoffs(M);
		call MLME_SET.macMinBE(M_0);
		call MLME_SET.macMaxBE(M_B);
		call MLME_SET.macMaxFrameRetries(N);
		call Resource.request();
		call TimerSamples.startPeriodic(200);

		deviceShortAddress.shortAddress = COORDINATOR_ADDRESS;
		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);
#ifndef TKN154_BEACON_DISABLED
		// scan only the channel where we expect the coordinator
		channelMask = ((uint32_t) 1) << RADIO_CHANNEL;

		// we want all received beacons to be signalled 
		// through the MLME_BEACON_NOTIFY interface, i.e.
		// we set the macAutoRequest attribute to FALSE
		call MLME_SET.macAutoRequest(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);
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
#endif
	}

	event void MLME_START.confirm(ieee154_status_t status) {}

	event message_t* MLME_BEACON_NOTIFY.indication (message_t* frame)
	{
		// received a beacon frame
		ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();

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
			call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
			call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
			call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);
		} else {
			startApp();
		}
	}

	event void MLME_SYNC_LOSS.indication(
			ieee154_status_t lossReason,
			uint16_t PANId,
			uint8_t LogicalChannel,
			uint8_t ChannelPage,
			ieee154_security_t *security)
	{
		m_wasScanSuccessful = FALSE;
	}

	/*************************** Send functions *************************/
	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	)
	{
		// reset the packet counters
		if (sendPerfParams.pckTotal >= SLIDING_WINDOW_PERFORMANCE) {
			sendPerfParams.pckTotal = 0;
			sendPerfParams.pckSuccess = 0;
			return;
		}
		if (status == IEEE154_SUCCESS) {
			sendPerfParams.pckSuccess ++;
//			call Leds.led1Toggle();
		} else {
//			call Leds.led0Toggle();
		}
		sendPerfParams.delay = (uint16_t) timestamp - timestamp2Delay;

	}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {return frame;}

	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/
	task void sendPacket() {
		memcpy(payloadRegion, &sensorValues, sizeof(SensorValues));

		// set the timestamp and copy the new performance data
		memcpy(&(payloadRegion->performValues), &sendPerfParams, sizeof(PerformanceParams));
		//set the new timestamp value
		timestamp2Delay = call LocalTime.get();

		sendPerfParams.pckTotal ++;

		if (call MCPS_DATA.request (
						&m_frame, // frame,
						m_payloadLen, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS)
		call Leds.led2Toggle(); //fail!
	}

	/*********************************************************************
	 * Multi Channel functions
	 *********************************************************************/

	task void getData()
	{
		call Leds.led2Toggle();
		call MultiChannel.getData();
	}

	event void Resource.granted()
	{
		atomic {
			adc12memctl_t memctl[] = { {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A2, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A3, REFERENCE_VREFplus_AVss}};

			if (call MultiChannel.configure(&config, memctl, 3, buffer, 4, 0) != SUCCESS) {
				call Leds.led0Toggle();
			}
		}
	}

	async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{
		call Leds.led1Toggle();
		sensorValues.IRVal1 = buf[0];
		sensorValues.IRVal2 = buf[1];
		sensorValues.IRVal3 = buf[2];
		sensorValues.IRVal4 = buf[3];
		post sendPacket();
	}
	/*********************************************************************
	 * END Multi Channel functions
	 *********************************************************************/
}
