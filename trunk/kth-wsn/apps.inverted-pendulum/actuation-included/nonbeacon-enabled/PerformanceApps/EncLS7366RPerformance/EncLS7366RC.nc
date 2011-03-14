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

#include "app_sensors_nbe.h"
#include "app_perform.h"
#include "TKN154_PHY.h"
#include <Timer.h>

module EncLS7366RC {
	uses {
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as TimerSPI;
		interface Timer<TMilli> as TimerSend;
		interface Timer<TMilli> as TimerBeacon;
		interface BusyWait<TMicro,uint16_t> as BusyWait;

		interface LS7366RConfig as EncConfig;
		interface LS7366RReceive as EncReceive;
		interface Resource as EncResource;

		interface Packet;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;

		interface LocalTime<TSymbolIEEE802154>;

#ifndef TKN154_BEACON_DISABLED
		interface MLME_SCAN;
		interface MLME_SYNC;
		interface MLME_BEACON_NOTIFY;
		interface MLME_SYNC_LOSS;
		interface IEEE154BeaconFrame as BeaconFrame;

		interface Random;
#endif

	}
}
implementation {
	uint8_t count[4];
	uint8_t counter;
	uint16_t rate = DEFAULT_RATE;

	message_t m_frame;
	SensingMsg *sendpkt;
	uint8_t m_payloadLen = sizeof(SensingMsg);
	

	// P E R F O R M A N C E   E V A L U A T I O N
	PerformanceParams sendPerfParams; // to compute the performance before sending
	uint32_t timestamp2Delay;

#ifndef TKN154_BEACON_DISABLED
	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;
#endif

	uint32_t beacontime;
	uint32_t bi_bo4;
	task void sendPacket();
	void startApp();
	void setAddressingFields(uint16_t address);
	event void TimerBeacon.fired() {}
	/**----------------------------------------------*/
	event void Boot.booted() {
		sendpkt = (SensingMsg*)(call Packet.getPayload(&m_frame, m_payloadLen));
		sendPerfParams.pckTotal = 0;
		sendPerfParams.pckSuccess = 0;

		call MLME_RESET.request(TRUE);

		bi_bo4 = 246;
	}

	/**----------------------------------------------*/
	//Send packets every rate -offset

	event void TimerSend.fired() {
		//call Leds.led2Toggle();
#ifndef TKN154_BEACON_DISABLED
		if((call TimerBeacon.getNow()) - beacontime < ( (((uint32_t) 1 << (uint32_t) ((call MLME_GET.macBeaconOrder())-4)) * bi_bo4)-40))
#endif
		post sendPacket();
	}
	//Request samples every rate -0 offset
	event void TimerSPI.fired() {

		//call Leds.led0Toggle();
		call EncResource.request();
	}

	/*********************************************************************
	 * LS7366R functions
	 *********************************************************************/

	/****************** EncReceive Events ****************/
	event void EncReceive.receiveDone( uint8_t* data ) {
		//call BusyWait.wait(1000);
		//post sendPacket();
	}

	/***************** Resource event  ****************/
	event void EncResource.granted() {
		call EncReceive.receive(count);
	}

	/****************** EncConfig Events ****************/
	event void EncConfig.syncDone( error_t error ) {}

	/*********************************************************************
	 * 802.15.4 functions
	 *********************************************************************/
	void setAddressingFields(uint16_t address) {
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
	void startApp()
	{
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = CONTROLLER_ADDRESS; // destination

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);
#ifndef TKN154_BEACON_DISABLED
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
		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);

		call TimerSPI.startPeriodicAt(0,rate);
		call TimerSend.startPeriodicAt(OFFSET_SENDER, rate);
#endif
	}
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
						m_PANDescriptor.CoordAddress.shortAddress == CONTROLLER_ADDRESS) {
					// yes! wait until SCAN is finished, then syncronize to the beacons
					m_wasScanSuccessful = TRUE;
				}
			}
		} else {
			beacontime = call TimerBeacon.getNow();
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

			call TimerSPI.startPeriodicAt(0,rate);
			call TimerSend.startPeriodicAt(OFFSET_SENDER, rate);
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
	event void MLME_RESET.confirm(ieee154_status_t status)
	{
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
	event void MLME_START.confirm(ieee154_status_t status) {}

	/*************************** Send functions *************************/
	task void sendPacket() {
		//counter++;
		atomic {
			sendpkt->data = (uint16_t) ((count[0] << 8) | count[1]);
		}

		// 
		
		// set the timestamp and copy the new performance data
		memcpy(&(sendpkt->performValues), &sendPerfParams, sizeof(PerformanceParams) );
		//set the new timestamp value
		timestamp2Delay = call LocalTime.get();

		sendPerfParams.pckTotal ++;

		if (call MCPS_DATA.request (
						&m_frame, // frame,
						m_payloadLen, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led2Toggle(); //fail!
		}else {
			call Leds.led1Toggle();
		}

	}

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
		} else{
			call Leds.led0Toggle();
		}

		sendPerfParams.delay = (uint16_t) timestamp - timestamp2Delay;

	}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {return frame;}
	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/
}