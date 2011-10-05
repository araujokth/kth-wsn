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
#include <Timer.h>

#include "TKN154.h"
#include "app_sensors.h"
#include "app_wt_calibration.h"
#include "printf.h"

module SensorC
{
	uses {
		interface Boot;

		
		interface Leds;

		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_SET;
		interface MLME_GET;
		interface MLME_SCAN;
		interface MLME_SYNC;
		interface MLME_BEACON_NOTIFY;
		interface MLME_SYNC_LOSS;
		interface MLME_GTS;
		interface IEEE154Frame as Frame;
		interface IEEE154BeaconFrame as BeaconFrame;
		interface Packet;

		//interface SuperframeStructure as SF;

		interface GtsUtility;
		interface GetNow<bool> as IsGtsOngoing;

		interface Notify<bool> as IsEndSuperframe;

		interface LocalTime<TSymbolIEEE802154>;
		
		interface Alarm<TSymbolIEEE802154, uint32_t> as AlarmGeneratePacket; // To "control" the generation of packets
		

	}

}

implementation
{

	// Type of packets exchanged
	XtraSensorMsg* xtraSensorMsg;
	Control2MoteMsg control2MoteMsg;

	uint8_t m_payloadLen;
	uint8_t m_payloadLenControl;

	message_t m_frame;

	ieee154_PANDescriptor_t m_PANDescriptor;

	bool m_wasScanSuccessful;

	void startApp();

	task void sendPacketCAP();

	void printfValues();

	// To control the phase and type of the experiment
	uint8_t m_state, typeExperiment;
	uint8_t init;
	enum {
		PHASE_1 = 0x01,
		PHASE_2 = 0x02,
		PHASE_3 = 0x03, // End of the experiment
	};

	// Define the type of experiment
	enum {
		PERIODIC_SLOW = 0x00,
		PERIODIC_FAST = 0x01, // Periodic packet generation rate
		BURSTY = 0x02, // Bursty packet generation rate
	};

	// Alarm times
	enum {

		K_INTERVAL_TIME = 65,
	};
	uint32_t periodGeneration;
	uint32_t beaconIntervalSymb;

	uint16_t numBeaconsPhase2;

	// Performance Evaluation Variables

	uint16_t packetsGenerated;
	uint16_t numACKsReceived;
	uint32_t delayPacket;
	uint32_t timestampACK;
	uint32_t initialTimeOffset;

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

	void printfValues()
	{

		if (m_state==PHASE_2) {
			printf("%lu ",packetsGenerated);
			printf("%lu ",numACKsReceived);
			printf("%lu ",delayPacket);
			printf("%lu\n",timestampACK);
		}

	}

	//boot start
	event void Boot.booted()
	{

		xtraSensorMsg = (XtraSensorMsg*)(call Packet.getPayload(&m_frame,m_payloadLen));
		m_payloadLen = sizeof(XtraSensorMsg);
		m_payloadLenControl = sizeof(Control2MoteMsg);

		control2MoteMsg.ref = 0;

		call MLME_RESET.request(TRUE);

		m_state = PHASE_1;
		typeExperiment = BURSTY;
		init=TRUE;

		periodGeneration = 0;
		beaconIntervalSymb = (uint32_t) (1 << BEACON_ORDER)*IEEE154_aBaseSuperframeDuration;

		numBeaconsPhase2=0;

		// Performance Evaluation 
		packetsGenerated = 0;
		numACKsReceived = 0;
		timestampACK=0;
		initialTimeOffset=0;

	}

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status == IEEE154_SUCCESS)
		startApp();
	}

	task void sendPacketCAP() {
		if (!m_wasScanSuccessful) {
			return;
		} else {

			xtraSensorMsg->timestamp = call LocalTime.get();

			if( call MCPS_DATA.request (
							&m_frame, // frame,
							m_payloadLen, // payloadLength,
							0, // msduHandle,
							TX_OPTIONS_ACK // TxOptions,
					) != IEEE154_SUCCESS) {
				
			} else {
				call Leds.led0Toggle();
				packetsGenerated = packetsGenerated + 1;
			}

		}
	}

	async event void AlarmGeneratePacket.fired() {
		post sendPacketCAP();
		call AlarmGeneratePacket.start(periodGeneration);
	}

	

	/**************************************************************
	 * IEEE 802.15.4
	 *************************************************************/

	void startApp() {
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);

		// scan only the channel where we expect the coordinator
		channelMask = ((uint32_t) 1) << RADIO_CHANNEL;

		// we want all received beacons to be signalled 
		// through the MLME_BEACON_NOTIFY interface, i.e.
		// we set the macAutoRequest attribute to FALSE
		call MLME_SET.macAutoRequest(FALSE);
		call MLME_SET.macRxOnWhenIdle(FALSE);
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

	}
	uint8_t getPendAddrSpecOffset(uint8_t *macPayloadField) {
		uint8_t gtsDescriptorCount = (macPayloadField[BEACON_INDEX_GTS_SPEC]
				& GTS_DESCRIPTOR_COUNT_MASK) >> GTS_DESCRIPTOR_COUNT_OFFSET;

		return BEACON_INDEX_GTS_SPEC + 1
		+ ((gtsDescriptorCount > 0) ? GTS_DIRECTION_FIELD_LENGTH
				+ gtsDescriptorCount * GTS_LIST_MULTIPLY : 0);
	}
	event message_t* MLME_BEACON_NOTIFY.indication (message_t* frame)
	{
		// received a beacon frame
		ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();
		ieee154_macBSN_t beaconSequenceNumber = call BeaconFrame.getBSN(frame);
		uint8_t beaconOrder = call MLME_GET.macBeaconOrder();

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
			memcpy(&control2MoteMsg,call BeaconFrame.getBeaconPayload(frame), m_payloadLenControl);
			if(control2MoteMsg.ref==10)
			{
				atomic m_state = PHASE_2;
				numBeaconsPhase2=numBeaconsPhase2+1;

				if (init) {
					init = FALSE;
					initialTimeOffset=call LocalTime.get();
					call AlarmGeneratePacket.start(TOS_NODE_ID*K_INTERVAL_TIME); // At start, nodes generate first packet in different moments in time
					if (typeExperiment == PERIODIC_SLOW) {
						periodGeneration = 5*beaconIntervalSymb;
					} else if (typeExperiment == PERIODIC_FAST) {
						periodGeneration = 1*beaconIntervalSymb;
					} else if (typeExperiment == BURSTY) {
						periodGeneration = 1*beaconIntervalSymb;
					}

				}
				else {
					// For the bursty experiment, we change the packet generation rate during the experiment
					if (typeExperiment == BURSTY) {
						if ((numBeaconsPhase2 >= 25 && numBeaconsPhase2 < 100) ||
								(numBeaconsPhase2 >= 125 && numBeaconsPhase2 < 190) ||
								(numBeaconsPhase2 >= 215) ) {
							periodGeneration = 5*beaconIntervalSymb;
						} else {
							periodGeneration = 1*beaconIntervalSymb;
						}
					}

				}

			} else if (control2MoteMsg.ref==20) {
				atomic m_state = PHASE_3;

			}

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
			

		} else
		startApp();
	}

	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	)
	{
		XtraSensorMsg *m_device;
		m_device = (XtraSensorMsg*)(call Packet.getPayload(msg,sizeof(XtraSensorMsg)));

		if (status == IEEE154_SUCCESS) {
			call Leds.led1Toggle();
			numACKsReceived = numACKsReceived + 1; 
			delayPacket = ((call LocalTime.get())-(m_device->timestamp));
			timestampACK = ((call LocalTime.get())-initialTimeOffset);
			printfValues();
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
		call Leds.led1Off();
		call Leds.led2Off();

		startApp();
	}

	event message_t* MCPS_DATA.indication (message_t* frame)
	{
		return frame;
	}

	event void MLME_GTS.confirm (
			uint8_t GtsCharacteristics,
			ieee154_status_t status
	) {}

	event void MLME_GTS.indication (
			uint16_t DeviceAddress,
			uint8_t GtsCharacteristics,
			ieee154_security_t *security
	) {}
	event void IsEndSuperframe.notify( bool val ) {

	}
}//end of implementation
