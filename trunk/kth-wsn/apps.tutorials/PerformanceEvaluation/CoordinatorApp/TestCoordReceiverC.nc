/*
 * Copyright (c) 2008, Technische Universitaet Berlin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of the Technische Universitaet Berlin nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * - Revision -------------------------------------------------------------
 * $Revision: 1.2 $
 * $Date: 2010/01/05 17:12:56 $
 * @author: Jan Hauer <hauer@tkn.tu-berlin.de>
 * ========================================================================
 */

#include "TKN154.h"
#include "app_profile.h"
module TestCoordReceiverC
{
	uses {
		interface Boot;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;
		interface IEEE154TxBeaconPayload;
		interface Leds;
		interface Packet;

		interface LocalTime<TSymbolIEEE802154>;

	}
}implementation {

	bool m_ledCount;

	message_t m_frame;
	uint8_t m_payloadLen;
	DummyInformationMsg* dummyMsg;

	// P E R F O R M A N C E   E V A L U A T I O N
	PerformanceParams sendPerfParams; // to compute the performance before sending
	uint32_t timestamp2Delay; // temp variable to compute the delay

	event void Boot.booted() {
		// The packet that we want to send over IEEE 802.15.4
		m_payloadLen = sizeof(DummyInformationMsg);
		dummyMsg = call Packet.getPayload(&m_frame, m_payloadLen);

		// Initialize the PerformanceParams and temp var for the delay
		sendPerfParams.pckTotal = 0;
		sendPerfParams.pckSuccess = 0;
		timestamp2Delay = 0;

		if (m_payloadLen <= call Packet.maxPayloadLength()) {
			call MLME_RESET.request(TRUE);
		}
	}

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		ieee154_address_t deviceShortAddress;

		if (status != IEEE154_SUCCESS)
		return;

		deviceShortAddress.shortAddress = 0x02; // destination

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(COORDINATOR_ADDRESS);
		call MLME_SET.macAssociationPermit(FALSE);
		call MLME_SET.macPANId(PAN_ID);

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
	}

	task void packetSendTask()
	{

		// set the timestamp and copy the new performance data
		memcpy(&(dummyMsg->performValues), &sendPerfParams, sizeof(PerformanceParams));
		//set the new timestamp value
		timestamp2Delay = call LocalTime.get();

		sendPerfParams.pckTotal ++;

		if (call MCPS_DATA.request (
						&m_frame, // frame,
						m_payloadLen, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led0Toggle(); //fail!
		} else {
			call Leds.led0Off();
		}
	}

	event message_t* MCPS_DATA.indication ( message_t* frame )
	{
		// We copy all the payload. The PerformanceParams, will be overwritten before sending
		memcpy(dummyMsg, (DummyInformationMsg*)(call Packet.getPayload(frame, m_payloadLen)), m_payloadLen );
		post packetSendTask();
		
		//call Leds.led1Toggle();

		return frame;
	}

	event void MLME_START.confirm(ieee154_status_t status) {}

	event void MCPS_DATA.confirm(
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	) {
		// reset the packet counters
		if (sendPerfParams.pckTotal >= SLIDING_WINDOW_PERFORMANCE) {
			sendPerfParams.pckTotal = 0;
			sendPerfParams.pckSuccess = 0;
			return;
		}
		if (status == IEEE154_SUCCESS) {
			sendPerfParams.pckSuccess ++;
			call Leds.led1Toggle();
		}
		sendPerfParams.delay = (uint16_t) timestamp - timestamp2Delay;
	}

	event void IEEE154TxBeaconPayload.aboutToTransmit() {}

	event void IEEE154TxBeaconPayload.setBeaconPayloadDone(void *beaconPayload, uint8_t length) {}

	event void IEEE154TxBeaconPayload.modifyBeaconPayloadDone(uint8_t offset, void *buffer, uint8_t bufferLength) {}

	event void IEEE154TxBeaconPayload.beaconTransmitted()
	{
		ieee154_macBSN_t beaconSequenceNumber = call MLME_GET.macBSN();
		if (beaconSequenceNumber & 1)
		call Leds.led2On();
		else
		call Leds.led2Off();
	}
}
