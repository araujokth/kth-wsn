/*
 * Copyright (c) 2010, KTH Royal Institute of Technology
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
 * - Neither the name of the KTH Royal Institute of Technology nor the names
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
 */
/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @version $Revision: 1.0 $ $Date: 2010/01/25 $
 */
#include "printf.h"
#include "TKN154.h"

#include "app_crane.h"

module EDReceiverP @ safe() {

uses {
	interface Boot;
	interface Leds;
	interface LocalTime<TSymbolIEEE802154> as LocalTime;

#ifdef PIN_DEBUG
	interface GeneralIO as PinA; //channel A
#endif

	//Serial UART0
	interface UartStream;
	interface Resource as UartResource;

	// For 802.15.4
	interface Packet;
	interface MCPS_DATA;
	interface MLME_RESET;
	interface MLME_START;
	interface MLME_SET;
	interface MLME_GET;
	interface IEEE154Frame as Frame;

#ifndef TKN154_BEACON_DISABLED
	interface MLME_SCAN;
	interface MLME_SYNC;
	interface MLME_BEACON_NOTIFY;
	interface MLME_SYNC_LOSS;
	interface IEEE154BeaconFrame as BeaconFrame;
#endif
}
provides interface Msp430UartConfigure;
}
implementation {

	// variables to send the message
	message_t m_frame;
	CraneSensors *m_crane;

	uint8_t m_payloadLenSend = sizeof(CraneSensors);
	task void sendValue();
	void setAddressingFields(uint16_t address);

#ifndef TKN154_BEACON_DISABLED
	bool m_wasScanSuccessful;
	void setAddressingFields(uint16_t address);
	ieee154_PANDescriptor_t m_PANDescriptor;
#endif

	// variables for the received value
	uint32_t receivedValue;
	uint8_t idx; //to create each float
	uint8_t idx_v;
	bool debug;
	void printfFloat(float toBePrinted);
	task void plotResults();

	// P E R F O R M A N C E   E V A L U A T I O N
	PerformanceParams sendPerfParams; // to compute the performance before sending

	msp430_uart_union_config_t msp430_uart_config = {
		{
			utxe : 1,
			urxe : 1,
			ubr : BAUD_RATE_UBR,
			umctl : BAUD_RATE_UMCTL,
			ssel : 0x02,
			pena : 0,
			pev : 0,
			spb : 0,
			clen : 1,
			listen : 0,
			mm : 0,
			ckpl : 0,
			urxse : 0,
			urxeie : 1,
			urxwie : 0
		}
	};
	async command msp430_uart_union_config_t* Msp430UartConfigure.getConfig() {
		return &msp430_uart_config;
	}

	event void Boot.booted() {
		call MLME_RESET.request(TRUE);

		atomic {
			m_crane = (CraneSensors*)(call Packet.getPayload(&m_frame,m_payloadLenSend));

			m_crane->u[0] = 0.00;
			m_crane->u[1] = 0.00;

			idx = 0; idx_v = 0;
			receivedValue = 0;

			sendPerfParams.pckTotal = 0;
			sendPerfParams.pckSuccess = 0;
		}
#ifdef PIN_DEBUG
		call PinA.makeOutput();
		debug = TRUE;
#endif

	}
	/*********************************************************************
	 * S E R I A L   R E C E I V E R
	 *********************************************************************/
	event void UartResource.granted() {
		call UartStream.enableReceiveInterrupt();
	}
	async event void UartStream.sendDone( uint8_t* buf, uint16_t len, error_t error ) {}
	async event void UartStream.receiveDone( uint8_t* buf, uint16_t len, error_t error ) {}

	async event void UartStream.receivedByte( uint8_t byte ) {
		atomic {
			receivedValue |= (( (uint32_t) byte ) << idx*8 );
			idx = (idx + 1 ) % sizeof(float);

			if (idx == 0 ) {
				m_crane->u[idx_v] = *(float*) &receivedValue;
				idx_v = (idx_v +1 ) % (NUM_CONTROL + NUM_CONTROL);
				receivedValue = 0;

				if (idx_v == 0) {
#ifdef PIN_DEBUG
					if (debug){ call PinA.set(); debug = FALSE;}
					else{ call PinA.clr(); debug = TRUE;}
					 
#endif
					//post plotResults();
					call UartResource.release();
					post sendValue();
				}
			}
		}
	}

	task void sendValue() {
		atomic {
			sendPerfParams.timestamp = call LocalTime.get();

			// set the timestamp and copy the new performance data
			memcpy(&(m_crane->performValues), &sendPerfParams, sizeof(PerformanceParams) );

			//set the new timestamp value

			sendPerfParams.pckTotal ++;

			setAddressingFields(ACTUATOR_ADDRESS);

			if (call MCPS_DATA.request (
							&m_frame, // frame,
							m_payloadLenSend, // payloadLength,
							0, // msduHandle,
							TX_OPTIONS_ACK // TxOptions,
					) != IEEE154_SUCCESS) {
				call Leds.led2Toggle(); //fail!
			}
			else {
				call Leds.led1Toggle();

			}
		}
	}

	task void plotResults() {
		uint8_t i = 0;
		printf("\n u[]=");
		for (i = 0; i < NUM_CONTROL; i++) {
			printfFloat(m_crane->u[i]);
			printf(" ");
		}
		printf("\n");

		printfflush();
	}

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
	/*********************************************************************
	 * E N D   S E R I A L   R E C E I V E R
	 *********************************************************************/

	/*********************************************************************
	 * I E E E   8 0 2 . 1 5 . 4
	 *********************************************************************/
	void setAddressingFields(uint16_t address) {
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = address;

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
#ifndef TKN154_BEACON_DISABLED
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;
#endif
		call Leds.led0Off();
		call Leds.led1Off();
		call Leds.led2Off();

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(ED_ADDRESS);
		call MLME_SET.macRxOnWhenIdle(FALSE);
		call MLME_SET.macMaxCSMABackoffs(M);
		call MLME_SET.macMinBE(M_0);
		call MLME_SET.macMaxBE(M_B);
		call MLME_SET.macMaxFrameRetries(N);

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
		call UartResource.request();

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
			call Leds.led0Off();
			call Leds.led1Off();
			call Leds.led2On();
			// we received a beacon from the coordinator before
			call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
			call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
			call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);

		} else {
			call Leds.led0On();
			call Leds.led1On();
			call Leds.led2Off();

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
		call Leds.led2Off();
		startApp();
	}
#endif
	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status != IEEE154_SUCCESS) return;

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(ED_ADDRESS);
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
		} else
		call Leds.led0Toggle();

		call UartResource.request();
	}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {return frame;}
	/*********************************************************************
	 * E N D   I E E E   8 0 2 . 1 5 . 4
	 *********************************************************************/
}
