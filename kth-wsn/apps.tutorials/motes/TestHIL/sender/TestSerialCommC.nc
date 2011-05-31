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
 * 
 * @version  $Revision: 1.0 Date: 2011/04/19 $ 
 */
#include <Timer.h>
#include "TestSerialComm.h"

#include "TKN154_PHY.h"

module TestSerialCommC {
	uses {
		interface Boot;

		interface Leds;
		interface Timer<TMilli> as Timer0;

		interface SplitControl as SerialControl;
		interface AMSend as UartSend[am_id_t id];
		interface Receive as UartReceive[am_id_t id];
		interface Packet as UartPacket;
		interface AMPacket;

		interface Packet;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;

		interface Random;
	}

}
implementation {

	message_t pkt;
	message_t m_frame;
	TestSerialCommMsg* sendpkt;

	uint8_t m_payloadLen = sizeof(TestSerialCommMsg);
	bool busy = FALSE;

	void setAddressingFields(uint16_t address);
	void startApp();
	task void sendPacket();

	void setLeds(uint16_t val) {
		if (val & 0x01)
		call Leds.led0On();
		else
		call Leds.led0Off();
		if (val & 0x02)
		call Leds.led1On();
		else
		call Leds.led1Off();
		if (val & 0x04)
		call Leds.led2On();
		else
		call Leds.led2Off();
	}

	event void Boot.booted() {
		sendpkt = (TestSerialCommMsg*)(call Packet.getPayload(&m_frame, m_payloadLen));

		call AMPacket.setSource(&pkt, TOS_NODE_ID);
		call SerialControl.start();
		call MLME_RESET.request(TRUE);
	}

	event void SerialControl.startDone(error_t err) {
		if (err == SUCCESS) {
		}
		else {
			call SerialControl.start();
		}
	}

	event void SerialControl.stopDone(error_t err) {}

	event void Timer0.fired() {
	}

	event void UartSend.sendDone[am_id_t id](message_t* msg, error_t err) {
		if (&pkt == msg) {
			busy = FALSE;
		}
	}

	event message_t* UartReceive.receive[am_id_t id](message_t* msg, void* payload, uint8_t len) {
		if (len == m_payloadLen) {
			TestSerialCommMsg* btrpkt = (TestSerialCommMsg*) payload;
			setLeds(btrpkt->counter);
			memcpy(sendpkt, btrpkt, m_payloadLen);

			post sendPacket();

		}
		return msg;
	}
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
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = SENSOR_ADRESS; // destination

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(ACTUATOR_ADRESS);
		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);

	}

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

		if (call MCPS_DATA.request (
						&m_frame, // frame,
						m_payloadLen, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led2Toggle(); //fail!
		} else {
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
		if (status == IEEE154_SUCCESS) {
			call Leds.led1Toggle();
		} else {
			call Leds.led0Toggle();
		}

	}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {return frame;}
	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/

}
