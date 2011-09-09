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
 * @author Aziz Khakulov <khakulov@kth.se> * 
 * 
 * @version  $Revision: 1.0 Date: 2011/06/07 $ 
 * @modified 2011/06/07 
 */

#include "TKN154.h"
#include "app_profile.h"
#include "UserButton.h"

module LineFollowingP @ safe() {

uses {
	interface Boot;
	interface Leds;
	//interface LocalTime<TSymbolIEEE802154> as LocalTime;
    interface Timer<TMilli> as TimerSamples;
	interface GeneralIO as PloadPin;
	interface GeneralIO as ClEnablePin;
	interface GeneralIO as ClockPin;
	interface GeneralIO as DataPin;
	interface GpioInterrupt as PinAFallingInt;
	interface BusyWait<TMicro,uint16_t>;
	interface Notify<button_state_t>;
	
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

}
implementation {	
	// variables to send the message
	message_t m_frame;	
	SensorValues sensorValues;
	uint8_t leng;
	uint8_t bytesVal;
	uint8_t bitVal;
	uint8_t i;
	uint8_t *payloadRegion;
	uint16_t ticks;
	task void sendPacket();
	ieee154_address_t deviceShortAddress;
#ifndef TKN154_BEACON_DISABLED
	bool m_wasScanSuccessful;
	void setAddressingFields(uint16_t address);
	ieee154_PANDescriptor_t m_PANDescriptor;
#endif

	event void Boot.booted() {
		call Notify.enable();	
		atomic ticks=0;	
		sensorValues.ticks = 0;	
		bytesVal = 0;
		call PloadPin.makeOutput();
		call ClEnablePin.makeOutput();
		call ClockPin.makeOutput();
		call DataPin.makeInput();		
		leng = sizeof(SensorValues);
		payloadRegion = call Packet.getPayload(&m_frame, leng);		
		call MLME_RESET.request(TRUE);			
	}
	
	async event void PinAFallingInt.fired(){
		atomic ticks++;	
	}
	
	event void TimerSamples.fired() {		
		call ClEnablePin.set();
		call PloadPin.clr();
		call BusyWait.wait(PULSE_WIDTH);
		call PloadPin.set();
		call ClEnablePin.clr();
		bytesVal = 0;
		call BusyWait.wait(PULSE_WIDTH);
		for(i = 0; i < DATA_WIDTH; i++)
    	{
        /* Pulse the Clock (rising edge shifts the next bit).
        */        
        bitVal = call DataPin.get();
        /* Set the corresponding bit in bytesVal.*/
        bytesVal |= (bitVal << ((DATA_WIDTH-1) - i)); 
        call ClockPin.set();
        call BusyWait.wait(PULSE_WIDTH);
        call ClockPin.clr();               
    	}
    	post sendPacket();
	}
	
	task void sendPacket() {
		sensorValues.lineVal = bytesVal;
		atomic sensorValues.ticks = ticks;	
    	memcpy(payloadRegion, &sensorValues, leng);	
  		if (call MCPS_DATA.request  (
          &m_frame,                         // frame,
          leng,                     // payloadLength,
          0,                                // msduHandle,
          TX_OPTIONS_ACK 					// TxOptions,
          ) != IEEE154_SUCCESS)
			call Leds.led0Toggle(); //fail!
		else call Leds.led2Toggle(); 
	}
	
	event void Notify.notify( button_state_t state ) {
		if ( state == BUTTON_PRESSED ) {
			post sendPacket();
			call Leds.led2On();
		} else if ( state == BUTTON_RELEASED ) {
			call Leds.led2Off();
		}
	}
	
	/*********************************************************************
	 * I E E E   8 0 2 . 1 5 . 4
	 *********************************************************************/
	void startApp()
	{
#ifndef TKN154_BEACON_DISABLED
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;
#endif
//		call Leds.led0Off();
//		call Leds.led1Off();
//		call Leds.led2Off();

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);
		call MLME_SET.macRxOnWhenIdle(TRUE);
		call MLME_SET.macMaxCSMABackoffs(M);
		call MLME_SET.macMinBE(M_0);
		call MLME_SET.macMaxBE(M_B);
		call MLME_SET.macMaxFrameRetries(N);
		
		deviceShortAddress.shortAddress = COORDINATOR_ADDRESS;
    	call Frame.setAddressingFields(
        	&m_frame,                
       		ADDR_MODE_SHORT_ADDRESS,        // SrcAddrMode,
        	ADDR_MODE_SHORT_ADDRESS,        // DstAddrMode,
        	PAN_ID,						    // DstPANId,
      		&deviceShortAddress, 			// DstAddr,
      		NULL                            // security
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
		//call UartResource.request();
		//call TimerSamples.startPeriodic(50);
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
			//call Leds.led2On();
			else
		//	call Leds.led2Off();
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
//			call Leds.led0Off();
//			call Leds.led1Off();
//			call Leds.led2On();
			// we received a beacon from the coordinator before
			call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
			call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
			call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);
			
		} else {
//			call Leds.led0On();
//			call Leds.led1On();
//			call Leds.led2Off();

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

	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	) {
	}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {		
		return frame;
	}
	/*********************************************************************
	 * E N D   I E E E   8 0 2 . 1 5 . 4
	 *********************************************************************/
}
