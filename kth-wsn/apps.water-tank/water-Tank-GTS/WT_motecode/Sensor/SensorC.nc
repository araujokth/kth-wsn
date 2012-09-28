/*
 * Copyright (c) 2012, KTH Royal Institute of Technology
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
 * led1Toggle  GTS Request Send
 * led0Toggle  Data Send to Coordinator 
 * 
 */
/**
 * @author Anser Ahmed <ansera@kth.se>
 *  
 */

#include "TKN154.h"
#include "printf.h"
#include "app_profile.h"
#include "app_wt_calibration.h"

module SensorC
{
	uses {
		interface Boot;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_SET;
		interface MLME_GET;
		interface MLME_SCAN;
		interface MLME_SYNC;
		interface MLME_BEACON_NOTIFY;
		interface MLME_SYNC_LOSS;
		interface MLME_GTS;

		interface GtsUtility;
		interface IEEE154Frame as Frame;
		interface IEEE154BeaconFrame as BeaconFrame;
		interface Leds;
		interface Packet;
		interface Random;

		interface Notify<bool> as IsEndSuperframe;
		interface GetNow<bool> as IsGtsOngoing;
	
		interface Resource;
		interface Msp430Adc12MultiChannel as MultiChannel;

	}
	provides interface AdcConfigure<const msp430adc12_channel_config_t*>;
}implementation {

	message_t m_frame;
	message_t m_framegts;
	uint8_t m_payloadLen;
    uint16_t buffer[3];
	uint8_t m_payload;
	uint16_t i_p;
	uint16_t counter;
	
	float x1_p;
	float x2_p;
	

	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;

	SensorValues* datapkt;//SensorValues sensorValues;
	uint8_t m_payloadLen = sizeof(SensorValues);
	
    GTSrequest* gtspkt; 
	
	void startApp();
	task void packetSendTask();
	task void GTS_Request();
	void printfFloat(float toBePrinted);
	task void printvalues();
	
	
	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};
	/**  *********  **/
	/*********************************************************************
	 *            Boot 
	 **********************************************************************/

	 event void Boot.booted() {
 
		 datapkt =(SensorValues*)(call Packet.getPayload(&m_frame,sizeof(SensorValues)));
		 gtspkt =(GTSrequest*)(call Packet.getPayload(&m_framegts,sizeof(GTSrequest)));
		 gtspkt->Id=TOS_NODE_ID;
 
		 datapkt->srcId = TOS_NODE_ID;
		 datapkt->trgtId = COORDINATOR_ADDRESS;
		 datapkt->other = 4;
		 counter=0;
		
		 atomic x1_p = 0;
		 atomic x2_p = 0;
		 call MLME_RESET.request(TRUE);
		 printf (" -------------------Sensor booted-------------------------  \n");
		 printfflush();
 
	 }

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status == IEEE154_SUCCESS)
		{
			startApp();
			call Resource.request();
		}
	}
    /*********************************************************************
	 *                           Scanning for beacons
	 **********************************************************************/
	void startApp()
	{
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

	/*********************************************************************
	 *            Beacon Scanning
	 **********************************************************************/ 
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

			// Transmit a packet if we have a slot allocated for us
			printf("\n------------------------------------------------------------------\n");
			printf(" BSN  %u   ",beaconSequenceNumber);
			printfflush();

			//received a beacon during synchronization, toggle LED2
			if (beaconSequenceNumber & 1)
				call Leds.led2On();
			else
				call Leds.led2Off();
	
			counter++;
			if(counter==4)
			{printf(" GTS_Request \n");printfflush();
				gtspkt->gtspayload=1;
				post GTS_Request();}
	
		
	
	
			if (call IsGtsOngoing.getNow()){
				i_p++;
				//printf(" IsGtsOngoing \n");printfflush();
	
				call MultiChannel.getData();
				post packetSendTask(); ///
				post printvalues();
			}
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
		
			call Frame.setAddressingFields(
					&m_framegts,
					ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
					ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
					m_PANDescriptor.CoordPANId, // DstPANId,
					&m_PANDescriptor.CoordAddress, // DstAddr,
					NULL // security
			);
		} else
			startApp();
	}

	/*********************************************************************
	 *            packetSendTask
	 **********************************************************************/
	task void packetSendTask()
	{
		if (!m_wasScanSuccessful) {
	
			return;
		} else if( call MCPS_DATA.request (
							&m_frame, // frame,
						m_payloadLen, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_GTS | TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led0Off(); //fail!
		} else
			call Leds.led0Toggle(); 

	}

	event void MLME_SYNC_LOSS.indication(
			ieee154_status_t lossReason,
			uint16_t PANId,
			uint8_t LogicalChannel,
			uint8_t ChannelPage,
			ieee154_security_t *security)
	{
		m_wasScanSuccessful = FALSE;
		call Leds.led0Off();
		call Leds.led1Off();
		call Leds.led2Off();
		startApp();

	}

	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp )
	{	
		//if (status == IEEE154_SUCCESS)
			//call Leds.led1Toggle();
	}
	/*****************************************************************************************
	 *                  Data Received from Radio
	 *****************************************************************************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {
	
		return frame;}

	event void IsEndSuperframe.notify( bool val ) {}

	/*****************************************************************************************
	 * G T S   F U N C T I O N S 
	 *****************************************************************************************/
	event void MLME_GTS.indication (
			uint16_t DeviceAddress,
			uint8_t GtsCharacteristics,
			ieee154_security_t *security
	) {}

	event void MLME_GTS.confirm (
			uint8_t GtsCharacteristics,
			ieee154_status_t status
	) {}

    /*****************************************************
	 *               Reading Data From Sensor
	 *****************************************************/


	async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{
		// Copy sensor readings
		atomic{
	
			datapkt->data[0] = buf[1]; //buf[1]  data[0] Lower tank
			datapkt->data[1] = buf[2];  //buf[2] data[1] Upper Tank tank
			datapkt->data[2] = i_p;//humidity;
			datapkt->trgtId = COORDINATOR_ADDRESS;
			datapkt->other = 4;
            x1_p = (buf[1])/(WT_CALIBRATION);  //buf[1]  data[0] Lower tank
		    x2_p = (buf[2])/(WT_CALIBRATION);  //buf[2]  data[1] Upper Tank tank
		}
		

	}
	//when readtemp request has been processed
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}

	/***** Finish Reading Data from Sensors **************/
  
  
    /*********************************************************************
	 *                            Resource
	 **********************************************************************/
  	event void Resource.granted()
	  {
	
		  atomic {
			  adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}};

			  if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) != SUCCESS) {
				  call Leds.led0On();
			  }
		  }
	
	  }
	/*****************************************************************************************
	 * GTS_Request  
	 *****************************************************************************************/
	task void GTS_Request() {

	//	gtspkt->gtspayload=call GtsUtility.setGtsCharacteristics(1, GTS_TX_ONLY_REQUEST, GTS_ALLOCATE_REQUEST);
		
		if( call MCPS_DATA.request (
							&m_framegts, // frame,
						sizeof(GTSrequest), // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led1Off(); //fail!
		} else
		{call Leds.led1Toggle();}
		
		printf("GTS_Request send \n ");
		printfflush();

	}
	/*********************************************************************
	 *                  Timmer for Printing Tank Values
	 **********************************************************************/
	 task void printvalues() {
		
		// Sensor prints out sensor values (x1,x2) 
		printfFloat (x1_p);
		printfFloat (x2_p);
		printf(" local counter %u",i_p);
		printf("  \n");
		printfflush();
	}
    /*********************************************************************
	 * ADDITIONAL TOOLS (For printing out float values)
	 **********************************************************************/
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
