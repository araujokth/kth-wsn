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
 * 
 * led1Toggle  GTS Request Send
 * led0Toggle  Data Received from Coordinator 
 */
/**
 * @author Anser Ahmed <ansera@kth.se>
 * 
 */

#include "TKN154.h"
#include "printf.h"
#include "app_profile.h"

module ActuatorC
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
	
		interface GeneralIO as ADC0;
		interface GeneralIO as ADC1;

	}
}implementation {

	uint16_t i_p;
	float u_p;
	message_t m_frame;
	message_t m_framegts;
	uint8_t m_payloadLen;
	uint16_t counter;
	uint16_t counter2;

	uint8_t m_payload;

	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;

	ToActuator* receivepkt;
	uint8_t m_payloadLenAct = sizeof(ToActuator);
	
    GTSrequest* gtspkt; 
	
	void startApp();
	task void GTS_Request();
	void printfFloat(float toBePrinted);
	
	
	/*********************************************************************
	 *            Boot 
	 **********************************************************************/

	 event void Boot.booted() {

		 atomic {
			 ADC12CTL0 = REF2_5V +REFON;
			 DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;
		 }

		 i_p = 0;
		 u_p = 0;
		 counter=0;
		 counter2=0;
		 //To not disturb the Sensor Values
		 call ADC0.makeInput();
		 call ADC1.makeInput();
		 DAC12_0DAT = 0;
 
		 gtspkt =(GTSrequest*)(call Packet.getPayload(&m_framegts,sizeof(GTSrequest)));
		 gtspkt->Id=TOS_NODE_ID;	

		 printf ("-------------------Actuator booted ----------------------- \n");
		 printfflush();
		 call MLME_RESET.request(TRUE);

	 }

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status == IEEE154_SUCCESS)
		startApp();
	}

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

			//received a beacon during synchronization, toggle LED2
			if (beaconSequenceNumber & 1)
				call Leds.led2On();
			else
				call Leds.led2Off();
			counter2++;
			printf("\n------------------------------------------------------------------\n");
			printf(" BSN % u    ",beaconSequenceNumber);
			printfflush();
 
            /**			if(counter2==10)
			{printf(" GTS_Request \n");printfflush();
				post GTS_Request();}  **/		
	
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
	//	if (status == IEEE154_SUCCESS)
	//	call Leds.led1Toggle();
	}
     /*****************************************************************************************
	 *                  Data Received from Radio
	 *****************************************************************************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {

 
		if (  call Frame.getPayloadLength(frame) == sizeof(ToActuator) && call Frame.getFrameType(frame) == FRAMETYPE_DATA) {
 
			// Received a packet from the Controller
			receivepkt = (ToActuator*) (call Packet.getPayload(frame,m_payloadLenAct));
			call Leds.led0Toggle();
 
			atomic {
				counter++;
				DAC12_0DAT = receivepkt->volt;
				u_p = ((float)(receivepkt->volt))/273; //          /273
			}
			printf("  voltage=");
			printfFloat (u_p);
			printf("  val = %u    id %u  count %u \n",receivepkt->volt,receivepkt->trgId,counter);
			printfflush();

		}	
		return frame;
	}

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


  /*****************************************************************************************
	 * GTS_Request  
	 *****************************************************************************************/
	task void GTS_Request() {

		gtspkt->gtspayload=call GtsUtility.setGtsCharacteristics(1, GTS_RX_ONLY_REQUEST, GTS_ALLOCATE_REQUEST);
		if( call MCPS_DATA.request (
							&m_framegts, // frame,
						sizeof(GTSrequest), // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led0Off(); //fail!
		} else
		{call Leds.led1Toggle();}
		
		printf("GTS_Request send \n ");
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
