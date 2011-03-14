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

#include "TKN154.h"
#include "app_perform.h"
#include "app_sensors_nbe.h"
#include "math.h"

module ActuatorC {
	uses {
		interface Boot;
		interface Leds;

		interface Packet;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;
	}
}
implementation {

	bool m_wasScanSuccessful;

	uint8_t m_payloadLen = sizeof(SensingMsg);
	ieee154_PANDescriptor_t m_PANDescriptor;

	void setAddressingFields(uint16_t address);

	uint8_t i,j;

	/**----------------------------------------------*/
	event void Boot.booted() {

		atomic {
			ADC12CTL0 = REF2_5V + REFON;
			DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;
		}

		call MLME_RESET.request(TRUE);

		call Leds.led0On();
		call Leds.led1On();
		call Leds.led2On();
	}

	/*********************************************************************
	 * 802.15.4 functions
	 *********************************************************************/
	void startApp()
	{
		ieee154_phyChannelsSupported_t channelMask;

		call Leds.led0Off();
		call Leds.led1Off();
		call Leds.led2Off();

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(ACTUATOR_ADDRESS);
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

	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	) {}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame)
	{
		ieee154_address_t deviceAddr;
		SensingMsg *m_device;

		// We assume that the packets with length equal to EncMsg
		// are a EncMsg
		if (call Frame.getPayloadLength(frame) == m_payloadLen &&
				call Frame.getFrameType(frame) == FRAMETYPE_DATA) {

			call Frame.getSrcAddr(frame, &deviceAddr);
			if(deviceAddr.shortAddress == CONTROLLER_ADDRESS) {
				m_device = (SensingMsg*)(call Packet.getPayload(frame,m_payloadLen));

				atomic DAC12_0DAT = m_device->data;
				call Leds.led1Toggle();
			}

		}
		return frame;
	}
	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/

}
