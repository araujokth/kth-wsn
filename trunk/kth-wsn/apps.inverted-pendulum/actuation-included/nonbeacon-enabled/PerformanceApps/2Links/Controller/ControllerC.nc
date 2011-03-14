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
#include "TKN154_PHY.h"

#include "app_sensors_nbe.h"
#include "app_perform.h"

#include "UserButton.h"
#include "math.h"

module ControllerC {
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

		interface Timer<TMilli> as TimerTimeout;
		interface Timer<TMilli> as TimerBeacon;

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

	message_t m_frame;

	bool m_wasScanSuccessful;

	uint32_t beacontime;

	uint8_t m_payloadLen = sizeof(SensingMsg);
	SensingMsg *m_sensor;

	ieee154_PANDescriptor_t m_PANDescriptor;

	void setAddressingFields(uint16_t address);
	task void calcAndAct();

	// P E R F O R M A N C E   E V A L U A T I O N
	PerformanceParams sendPerfParams; // to compute the performance before sending
	uint32_t timestamp2Delay;

	/* LQG and Kalman filter */
	nx_float L[4], K[4][2], A_tau[4][4], A_htau[4][4], B_tau[4], B_htau[4], C[2][4];
	nx_float x[4], u;
	uint8_t i,j;
	uint32_t bi_bo4;

	bool got[2]; //got[0]:  xc_sensor ; got[1]: theta_sensor
	nx_int16_t data[2];

	event void TimerBeacon.fired() {

		post calcAndAct();

	}
	event void TimerTimeout.fired() {

		atomic {
			if ((got[1] && !got[0])||(!got[1] && got[0]))
			{
#ifndef TKN154_BEACON_DISABLED
				if((call TimerBeacon.getNow()) - beacontime < ( (((uint32_t) 1 << (uint32_t) ((call MLME_GET.macBeaconOrder())-4)) * bi_bo4)-40)) {
#endif
					//post calcAndAct();

					got[1] = FALSE;
					got[0] = FALSE;

					/*uartQueue[uartIn] = &m_frame;
					 uartIn = (uartIn + 1) % UART_QUEUE_LEN;

					 if (!uartBusy)
					 {
					 post uartSendTask();
					 uartBusy = TRUE;
					 }*/
#ifndef TKN154_BEACON_DISABLED
				}
#endif
			} else {
				//call Leds.led0Toggle();
			}
		}

	}

	/**----------------------------------------------*/
	event void Boot.booted() {

		bi_bo4 = 246;

		got[1] = FALSE;
		got[0] = FALSE;
		data[0] = 0;
		data[1] = 0;

		call MLME_RESET.request(TRUE);

		m_sensor = (SensingMsg*)(call Packet.getPayload(&m_frame,m_payloadLen));
		m_sensor->data = 0;

		sendPerfParams.pckTotal = 0;
		sendPerfParams.pckSuccess = 0;

		x[0]=0;
		x[1]=0;
		x[2]=0;
		x[3]=0;

		u=0;

		// LQ Gain
		L[0]=-27.9205;
		L[1]=133.5942;
		L[2]=-34.225;
		L[3]=19.522;

//		L[0]=-47.4323;
//		L[1]=199.3062;
//		L[2]=-50.3066;
//		L[3]=27.9151;
		
		// Wireless
		K[0][0]=1.0000;
		K[0][1]=0.0000;
		K[1][0]=0.0000;
		K[1][1]=1.0000;
		K[2][0]=0.0000;
		K[2][1]=0.0011;
		K[3][0]=0.0010;
		K[3][1]=0.5869;

		A_tau[0][0]=1.00;
		A_tau[0][1]=0.000847131340963129;
		A_tau[0][2]=0.0240625846979718;
		A_tau[0][3]=0.00000605644181647328;
		A_tau[1][0]=0.00;
		A_tau[1][1]=1.01213514891668;
		A_tau[1][2]=-0.0136946676192767;
		A_tau[1][3]=0.0300828963670533;
		A_tau[2][0]=0.00;
		A_tau[2][1]=0.0525780049495745;
		A_tau[2][2]=0.632064732925621;
		A_tau[2][3]=0.000678154224755660;
		A_tau[3][0]=0.00;
		A_tau[3][1]=0.801112383040290;
		A_tau[3][2]=-0.849971331178926;
		A_tau[3][3]=1.00955363381334;

		A_htau[0][0]=1.00;
		A_htau[0][1]=0.00000431194071106424;
		A_htau[0][2]=0.00196974549031381;
		A_htau[0][3]=-0.0000000109757913270413;
		A_htau[1][0]=0.00;
		A_htau[1][1]=1.00005513915775;
		A_htau[1][2]=-0.0000697066949740868;
		A_htau[1][3]=0.00199985911172103;
		A_htau[2][0]=0.00;
		A_htau[2][1]=0.00428999948243722;
		A_htau[2][2]=0.969898932861259;
		A_htau[2][3]=-0.00000947530087570435;
		A_htau[3][0]=0.00;
		A_htau[3][1]=0.0550876802739018;
		A_htau[3][2]=-0.0693519884267960;
		A_htau[3][3]=0.999877639643667;

		B_tau[0]=0.000851066179853329;
		B_tau[1]=0.00196301316566279;
		B_tau[2]=0.0527396598540434;
		B_tau[3]=0.121836225123766;

		B_htau[0]=0.00000433666647868565;
		B_htau[1]=0.00000999183985705511;
		B_htau[2]=0.00431467209951842;
		B_htau[3]=0.00994099639333767;

		C[0][0]=1;
		C[0][1]=0;
		C[0][2]=0;
		C[0][3]=0;
		C[1][0]=0;
		C[1][1]=1;
		C[1][2]=0;
		C[1][3]=0;

		call Leds.led0On();
		call Leds.led1On();
		call Leds.led2On();

	}

	task void calcAndAct() {

		nx_float y[2], x_bar[4], res_0[4];
		nx_float u_old;

		u_old = u;

		y[0] = ((nx_float) data[0]) * 9.2E-5;
		y[1] = (((nx_float) data[1]) / 1024.0) * 2 * 3.14159265;

		//x_bar= x_old_mote+ K*(y-C*x_old_mote);
		memset(x_bar, 0, sizeof(x_bar));
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<2;i++) for(j=0;j<4;j++) res_0[i] += C[i][j] * x[j];
		for(i=0;i<2;i++) res_0[i] = y[i] - res_0[i];
		for(i=0;i<4;i++) for(j=0;j<2;j++) x_bar[i] += K[i][j] * res_0[j];
		for(i=0;i<4;i++) x_bar[i] += x[i];

		//x_bar= A_tau*x_bar- B_tau*u_old;
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<4;i++) for(j=0;j<4;j++) res_0[i] += A_tau[i][j] * x_bar[j];
		for(i=0;i<4;i++) x_bar[i] = res_0[i] - B_tau[i] * u_old;

		//u_mote= L*x_bar;
		u = 0;
		for(i=0;i<4;i++) u += L[i] * x_bar[i];

		//x_mote= A_htau*x_bar- B_htau*u_old;
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<4;i++) for(j=0;j<4;j++) res_0[i] += A_htau[i][j] * x_bar[j];
		for(i=0;i<4;i++) x[i] = res_0[i] - B_htau[i] * u_old;

		if(u > 10) u = 10.0;
		if(u < -10) u = -10.0;
		u = u * 0.6; //That's LabVIEW's output!!

		m_sensor->data = (uint16_t) (((u + 6.0) / 4.8) * 1638.0);

		// set the timestamp and copy the new performance data
		memcpy(&(m_sensor->performValues), &sendPerfParams, sizeof(PerformanceParams) );
		//set the new timestamp value
		timestamp2Delay = call LocalTime.get();

		sendPerfParams.pckTotal ++;

		if (call MCPS_DATA.request (
						&m_frame, // frame,
						sizeof(SensingMsg), // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led2Toggle(); //fail!
		}
		else {
			call Leds.led1Toggle();
		}

	}

	/*********************************************************************
	 * 802.15.4 functions
	 *********************************************************************/
	void startApp()
	{
//		ieee154_phyChannelsSupported_t channelMask;
//		uint8_t scanDuration = BEACON_ORDER;
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = ACTUATOR_ADDRESS;

		call Leds.led0Off();
		call Leds.led1Off();
		call Leds.led2Off();

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);
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
		call Frame.setAddressingFields(
				&m_frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);
		call TimerBeacon.startPeriodicAt(0, 32);
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
		// reset the packet counters
		if (sendPerfParams.pckTotal >= SLIDING_WINDOW) {
			sendPerfParams.pckTotal = 0;
			sendPerfParams.pckSuccess = 0;
			return;
		}
		if (status == IEEE154_SUCCESS) {
			sendPerfParams.pckSuccess ++;
			call Leds.led1Toggle();
		} else {
			call Leds.led0Toggle();
		}

		sendPerfParams.delay = (uint16_t) timestamp - timestamp2Delay;

	}
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
			m_device = (SensingMsg*)(call Packet.getPayload(frame,m_payloadLen));
			data[deviceAddr.shortAddress - 1] = m_device->data;
			got[deviceAddr.shortAddress -1] = TRUE;

			call TimerTimeout.startOneShot(5);

			atomic {
				if (got[1] && got[0])
				{
#ifndef TKN154_BEACON_DISABLED
					if((call TimerBeacon.getNow()) - beacontime < ( (((uint32_t) 1 << (uint32_t) ((call MLME_GET.macBeaconOrder())-4)) * bi_bo4)-40)) {
#endif
						//post calcAndAct();

						got[1] = FALSE;
						got[0] = FALSE;

#ifndef TKN154_BEACON_DISABLED
					}
#endif
				}
				else {
					//call Leds.led0Toggle();
				}
			}
		}
		return frame;
	}
	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/

}