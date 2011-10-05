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
 * @author David Andreu <daval@kth.se>*
 *  
 * @version  $Revision: 1.2 Date: 2011/09/28 $ 
 */
#include <Timer.h>
#include "TKN154.h"
#include "app_sensors.h"
#include "app_wt_calibration.h"
#include "AM.h"
#include "Serial.h"
#include "printf.h"
#include "PrintfUART.h"
#include <math.h>

module SensorC
{
	uses {
		interface Boot;

		interface Resource;
		interface Leds;

		interface Msp430Adc12MultiChannel as MultiChannel;

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

		
		interface GtsUtility;
		interface GetNow<bool> as IsGtsOngoing;

		interface LocalTime<TSymbolIEEE802154>;

	}
	provides interface AdcConfigure<const msp430adc12_channel_config_t*>;

}

implementation
{
	// ADC channel configuration
	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	// Define variations or constants  
	uint16_t buffer[3];	
//	uint16_t buf_old[3]; // buf_old and parameter alpha was used to set a low pass filter of the sensor readings
//	float alpha;

	// To control the phase of the experiment
	enum {
		PHASE_1 = 0x01, // In this phase it is implemented a Periodic Controller 
		PHASE_2 = 0x02, // Event Trigered Controller
		PHASE_3 = 0x03, // End of the experiment
	};

	// Matrix and variables definitions for Jose Event Triggered Controller
	float Ad[4], As[4], BdK[4], BdV[4], P[4], r_tk[2];
	float x0[2],x_ts[2],x_tk[2],x_pred[2],zs_ts[2],zs_tk[2],z_pred[2],zs_pred[2];

	float V_pred, S_pred;

	// vars for 802.15.4
	WTSensorValuesMsg* wtSensorValuesMsg;
	Control2MoteMsg control2MoteMsg;

	// Struct with all the controller information
	InitConfigureController CONTROLLER;

	uint8_t m_payloadLen;
	uint8_t m_payloadLenControl;

	message_t m_frame;

	ieee154_PANDescriptor_t m_PANDescriptor;
	
	bool m_wasScanSuccessful;

	
	void startApp();

	nx_float beacon_interval_table[15];

	void task getData();
	task void sendPacket();
	task void printfValues();

	// For the event triggered we need extra information	
	bool isTriggeredCondition();
	void multiplyMatrixs2x2X2x1(float matrix2x2[4], float matrix2x1[2],float matrixRes[2]);
	float multiplyMatrixs1x2X2x1(float matrix1x2[2], float matrix2x1[2]);

	uint8_t m_state;
	uint8_t init;

	uint8_t wtId;

	nx_float x_int;

	// To print
	uint32_t absoluteTime;
	uint32_t absoluteTimeCFPSuc;
	uint32_t intersamplingCFPSuc;
	uint32_t lastTimeSuc;

	uint32_t initialTimeOffset;
	uint32_t absoluteTimeAllocated;
	uint32_t intersamplingAllocated;
	uint32_t lastTimeAllocated;
	
	uint16_t numBeaconsPhase2;
	float iae;
	uint8_t numberSamples;
	

	// We set a maximum number of beacons without sampling
	uint8_t maxSampleIntervalcounter;
	enum {
		MAX_SAMPLE_INTERVAL = 15,
	};

	
	
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
	
	//boot start
	event void Boot.booted()
	{
		uint32_t i=0;
		uint32_t bi_symb;

		wtSensorValuesMsg = (WTSensorValuesMsg*)(call Packet.getPayload(&m_frame,m_payloadLen));
		m_payloadLen = sizeof(WTSensorValuesMsg);
		m_payloadLenControl = sizeof(Control2MoteMsg);

		

		wtId = (TOS_NODE_ID + 1)/2;

		x_int = 0;
		control2MoteMsg.ref = 0;

		atomic m_state = PHASE_1;
		init=TRUE;
		
//		alpha = 0.7;
//		buf_old[0]=0;
//		buf_old[1]=0;
//		buf_old[2]=0;

		// To print
		absoluteTime=0;
		initialTimeOffset=0;
		absoluteTimeCFPSuc=0;
		intersamplingCFPSuc=0;
		lastTimeSuc=0;
		absoluteTimeAllocated=0;
		lastTimeAllocated=0;
		
		numBeaconsPhase2=0;
		iae = 0;
		numberSamples=0;
		
		// We set a maximum number of beacons without sampling
		maxSampleIntervalcounter=0;
		
		for (i = 0; i < 15; i++) {
			bi_symb = (uint32_t) (1 << i) * IEEE154_aBaseSuperframeDuration;
			//			printf("BI [symb] = %lu * %lu = %lu bi", (uint32_t) ( 1 << i), (uint32_t) IEEE154_aBaseSuperframeDuration , (uint32_t) bi_symb );
			beacon_interval_table[i] = (nx_float) (bi_symb * 0.000015259);
			//			printfFloat(beacon_interval_table[i]);
			//			printf(" s = BI[%u] \n", i);
			//			printfflush();
		}

		

		
		initController(&CONTROLLER);
		for (i=0;i<4;i++) {
			Ad[i]=CONTROLLER.Ad[(TOS_NODE_ID-1)/2][i];
			As[i]=CONTROLLER.As[(TOS_NODE_ID-1)/2][i];
			BdK[i]=CONTROLLER.BdK[(TOS_NODE_ID-1)/2][i];
			BdV[i]=CONTROLLER.BdV[(TOS_NODE_ID-1)/2][i];
			P[i]=CONTROLLER.P[(TOS_NODE_ID-1)/2][i];
		}
		for (i=0;i<2;i++) {
			r_tk[i]=CONTROLLER.r_tk[(TOS_NODE_ID-1)/2][i];

		}

		call MLME_RESET.request(TRUE);

	}
	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status == IEEE154_SUCCESS)
		startApp();
	}

	/******************************************************************************
	 @name:      sendPacket

	 @function:  send the nodeId and current value to receiver(base station)

	 @parameter: pointer to payload

	 @return:    none
	 *******************************************************************************/
	task void printfValues()
	{
		if (m_state==PHASE_2) {
			printfFloat((wtSensorValuesMsg->tankLevel[0])/WT_CALIBRATION);
			printfFloat((wtSensorValuesMsg->tankLevel[1])/WT_CALIBRATION);
			//			printfFloat(V_ts);
			//			printfFloat((S_ts+1.5));
			printf(" %lu ",absoluteTime);
			printf("0 ");
			printf("0 ");
			printf("%lu ",absoluteTimeCFPSuc);
			printf("%lu ",intersamplingCFPSuc);
			printf("%lu ",absoluteTimeAllocated);
			printf("%lu ",intersamplingAllocated);
			printfFloat(iae);
			printf(" %u ",numberSamples);
			printf("\n");
			printfflush();
		}
	}

	task void sendPacket() {
		uint8_t debug;
		if (!m_wasScanSuccessful) {
			return;
		} else {
			wtSensorValuesMsg->isGTSpacket=TRUE;
			

			

			

			debug = call MCPS_DATA.request (
					&m_frame, // frame,
					m_payloadLen, // payloadLength,
					0, // msduHandle,
					TX_OPTIONS_GTS | TX_OPTIONS_ACK // TxOptions,
			);

			if( debug != IEEE154_SUCCESS) {
				if( debug == IEEE154_TRANSACTION_OVERFLOW) {
					call Leds.led0Toggle(); //fail!
				}
			} else
			call Leds.led0Off();
		}

	}

	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}

	/******************************************************************************
	 @name:      getData

	 @function:  request the data from ADC readings

	 @parameter: none

	 @return:    none
	 *******************************************************************************/
	void task getData()
	{
		if (!call Resource.isOwner()) {
			call Resource.request();
		}
		else {
			call MultiChannel.getData();
		}

	}
	event void Resource.granted()
	{
		atomic {
			adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}};
			if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) == SUCCESS) {
				call MultiChannel.getData();
			}
		}
	}

	/*************************************************************************************************

	 Description:
	 configureMultiple
	 command error_t configureMultiple(msp430adc12_channel_config_t *config, uint16_t *buffer, uint16_t numSamples, uint16_t jiffies)

	 Configures the ADC for sampling a channel numSamples times with a given sampling period. Any previous configuration will be overwritten. In contrast to the configureSingleRepeat() command, this configuration means that only one event will be signalled after all samples have been taken (which is useful for high-frequency sampling). If SUCCESS is returned calling getData() will start sampling the adc channel numSamples times and the first conversion is started immediately. Conversion results are stored in a buffer allocated by the client (the buffer parameter). The sampling period is specified by the jiffies parameter, which defines the time between successive conversions in terms of clock ticks of clock source "sampcon_ssel" and clock input divider "sampcon_id" as specified in the config parameter. If jiffies is zero successive conversions are performed as quickly as possible. After numSamples conversions an event multipleDataReady() is signalled with the conversion results.

	 Parameters:
	 config - ADC12 configuration data. 
	 jiffies - Sampling period in terms of clock ticks of "sampcon_ssel" and input divider "sampcon_id". 
	 buffer - The user-allocated buffer in which the conversion results will be stored. It must have at least numSamples entries, i.e. it must have a size of at least numSamples * 2 byte. 
	 numSamples - Number of adc samples 
	 Returns:
	 SUCCESS means that the ADC was configured successfully and getData() can be called to start with the first convers

	 *****************************************************************************************************/

	//when data is ready   
	async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{
		uint8_t error;
		uint8_t beaconOrder = call MLME_GET.macBeaconOrder();
		atomic {
			//			buf [1] = alpha*buf[1]+(1-alpha)*buf_old[1];
			//			buf [2] = alpha*buf[2]+(1-alpha)*buf_old[2];
			//			buf_old[1]=buf[1];
			//			buf_old[2]=buf[2];

			wtSensorValuesMsg->tankLevel[0] = buf[1];
			wtSensorValuesMsg->tankLevel[1] = buf[2];
			wtSensorValuesMsg->integrator = x_int; // We send integral[k]
			if (((wtSensorValuesMsg->tankLevel[1])/WT_CALIBRATION-control2MoteMsg.ref)>=0){
				iae = iae + ((wtSensorValuesMsg->tankLevel[1])/WT_CALIBRATION-control2MoteMsg.ref);
			}else{
				iae = iae - ((wtSensorValuesMsg->tankLevel[1])/WT_CALIBRATION-control2MoteMsg.ref);
			}
			

		}
		x_int += beacon_interval_table[ beaconOrder ]*((((nx_float) buf[2])/WT_CALIBRATION) - control2MoteMsg.ref);
		if (m_state==PHASE_1) {
			if (call IsGtsOngoing.getNow()) {
				call Leds.led1Toggle();

				post sendPacket();
			} else {
				//				ME2 &= ~(UTXE1 | URXE1);
				//				U1TCTL &= ~SSEL1;
			}
		} else if (m_state==PHASE_2) {

			if (init)
			{
				init=FALSE;
				x0[0]=buf[1]/WT_CALIBRATION;
				x0[1]=buf[2]/WT_CALIBRATION;

				zs_ts[0]=-r_tk[0];
				zs_ts[1]=-r_tk[1];
				z_pred[0]=-r_tk[0];
				z_pred[1]=-r_tk[1];
				x_tk[0]=0;
				x_tk[1]=0;
			}
			x_ts[0]=buf[1]/WT_CALIBRATION-x0[0];
			x_ts[1]=buf[2]/WT_CALIBRATION-x0[1];
			if (isTriggeredCondition()) {

			}
			post printfValues();
		}

	}

	/**************************************************************
	 * E V E N T - T R I G G E R E D
	 *************************************************************/
	bool isTriggeredCondition() {

		float tmp1[2],tmp2[2],tmp3[2];

		// Step 1
		multiplyMatrixs2x2X2x1(Ad,x_ts,tmp1);
		multiplyMatrixs2x2X2x1(BdK,x_tk,tmp2);
		multiplyMatrixs2x2X2x1(BdV,r_tk,tmp3);

		x_pred[0]=tmp1[0]+tmp2[0]+tmp3[0];
		x_pred[1]=tmp1[1]+tmp2[1]+tmp3[1];

		// Step 2
		multiplyMatrixs2x2X2x1(As,zs_ts,tmp1);

		zs_pred[0]=tmp1[0];
		zs_pred[1]=tmp1[1];

		// Step 3
		z_pred[0]=x_pred[0]-r_tk[0];
		z_pred[1]=x_pred[1]-r_tk[1];

		// Step 4
		multiplyMatrixs2x2X2x1(P,z_pred,tmp1);
		V_pred = multiplyMatrixs1x2X2x1(z_pred,tmp1);

		multiplyMatrixs2x2X2x1(P,zs_pred,tmp1);
		S_pred = multiplyMatrixs1x2X2x1(zs_pred,tmp1);

		// Step 5
		if (V_pred>(S_pred+1.5)) {
			
			if (call IsGtsOngoing.getNow()) {
				maxSampleIntervalcounter=0;
				post sendPacket();
				zs_tk[0]=x_ts[0]-r_tk[0];
				zs_tk[1]=x_ts[1]-r_tk[1];
				x_tk[0]=x_ts[0];
				x_tk[1]=x_ts[1];

				multiplyMatrixs2x2X2x1(As,zs_tk,zs_ts);
			}

			return TRUE;
		} else if ((maxSampleIntervalcounter>=MAX_SAMPLE_INTERVAL) && (call IsGtsOngoing.getNow())) {
			maxSampleIntervalcounter=0;
			post sendPacket();
			zs_tk[0]=x_ts[0]-r_tk[0];
			zs_tk[1]=x_ts[1]-r_tk[1];
			x_tk[0]=x_ts[0];
			x_tk[1]=x_ts[1];

			multiplyMatrixs2x2X2x1(As,zs_tk,zs_ts);

			return FALSE;

		} else {
			zs_ts[0]=zs_pred[0];
			zs_ts[1]=zs_pred[1];

			return FALSE;
		}

	}

	void multiplyMatrixs2x2X2x1(float matrix2x2[4], float matrix2x1[2],float matrixRes[2]) {
		matrixRes[0]=matrix2x2[0]*matrix2x1[0]+matrix2x2[1]*matrix2x1[1];
		matrixRes[1]=matrix2x2[2]*matrix2x1[0]+matrix2x2[3]*matrix2x1[1];
	}
	float multiplyMatrixs1x2X2x1(float matrix1x2[2], float matrix2x1[2]) {
		return (matrix1x2[0]*matrix2x1[0]+matrix1x2[1]*matrix2x1[1]);
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
	uint8_t getPendAddrSpecOffset(uint8_t *macPayloadField)
	{
		uint8_t gtsDescriptorCount = (macPayloadField[BEACON_INDEX_GTS_SPEC] & GTS_DESCRIPTOR_COUNT_MASK) >> GTS_DESCRIPTOR_COUNT_OFFSET;

		return BEACON_INDEX_GTS_SPEC + 1 + ((gtsDescriptorCount > 0) ? GTS_DIRECTION_FIELD_LENGTH + gtsDescriptorCount * GTS_LIST_MULTIPLY: 0);
	}
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
			memcpy(&control2MoteMsg,call BeaconFrame.getBeaconPayload(frame), m_payloadLenControl);

			if(control2MoteMsg.ref==10)
			{
				atomic m_state = PHASE_2;
				maxSampleIntervalcounter=maxSampleIntervalcounter+1;
				numBeaconsPhase2=numBeaconsPhase2+1;
				if (numBeaconsPhase2==200){
					iae = 0;
				}
					
				if (init) {
					initialTimeOffset=call LocalTime.get();
					absoluteTime=0;
					iae=0;
				}
				else {
					absoluteTime=(call LocalTime.get())-initialTimeOffset;
				}
				if (call IsGtsOngoing.getNow()) {
					absoluteTimeAllocated=absoluteTime;
					intersamplingAllocated=absoluteTimeAllocated-lastTimeAllocated;
					lastTimeAllocated=absoluteTimeAllocated;
				}
			} else if (control2MoteMsg.ref==20) {
				call Leds.led0Off();
				call Leds.led1Off();
				atomic m_state = PHASE_3;
			}

			// We sample every superframe to compute the integral
			post getData();

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

		
		if (status == IEEE154_SUCCESS) {
			
			if (m_state==PHASE_2) {
				absoluteTimeCFPSuc=(call LocalTime.get())-initialTimeOffset;
				intersamplingCFPSuc=absoluteTimeCFPSuc-lastTimeSuc;
				lastTimeSuc=absoluteTimeCFPSuc;
				numberSamples=numberSamples+1;
			}
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
		//		Control2MoteMsg * controlReceived;
		//
		//		call Leds.led2On();
		//		call Leds.led1On();
		//		call Leds.led0On();
		//
		//		// Check if the message is a Control2Mote message
		//		if (call Frame.getPayloadLength(frame) == m_payloadLenControl &&
		//				call Frame.getFrameType(frame) == FRAMETYPE_DATA) {
		//
		//			controlReceived = (Control2MoteMsg*)(call Packet.getPayload(frame,m_payloadLenControl));
		//			control2MoteMsg.ref = controlReceived->ref;
		//
		//		}
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
}//end of implementation
