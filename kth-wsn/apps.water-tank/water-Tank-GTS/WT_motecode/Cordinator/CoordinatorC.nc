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
 */
/**
 * @author Anser Ahmed<ansera@kth.se>
 * @modified 2012/04/13
 */

#include "TKN154.h"
#include "app_profile.h"
#include "UserButton.h"
module CoordinatorC
{
	uses {
		interface Boot;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface MLME_GTS;

		interface IEEE154Frame as Frame;
		interface IEEE154TxBeaconPayload;
		interface Leds;

		interface Packet;
		interface Timer<TMilli> as SendTimer;
	
		interface SplitControl as SerialControl;
		interface AMSend as UartSend;
		interface Receive as UartReceive;
		interface AMPacket as UartAMPacket;

		interface Get<ieee154_GTSdb_t*> as SetGtsCoordinatorDb;
		interface GtsUtility;
	
		interface Notify<button_state_t> as UserButton;

		interface Notify<bool> as IsEndSuperframe;
		interface GetNow<bool> as IsGtsOngoing;
	}provides {
		interface Notify<bool> as GtsSpecUpdated;
	}
}implementation {
	ieee154_GTSdb_t* GTSdb;

	bool m_ledCount;
	uint8_t x;
	uint8_t gtscount;
	uint8_t SensorMSGcount;         // Count msg received drom sensors
     
    uint8_t lock_frmSF;

	message_t m_frame;
	message_t m_frameToAct[ACTUATOR];  //message to Actuator
	message_t frametoSF;               //message to SF
	
	uint8_t m_payloadLen;
	uint16_t count1;
	uint16_t count2;
	uint16_t count3;
	bool startSend;
	uint8_t switchDescriptor;    //Switch between the desired GTS descriptor
	uint8_t j; 					//used for loops(For)
	uint8_t Sen[8]; 		//Shedular value Received from MathScript for Sensor
	uint8_t Act[ACTUATOR];      //Shedular value Received from MathScript for Actuator  
	uint8_t SenToSF[SENSOR];   //Shedular value send to SF
	
	void masking(uint8_t Sen_sheduling,uint8_t Act_sheduling);
	void setDefaultGtsDescriptor();
	void updateGtsDescriptor();
	void printfFloat(float toBePrinted);
	void setAddressingFields(uint16_t address, message_t * frame);
	void packetSendTask(message_t * frame  );
 

	SensorValues* receivepkt_S1; // Rx data from Sensors
	uint8_t m_payloadLenSensor = sizeof(SensorValues);
	
	SFtoController* SF_to_Controller;// Send data from SF to Controller
	uint8_t m_payloadLenSFtoController = sizeof(SFtoController);
	
	ControllertoSF* data_to_SF;   // Send data from Controller to SF
	uint8_t m_payloadLentoSF = sizeof(ControllertoSF);
 
	ToActuator* data_to_Act[ACTUATOR];    // Send data from Controller to Actuator A
	uint8_t m_payloadLenActuator = sizeof(ToActuator);
 
	GTSrequest* gts_req_pkt; // Rx data from Sensors
	uint8_t m_payloadLengts = sizeof(GTSrequest);
 

	
	/*********************************************************************
	 *                             Boot 
	 **********************************************************************/
	event void Boot.booted() {
	     uint8_t tempIndex=0;
		data_to_SF =(ControllertoSF*)(call Packet.getPayload(&frametoSF,sizeof(ControllertoSF)));
	
		for (j=0;j<(ACTUATOR);j++)
		{
			data_to_Act[j] =(ToActuator*)(call Packet.getPayload(&m_frameToAct[j],sizeof(ToActuator)));
			data_to_Act[j]->trgId=j+1;  //mote ID 
        }
	
		for(j=0;j<SENSOR;j++)
		{ 
			data_to_SF->Sensor[j]=0;	
			data_to_SF->sensorId[j]=j+SENSOR_ADDRESS1;
		    data_to_SF->dataWT[tempIndex]=0; 
			data_to_SF->dataWT[tempIndex+1]=0;
			tempIndex=tempIndex+2;
		}	

		count1=0;
		count2=0;
		count3=0;
		x=0; gtscount=0;SensorMSGcount=0;
		call UserButton.enable();
		startSend = FALSE;
		switchDescriptor = FALSE;
	}
	
	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status != IEEE154_SUCCESS)
			return;
		call MLME_SET.phyTransmitPower(TX_POWER_BEACON);
		call MLME_SET.macShortAddress(COORDINATOR_ADDRESS);
		call MLME_SET.macAssociationPermit(FALSE);
		call MLME_SET.macGTSPermit(TRUE);

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
	}
	
	event void MLME_START.confirm(ieee154_status_t status) {
		//setDefaultGtsDescriptor();

	}
	event void SendTimer.fired() {startSend = TRUE;}

	/*****************************************************************************************
	 *                Data Received on Radio
	 *****************************************************************************************/
	event message_t* MCPS_DATA.indication ( message_t* frame )
	{
		uint8_t binaryweight=1;
		uint8_t moteID=0;
        data_to_SF->lock=0; 		
 
 		if (call Frame.getPayloadLength(frame) == m_payloadLenSensor &&
					call Frame.getFrameType(frame) == FRAMETYPE_DATA) {
			receivepkt_S1 = (SensorValues*) (call Packet.getPayload(frame,m_payloadLenSensor));		  
 
			for(j=0;j<SENSOR*2;j=j+2)
			{ 
				if( (nx_uint8_t)(receivepkt_S1->srcId)==(moteID+SENSOR_ADDRESS1))
				{
					count1++;
					data_to_SF->sensorId[moteID]=moteID+SENSOR_ADDRESS1;
					data_to_SF->dataWT[j]=receivepkt_S1->data[0]; 
					data_to_SF->dataWT[j+1]=receivepkt_S1->data[1];//count1;//receivepkt_S1->count1; 
 
					//data_to_SF->dataWTa[2]=receivepkt_S1->data[2];
	
					//printf ("Sensor Id %i  data1 %i  data2 %i  data3 %i  \n",data_to_SF->sensorId[moteID],data_to_SF->dataWT[j],data_to_SF->dataWT[j],receivepkt_S1->data[2]);
					//printfflush();
					SensorMSGcount++;
	
				}  
				moteID++;
			}
			count3++;
		}
		// GTS request received from sensor
		if (call Frame.getPayloadLength(frame) == m_payloadLengts &&
					call Frame.getFrameType(frame) == FRAMETYPE_DATA ) {
			gts_req_pkt = (GTSrequest*) (call Packet.getPayload(frame,m_payloadLengts));
 
			for(j=0;j<SENSOR;j++)
			{ 
				if((nx_uint8_t)(gts_req_pkt->Id)==(j+9))
				{
				    	
				    data_to_SF->lock=1;              
					data_to_SF->Sensor[j]=gts_req_pkt->gtspayload;
					if((gts_req_pkt->gtspayload)==0)
					gtscount=gtscount-1;
					else
					{gtscount++;}
					
				}
			}
			count3++;
		}
 
		//    printf(" SensorMSGcount %u  \n",SensorMSGcount);printfflush();
		// Sending data to Serial Forward
		if((SensorMSGcount==0)|| (SensorMSGcount==gtscount)){
			//	printf(" gtscount %u  \n",gtscount);printfflush();
			if (call UartSend.send(AM_BROADCAST_ADDR, &frametoSF,m_payloadLentoSF) == SUCCESS) {
				call Leds.led0Toggle();
			} else {
				x=3;
			}
		}
	
		return frame;
	}


	/************************************************************************************
	 *  PC -----MYMsg-----> INTERMEDIATE NODE ---- IEE802.15.4---> Actuator
	 ************************************************************************************/
	event message_t *UartReceive.receive(message_t *msg,
			void *payload,
			uint8_t len) {
 
		SF_to_Controller = (SFtoController*) (call Packet.getPayload(msg,sizeof(SFtoController)));
		for (j=0;j<(ACTUATOR);j++)
		{
			data_to_Act[j]->counter=count3;  // counter 
			data_to_Act[j]->volt=SF_to_Controller->volt[j]; 
		}

		masking(SF_to_Controller->Sensor,SF_to_Controller->Actuator);
	
		return msg;
	}
	/*****************************************************************************************
	 *                 Beacons 
	 *****************************************************************************************/

	
	event void IEEE154TxBeaconPayload.aboutToTransmit() {}

	event void IEEE154TxBeaconPayload.setBeaconPayloadDone(void *beaconPayload, uint8_t length) {}

	event void IEEE154TxBeaconPayload.modifyBeaconPayloadDone(uint8_t offset, void *buffer, uint8_t bufferLength) {}

	event void IEEE154TxBeaconPayload.beaconTransmitted()
	{
		ieee154_macBSN_t beaconSequenceNumber = call MLME_GET.macBSN();
 
		data_to_SF->BSN=beaconSequenceNumber;
		if (beaconSequenceNumber & 1){
			call Leds.led2On();}
		else
		{	call Leds.led2Off();}
 
		GTSdb = call SetGtsCoordinatorDb.get();
		signal GtsSpecUpdated.notify(TRUE);
		if(x>0)		
		{	for(j=1;j<=ACTUATOR;j++)   
			{
				if(call GtsUtility.getGtsEntryIndex(GTSdb,j ,GTS_RX_ONLY_REQUEST )<32)
				{
					call Leds.led1Toggle();
					setAddressingFields(j,&m_frameToAct[j-1]);
					packetSendTask(&m_frameToAct[j-1]);
					
				}
			}
		}
		x++;
	
	}

	/*****************************************************************************************
	 *                           G T S   F U N C T I O N S 
	 *****************************************************************************************/
	
	void updateGtsDescriptor() {
		uint8_t i;
		
		ieee154_GTSdb_t* GTSdb;
		GTSdb = call SetGtsCoordinatorDb.get();
		GTSdb->numGtsSlots = 0;
		i=1;
		gtscount=0;
		for(j=0;j<SENSOR;j++)
		{
			if(Sen[j]==1)
			{call GtsUtility.addGtsEntry(GTSdb, 9+j, IEEE154_aNumSuperframeSlots - i , 1, GTS_TX_ONLY_REQUEST);
				i++;
				gtscount++;
				}
		}
	
		for(j=0;j<ACTUATOR;j++)
		{
			if(Act[ACTUATOR-1-j]==1)
			{call GtsUtility.addGtsEntry(GTSdb, ACTUATOR-j,IEEE154_aNumSuperframeSlots-i , 1, GTS_RX_ONLY_REQUEST);
				i++;}
		}	
		signal GtsSpecUpdated.notify(TRUE);
	
	}
	
	void setDefaultGtsDescriptor() {
		uint8_t i;
		ieee154_GTSdb_t* GTSdb;
		GTSdb = call SetGtsCoordinatorDb.get();
		GTSdb->numGtsSlots = 0;
		i=0;
		call GtsUtility.addGtsEntry(GTSdb, SENSOR_ADDRESS1, IEEE154_aNumSuperframeSlots - 1 , 1, GTS_TX_ONLY_REQUEST);
		call GtsUtility.addGtsEntry(GTSdb, SENSOR_ADDRESS2, IEEE154_aNumSuperframeSlots -2, 1, GTS_TX_ONLY_REQUEST);
		for(j=0;j<ACTUATOR;j++)
		{call GtsUtility.addGtsEntry(GTSdb, ACTUATOR-j,IEEE154_aNumSuperframeSlots- 3-j , 1, GTS_RX_ONLY_REQUEST);
		}	
		signal GtsSpecUpdated.notify(TRUE);
	}

	event void MLME_GTS.confirm (
			uint8_t GtsCharacteristics,
			ieee154_status_t status) {}

	event void MLME_GTS.indication (
			uint16_t DeviceAddress,
			uint8_t GtsCharacteristics,
			ieee154_security_t *security) {}

	/***************************************************************************************
	 *                          Superframe events()
	 ***************************************************************************************/
	event void IsEndSuperframe.notify( bool val ) {
		SensorMSGcount=0;
		if(Sen[7]==1) 
		{updateGtsDescriptor();}

	}

	/***************************************************************************************
	 *                         DEFAULTS spec updated commands
	 ***************************************************************************************/
	command error_t GtsSpecUpdated.enable() {return FAIL;}
	command error_t GtsSpecUpdated.disable() {return FAIL;}
	default event void GtsSpecUpdated.notify( bool val ) {return;}
	/*********************************************************************
	 *                        DEFAULTS  commands
	 **********************************************************************/

	event void SerialControl.startDone(error_t error) {}

	event void SerialControl.stopDone(error_t error) {}

	event void UartSend.sendDone(message_t* msg, error_t error) {}

	/*********************************************************************
	 * 7)    packetSendTask
	 *********************************************************************/
	void packetSendTask(message_t *frame)
	{
		if( call MCPS_DATA.request (
							frame, // frame,
						m_payloadLenActuator, // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_GTS | TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS)
			call Leds.led1Off(); //fail!
		else	
			call Leds.led1Toggle();
	}
	
	event void MCPS_DATA.confirm(
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t Timestamp
	) {}
	/*********************************************************************
	 *                       User Button
	 *********************************************************************/
	event void UserButton.notify( button_state_t state ) {
	
		if ( state == BUTTON_PRESSED ) {

		} else if ( state == BUTTON_RELEASED ) {
			call MLME_RESET.request(TRUE);
			call SerialControl.start();
	
		}
	
	}
	
	/*********************************************************************
	 *      Set Adressing Field
	 **********************************************************************/

	void setAddressingFields(uint16_t address, message_t *frame)
	{
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = address; // destination

		call Frame.setAddressingFields(
				frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);
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
 
	/*********************************************************************
	 * masking. call when we received shedular data from  LavVIEW
	 **********************************************************************/
	void masking(uint8_t Sen_sheduling,uint8_t Act_sheduling){
		uint8_t mask;
	
		mask=1;
		for (j=0;j<(8);j++)
		{
			Sen[j]=((mask)&Sen_sheduling)>>j;
			data_to_SF->Sensor[j]=Sen[j];
			mask=mask*2;
		}	
		mask=1;
		for (j=0;j<(ACTUATOR);j++)
		{
			Act[j]=((mask)&Act_sheduling)>>j;
			mask=mask*2;
		}
	
	}
 
}
