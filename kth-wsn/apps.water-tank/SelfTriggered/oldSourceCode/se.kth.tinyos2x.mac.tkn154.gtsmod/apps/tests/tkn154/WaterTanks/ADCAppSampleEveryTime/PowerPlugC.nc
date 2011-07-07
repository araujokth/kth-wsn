/*******************
 @PowerPlugC.nc
 ********************/
#include <Timer.h>
#include "PowerPlug.h"

#include "TKN154.h"
#include "app_profile.h"

module PowerPlugC
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
		
		//interface SuperframeStructure as SF;

		interface Get<ieee154_GTSentry_t*> as GtsCoordinatorDb;

		interface GtsUtility;
		interface GetNow<bool> as IsGtsOngoing;
		
		//interface Alarm<TSymbolIEEE802154,uint32_t> as CfpSlotAlarm;

	}
	provides interface AdcConfigure<const msp430adc12_channel_config_t*>;

}

implementation
{
	//adc channel configuration
	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	//define variations or constants  
	uint16_t *data_ptr;
	uint16_t buffer[2*BUFFER_SIZE];
	uint16_t Plug1[BUFFER_SIZE];
	uint16_t Plug2[BUFFER_SIZE];
	uint16_t Ts;
	uint8_t j;
	uint8_t i;

	nx_float dt;
	nx_float sampleTime;
	nx_float symbolTime;
	uint32_t beaconTime;
	uint32_t slotDuration;
	bool secondSample = FALSE;


	// vars for 802.15.4
	MyMsg* datapkt;
	message_t m_frame;
	uint8_t m_payloadLen = sizeof(MyMsg);
	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;
	bool OKtoRead;
	void startApp();
	
	
	nx_float beacon_interval_table[11];

	void task getData();
	task void sendMessage();
	
	nx_float x_int;

	//boot start
	event void Boot.booted()
	{

		datapkt =(MyMsg*)(call Packet.getPayload(&m_frame,sizeof(MyMsg)));
		datapkt->srcId = TOS_NODE_ID;
		
		x_int = 0;
		dt = 0;
		symbolTime = 0.000015259;
		
		beacon_interval_table[0] = 0.23438;
		beacon_interval_table[1] = 0.46875;
		beacon_interval_table[2] = 0.93750;
		beacon_interval_table[3] = 1.87500;
		beacon_interval_table[4] = 3.75000;
		beacon_interval_table[5] = 7.50000;
		beacon_interval_table[6] = 15.00000;
		beacon_interval_table[7] = 30.00000;
		beacon_interval_table[8] = 60.00000;
		beacon_interval_table[9] = 120.00000;
		beacon_interval_table[10] = 240.00000;

		//Ts=10000;
		Ts=482;
		OKtoRead = FALSE;
		call MLME_RESET.request(TRUE);

	}

	/******************************************************************************
	 @name:      sendMessage

	 @function:  send the nodeId and current value to receiver(base station)

	 @parameter: pointer to payload

	 @return:    none
	 *******************************************************************************/

	task void sendMessage() {
		int beaconOrder;
		if (!m_wasScanSuccessful) {
			return;
		}
		else {
		


			atomic {

				for (j=0;j<BUFFER_SIZE;j++) {
					datapkt->data[j]=Plug2[j];
					datapkt->data[j+BUFFER_SIZE]=Plug1[j];
				}
				
				
				//Use 2.0 seconds as a fixed interval for integration even though we are changing the beacon interval. 
				//x_int += beacon_interval_table[(beaconOrder - 4)]*((((nx_float) Plug1[0])/87.36) - 10.0);
				//x_int += 2.0*((((nx_float) Plug1[0])/87.36) - 10.0);
				

				datapkt->trgtId = INTERMEDIATE_NODEID;
				datapkt->other = FIRST_PLUG;
				datapkt->integrator = x_int;
			
			}

			if (call MCPS_DATA.request (
							&m_frame, // frame,
							m_payloadLen, // payloadLength,
							0, // msduHandle,
							TX_OPTIONS_ACK | TX_OPTIONS_GTS // TxOptions,
					) != IEEE154_SUCCESS)
					
			{
			//call Leds.led0Toggle(); //fail!
			}else{
			call Leds.led0Off();
			}
			
			i=0;

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
		call Resource.request();
	}
	event void Resource.granted()
	{
		atomic{
		adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}};
		if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) == SUCCESS) {
			OKtoRead = TRUE;
			i=0;
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
		ieee154_GTSentry_t* GTSentry;
		int beaconOrder = call MLME_GET.macBeaconOrder();
		uint32_t numCapSlots;
		uint32_t m_capDuration;

		/*if (secondSample) {
			dt = (symbolTime * ((nx_float) call CfpSlotAlarm.getNow()))-sampleTime;
		} else {
			sampleTime = symbolTime * ((nx_float) call CfpSlotAlarm.getNow());
		}*/



		data_ptr = buf;


		Plug1[i]=data_ptr[1];
		Plug2[i]=data_ptr[2];

		i=i+1;
		
		//call Leds.led0Toggle();
		//call Leds.led1Toggle();
		//call Leds.led2Toggle();
		
		/*if (secondSample) {
			x_int += (dt)*((((nx_float) Plug1[0])/87.36) - 10.0);
		} else {*/
			x_int += (beacon_interval_table[(beaconOrder - 4)]-dt)*((((nx_float) Plug1[0])/87.36) - 10.0);
		//}

		if(call IsGtsOngoing.getNow()) {
			//if (secondSample) {
				call Resource.release();
			//	secondSample = FALSE;
				post sendMessage();
		    //} else {
		    //	GTSentry = call GtsCoordinatorDb.get();

		    //	beaconTime = call SF.sfStartTime();
		    //	slotDuration = (uint32_t) call SF.sfSlotDuration();
		    //	numCapSlots = (uint32_t) call SF.numCapSlots();
		    //	m_capDuration = numCapSlots * slotDuration;

		    //	call CfpSlotAlarm.startAt(beaconTime, m_capDuration+(GTSentry->startingSlot - numCapSlots)*slotDuration);

		    //}
		} else {
			dt = 0;
			call Resource.release();
			ME2 &= ~(UTXE1 | URXE1);
			U1TCTL &= ~SSEL1;
		}
		
	}


	/*async event void CfpSlotAlarm.fired() {

		secondSample = TRUE;
		call MultiChannel.getData();

	}*/

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

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status == IEEE154_SUCCESS)
		startApp();
	}

	event message_t* MLME_BEACON_NOTIFY.indication (message_t* frame)
	{
		// received a beacon frame
		ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();
		ieee154_macBSN_t beaconSequenceNumber = call BeaconFrame.getBSN(frame);
		int beaconOrder = call MLME_GET.macBeaconOrder();

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
			
			//if (OKtoRead) {
				
				
			//if(call IsGtsOngoing.getNow()) { //Condition for sampling: check if we have a slot during this beacon interval
				//call MultiChannel.getData();

				post getData();
			//} else {
				//Use 2.0 seconds as a fixed interval for integration even though we are changing the beacon interval. 
				//x_int += beacon_interval_table[(beaconOrder - 4)]*((((nx_float) Plug1[0])/87.36) - 10.0);
				//x_int += 2.0*((((nx_float) Plug1[0])/87.36) - 10.0);
			//}
				
			//}
			
			// received a beacon during synchronization, toggle LED2
			//if (beaconSequenceNumber & 1)
			//call Leds.led2On();
			//else
			//call Leds.led2Off();
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
			
			call MLME_GTS.request(call GtsUtility.setGtsCharacteristics(1, GTS_TX_ONLY_REQUEST, GTS_ALLOCATE_REQUEST) ,NULL);

			//post getData();

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
			//call Leds.led1Toggle();
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
		
		nx_uint8_t * databuf;
		
		if (call Frame.getPayloadLength(frame) == m_payloadLen &&
				call Frame.getFrameType(frame) == FRAMETYPE_DATA) {
			MyMsg *receivepkt = (MyMsg*) (call Packet.getPayload(frame,m_payloadLen ));
			//databuf = (nx_uint8_t *) (call Packet.getPayload(frame,m_payloadLen ));

			if(receivepkt->srcId==BASE_NODEID) {

			/*call Leds.led0Off();
			call Leds.led1Off(); 
			call Leds.led2Off();
			if(databuf[0] == 0) call Leds.led0On();
			if(databuf[1] == 0) call Leds.led1On();
			if(databuf[2] == 0) call Leds.led2On();*/
			
				if(receivepkt->other==3 && receivepkt->trgtId==111) {
					//call Leds.led0Toggle();
					
				}
				else if(receivepkt->other==4 && receivepkt->trgtId==TOS_NODE_ID) {
					Ts=receivepkt->data[0];
				}
			}
		}
		return frame;
	}
	
	event void MLME_GTS.confirm (
			uint8_t GtsCharacteristics,
			ieee154_status_t status
	) {

	}

	event void MLME_GTS.indication (
			uint16_t DeviceAddress,
			uint8_t GtsCharacteristics,
			ieee154_security_t *security
	) {}
}//end of implementation
