/*******************
 @PowerPlugC.nc
 ********************/
#include <Timer.h>
#include "printf.h"
#include "PowerPlug.h"

#include "TKN154.h"
#include "app_profile.h"

module PowerPlugC
{
	uses {
		interface Boot;

		interface Leds;


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
	}

}

implementation
{

	//define variations or constants  
	uint16_t *data_ptr;
	uint16_t buffer[2*BUFFER_SIZE];
	uint16_t Plug1[BUFFER_SIZE];
	uint16_t Plug2[BUFFER_SIZE];
	uint16_t Ts;
	uint8_t j;
	uint8_t i;

	// vars for 802.15.4
	MyMsg* datapkt;
	message_t m_frame;
	uint8_t m_payloadLen = sizeof(MyMsg);
	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;
	void startApp();


	//boot start
	event void Boot.booted()
	{

		atomic {
    	ADC12CTL0 = REF2_5V +REFON;
    	DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;
		}

		call MLME_RESET.request(TRUE);
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
			
			call MLME_GTS.request(call GtsUtility.setGtsCharacteristics(1, GTS_RX_ONLY_REQUEST, GTS_ALLOCATE_REQUEST) ,NULL);
			
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
			call Leds.led1Toggle();
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
			
				if(receivepkt->other==5 && receivepkt->trgtId==TOS_NODE_ID) {
					atomic {
						DAC12_0DAT = (receivepkt->data[0]);
						call Leds.led0Toggle();
					}
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
