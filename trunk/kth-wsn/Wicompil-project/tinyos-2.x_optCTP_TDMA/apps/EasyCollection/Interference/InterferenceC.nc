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
 * @version  $Revision: 1.0 Date: 2010/06/05 $ 
 * @modified 2010/11/29 
 */

#include "TKN154.h"
#include "app_profile.h"
#include "printf.h"
#include "UserButton.h"

module InterferenceC
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
    interface TimeCalc as TC;
    interface SuperframeStructure as SF;
    
    interface Notify<bool> as IsEndSuperframe;
    interface GetNow<bool> as IsGtsOngoing;
    interface Alarm<TSymbolIEEE802154,uint32_t> as CapOverAlarm;

    interface Notify<button_state_t> as UserButton;
    /* interface Timer<TMilli> as TimerInterf; */

    /* interface RadioTx as PhyTx; */
    /* interface Packet as RadioPacket; */
    interface Resource;
    interface CarrierTx;
  }
}implementation {
  message_t m_frame;
  uint8_t m_payloadLen;
  uint8_t m_payload;
  uint16_t count=0;
  uint32_t m_CapDuration;
  uint32_t Gtime;
  uint32_t SFStartTime;
	
  ieee154_PANDescriptor_t m_PANDescriptor, m_PANDescriptor_ext;
  bool m_ledCount;
  bool m_wasScanSuccessful;

  bool InterfEnabled=FALSE;
  bool CapOver=TRUE;
  
  
  void startApp();
  task void packetSendTask_CAP();
  task void packetSendTask_CFP();
  void parseBeacon(uint8_t* payload);
  void parseGTSDescriptor(uint8_t* payload, uint8_t m_numGtsSlots);

  event void Boot.booted() {
    TDMA_frame_msg* tdma_msg = (TDMA_frame_msg*)call Packet.getPayload(&m_frame, sizeof(TDMA_frame_msg));
    tdma_msg -> options = 0;
    tdma_msg -> thl = 0;
    tdma_msg -> etx = 20;
    tdma_msg -> origin = TOS_NODE_ID;
    tdma_msg -> originSeqNo = count++;
    tdma_msg ->  type = 0;
    tdma_msg -> Node_ID = TOS_NODE_ID;  
    tdma_msg -> val = count;
    
    m_payloadLen = sizeof(TDMA_frame_msg);

    /*		char payload[] = "Hello Coordinator";
		uint8_t *payloadRegion;
		m_payloadLen = sizeof(payload);
		payloadRegion = call Packet.getPayload(&m_frame, m_payloadLen);*/
		
    if (m_payloadLen <= call Packet.maxPayloadLength()) {
      //memcpy(payloadRegion, payload, m_payloadLen);
      call MLME_RESET.request(TRUE);
    }
    m_payload=0;
    call UserButton.enable();
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
    call MLME_SET.macCoordShortAddress(COORDINATOR_ADDRESS);

    /* printf("Starting \n"); */
    /* printfflush(); */
    
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
  
  event void MLME_BEACON_NOTIFY.myTimeSlot (uint8_t ts){}
  
  event message_t* MLME_BEACON_NOTIFY.indication (message_t* frame)
  {
    // received a beacon frame
    ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();
    ieee154_macBSN_t beaconSequenceNumber = call BeaconFrame.getBSN(frame);
    uint8_t *payload = call Frame.getPayload(frame);
    
    atomic{ CapOver = FALSE; }

    /* printf("m_PANDescriptor.CoordAddress.shortAddress %i \n",m_PANDescriptor.CoordAddress.shortAddress); */
    /* printfflush(); */
    
    if (!m_wasScanSuccessful) {
      // received a beacon during channel scanning
      if (call BeaconFrame.parsePANDescriptor(frame, RADIO_CHANNEL, page, &m_PANDescriptor) == SUCCESS) {
	// let's see if the beacon is from our coordinator...
	if (m_PANDescriptor.CoordAddrMode == ADDR_MODE_SHORT_ADDRESS &&
	    m_PANDescriptor.CoordPANId == PAN_ID &&
	    m_PANDescriptor.CoordAddress.shortAddress == COORDINATOR_ADDRESS) {
	  // yes! wait until SCAN is finished, then syncronize to the beacons
	  m_wasScanSuccessful = TRUE;
	}
      }
    } else {
      //parseBeacon(payload);
      // Transmit in the CAP
      
      /* printf("\n"); */
      /* printfflush(); */
      
      if(InterfEnabled){
	SFStartTime = call SF.sfStartTime();			
	m_CapDuration = ((uint32_t) call SF.numCapSlots()) * (uint32_t) call SF.sfSlotDuration(); ;
	Gtime = (uint32_t) call SF.sfSlotDuration();
      
	call CapOverAlarm.startAt( SFStartTime , m_CapDuration - Gtime );			
	post packetSendTask_CAP();
	/* call TimerInterf.startOneShot(ITRF_ITRV); */
      }
      
      if (beaconSequenceNumber & 1) {
	call Leds.led2On();
      }	
      else{
	call Leds.led2Off();
      }
    }
    return frame;
  }

  /* event void TimerInterf.fired(){ */
  /*   post packetSendTask_CAP(); */
  /* } */

  async event void CapOverAlarm.fired() {
    /* post packetSendTask_CFP(); */
    atomic{ /* CapOver = TRUE;  */
      call CarrierTx.TxModeOff();
      call Resource.release();
    }
    call Leds.led1Off();
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
    /* printf("Inside MLME_SCAN.confirm \n"); */
    /* printfflush(); */
    
    if (m_wasScanSuccessful) {
      // we received a beacon from the coordinator before
      call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
      call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
      call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);

      //m_PANDescriptor_ext.CoordAddress.shortAddress=1;
      
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
  
  task void packetSendTask_CFP()
  {
    bool CoN=1;
    uint16_t NParent = COORDINATOR_ADDRESS; //65535; // This is the broadcast address
    
    if (!m_wasScanSuccessful) {
      return;
    } else if( call MCPS_DATA.request (
				       &m_frame, // frame,
				       m_payloadLen, // payloadLength,
				       0, // msduHandle,
				       TX_OPTIONS_GTS | TX_OPTIONS_ACK,// TxOptions,
				       CoN, // Change or not
				       NParent // New parent ID
				       ) != IEEE154_SUCCESS) {
      
      /* printf("MCPS-Data.request in CFP failure \n"); */
      /* printfflush(); */
      
      call Leds.led0Toggle(); //fail!
    } else
      {
	/* printf("MCPS-Data.request in CFP success \n"); */
	/* printfflush();	 */
	call Leds.led0Off();
      }
  }

  task void packetSendTask_CAP()
  {
    /* bool CoN=1; */
    /* uint16_t NParent = 65535; // This is the broadcast address */
    
    /* TDMA_frame_msg* tdma_msg = (TDMA_frame_msg*)call Packet.getPayload(&m_frame, sizeof(TDMA_frame_msg)); */
    /* TDMA_frame_msg* tdma_msg = (TDMA_frame_msg*)call RadioPacket.getPayload(&m_frame); */
    /* ieee154_txframe_t * m_154frame = (ieee154_txframe_t *)&m_frame; */
    /* tdma_msg -> Node_ID = TOS_NODE_ID;   */
    /* tdma_msg -> val = count++; */
    /* m_payloadLen = sizeof(TDMA_frame_msg); */
    
    if (!m_wasScanSuccessful) {
      return;
    } /* else if( call MCPS_DATA.request ( */
      /* 	       			       &m_frame, // frame, */
      /* 	       			       m_payloadLen, // payloadLength, */
      /* 	       			       0, // msduHandle, */
      /* 	       			       0, //TX_OPTIONS_ACK, //TX_OPTIONS_GTS | TX_OPTIONS_ACK,// TxOptions, */
      /* 	       			       CoN, // Change or not */
      /* 	       			       NParent // New parent ID */
      /* 	       			       ) != IEEE154_SUCCESS) */
      /* 	      call PhyTx.transmit(m_154frame, call SF.sfStartTime(), (call SF.sfSlotDuration())*2) != SUCCESS){ */
      /*   printf("MCPS-Data.request in CAP failure \n"); */
      /*   printfflush(); */
      /*   call Leds.led0Toggle(); //fail! */
    else if(InterfEnabled){
      if(call Resource.immediateRequest() != SUCCESS)
	call Leds.led0Toggle();
      else{
	call CarrierTx.TxModeOn();
	call Leds.led0Off();
	call Leds.led1On();
      }
    }
    /* } */ else{
      /* printf("MCPS-Data.request in CAP success \n"); */
      /* printfflush(); */
      //post packetSendTask_CFP();
    }
    
    /* if(InterfEnabled){ */
    /*   if(!CapOver){ */
    /* 	call TimerInterf.startOneShot(ITRF_ITRV); */
    /*   } */
    /* } */
  }

  /* event void PhyTx.transmitDone(ieee154_txframe_t *frame, error_t result){} */
  
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
    if (status == IEEE154_SUCCESS)
      call Leds.led1Toggle();

    /* if(InterfEnabled){ */
    /*   if(!CapOver) */
    /* 	post packetSendTask_CAP(); */
    /* } */
    
    /* printf("Message transmitted\n"); */
    /* printfflush();     */
  }
  
  event message_t* MCPS_DATA.indication (message_t* frame) {
    TDMA_frame_msg* tdma_msg = (TDMA_frame_msg*) call Packet.getPayload(frame, sizeof(TDMA_frame_msg) );
    ieee154_address_t deviceAddr;
    call Frame.getSrcAddr(frame, &deviceAddr);
    
    /* printf("Message received from node %x,  counter value %i\n",tdma_msg->Node_ID,tdma_msg->val);  */
    //printf("Message received \n");
    /* printfflush(); */
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
   * My addition
   *****************************************************************************************/
  
	void parseBeacon(uint8_t* payload) {
	  uint8_t pendAddrSpecOffset = GTS_LIST_MULTIPLY + (((payload[BEACON_INDEX_GTS_SPEC] & GTS_DESCRIPTOR_COUNT_MASK) > 0) ?
							    GTS_DIRECTION_FIELD_LENGTH + (payload[BEACON_INDEX_GTS_SPEC] & GTS_DESCRIPTOR_COUNT_MASK) * GTS_LIST_MULTIPLY: 0); // skip GTS
	  uint8_t pendAddrSpec = payload[pendAddrSpecOffset];
	  uint8_t *beaconPayload = payload + pendAddrSpecOffset + 1;
	  uint8_t pendingAddrMode = ADDR_MODE_NOT_PRESENT;
	  uint8_t frameLen = ((uint8_t*) payload)[0] & FRAMECTL_LENGTH_MASK;
	  uint8_t gtsFieldLength;
	  uint8_t m_numGtsSlots, m_numCapSlots, m_superframeOrder, m_sfSlotDuration
	    ,m_battLifeExtDuration, m_beaconOrder, m_framePendingBit;
	  
	  m_numGtsSlots = payload[BEACON_INDEX_GTS_SPEC] & GTS_DESCRIPTOR_COUNT_MASK;
	  gtsFieldLength = 1 + ((m_numGtsSlots > 0) ? GTS_DIRECTION_FIELD_LENGTH + m_numGtsSlots * GTS_LIST_MULTIPLY: 0);
	  if (m_numGtsSlots != IEEE154_aNumSuperframeSlots)
	    m_numCapSlots = ((payload[BEACON_INDEX_SF_SPEC2] & SF_SPEC2_FINAL_CAPSLOT_MASK) >> SF_SPEC2_FINAL_CAPSLOT_OFFSET) + 1;
	  else
	    m_numCapSlots = 0;
	  
	  m_superframeOrder = (payload[BEACON_INDEX_SF_SPEC1] & SF_SPEC1_SO_MASK) >> SF_SPEC1_SO_OFFSET;
	  m_sfSlotDuration = (((uint32_t) 1) << (m_superframeOrder)) * IEEE154_aBaseSlotDuration;
	  
	  m_battLifeExtDuration = 0;
	  
	  m_beaconOrder = (payload[BEACON_INDEX_SF_SPEC1] & SF_SPEC1_BO_MASK) >> SF_SPEC1_BO_OFFSET;
	  /* printf("\t\tSF Spec: BO=%u ; SO=%u ; CAP slots=%u ; BatteryLife=%u\n" */
	  /* 	 , m_beaconOrder, m_superframeOrder, m_numCapSlots, m_battLifeExtDuration); */
	  /* printf("\t\tGts Spec: DescCount=%u ; Length=%u bytes\n", m_numGtsSlots, gtsFieldLength); */
	  
	  if (m_numGtsSlots > 0)
	    parseGTSDescriptor(&payload[BEACON_INDEX_GTS_SPEC], m_numGtsSlots); 
	}
  
  void parseGTSDescriptor(uint8_t* payload, uint8_t m_numGtsSlots) {
    uint8_t* gtsSpec = payload+1;
    uint8_t i=0, direction;
    uint32_t gtsDirectionsMask;
    
    gtsDirectionsMask = 0x0;
    
    gtsDirectionsMask |= ( (uint32_t) *(gtsSpec++) << 24 );
    gtsDirectionsMask |= ( (uint32_t) *(gtsSpec++) << 16 );
    gtsDirectionsMask |= ( (uint32_t) *(gtsSpec++) << 8 );
    gtsDirectionsMask |= ( (uint32_t) *(gtsSpec++) << 0 );
    
    for (i = 0; i < m_numGtsSlots; i++) {
      direction = (gtsDirectionsMask >> i) & 1;
      
      /* printf("\t\t\t\t[%u] address=0x%x ; length=%u ; direction=%u \n" */
      /* 	     , gtsSpec[2] & GTS_STARTING_SLOT_MASK */
      /* 	     ,(gtsSpec[1] << 8 ) | gtsSpec[0] */
      /* 	     , ((gtsSpec[2] & GTS_LENGTH_MASK ) >> GTS_LENGTH_OFFSET) */
      /* 	     , direction); */
      gtsSpec += GTS_LIST_MULTIPLY;
    }  
  }
  /*****************************************************************************************
   * My addition
   *****************************************************************************************/
  event void UserButton.notify(button_state_t state){
    if(state == BUTTON_PRESSED){
      call Leds.led0On();
    }else if(state == BUTTON_RELEASED){
      InterfEnabled = TRUE;
      call Leds.led0Off();
    }
  }

  event void Resource.granted() { }

  async event error_t CarrierTx.channelIdle() { }

  async event error_t CarrierTx.channelBusy() { }
}
