/* 
 * Copyright (c) 2011, Technische Universitaet Berlin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of the Technische Universitaet Berlin nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ========================================================================
 * Author(s): Jan Hauer <hauer@tkn.tu-berlin.de>
 * ========================================================================
 */

#include "AM.h"
#include "TKN154.h"
#include "TKN154_MAC.h"
#include "printf.h"

/* This module implements the Active Message abstraction over the
 * nonbeacon-enabled variant of the IEEE 802.15.4 MAC. Currently, when this
 * module is enabled (via SplitControl.start()) it makes sure that radio is
 * always in receive mode (unless we're transmitting), i.e. the
 * LowPowerListening interface is not yet implemented and the duty cycle is
 * 100%.
 *
 * The IEEE 802.15.4 frame format is different from the frame format used in
 * TinyOS. This module transparently converts between the two, to make both
 * sides happy: the upper layer in TinyOS will see the actual AM payload
 * starting at message_t->data, which allows briding between interfaces (so
 * apps like Oscilloscope are working). And before passing frames down to the
 * 15.4 MAC we make sure the AM type (and network byte) are moved to the MAC
 * payload portion (at message_t->data). This workaround and involves extra
 * memmoves, and will disappear once TinyOS introduces a new message buffer
 * abstraction.
 *
 * There are some TinyOS/CC2420 macros that are supported (e.g. set via CFLAGS
 * in the Makefile): TFRAMES_ENABLED, CC2420_DEF_CHANNEL, 
 * CC2420_NO_ADDRESS_RECOGNITION, CC2420_NO_ACKNOWLEDGEMENTS
 **/
 
//#include "/home/kthwsn/local/src/tinyos-2.x/apps/EasyCollection/Sink/EColl.h"

module TKN154ActiveMessageP {
  provides {
    interface SplitControl;
    interface AMSend[am_id_t id];
    interface Receive[am_id_t id];
    interface Receive as Snoop[am_id_t id];
    interface Packet;
    interface AMPacket;
    interface PacketAcknowledgements;
    interface Capisrunning;
	interface Notify<bool> as GtsSpecUpdated;
	interface HasTimeSlot;
	
/*    interface PacketTimeStamp<T32khz, uint32_t> as PacketTimeStamp32khz;*/
/*    interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;*/
/*    interface LowPowerListening;*/
  }
  
  uses {
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface MLME_GTS;
		  	
    	interface State as SplitControlState;		
 		interface Leds;
		interface Packet as SubPacket;
 		
 		interface IEEE154Frame as Frame;
		interface IEEE154TxBeaconPayload;
		
		interface Get<ieee154_GTSdb_t*> as SetGtsCoordinatorDb;
		interface GtsUtility;
		interface Notify<bool> as IsEndSuperframe;
		interface GetNow<bool> as IsGtsOngoing;
    	
		interface SuperframeStructure as SF;		
		interface Alarm<TSymbolIEEE802154,uint32_t> as CapOverAlarm;

  }
}
implementation {

/**#ifdef TKN154_ACTIVE_MESSAGE_SUPPORT_DISABLED
// When TKN154_ACTIVE_MESSAGE_SUPPORT_DISABLED is defined this component cannot
// work, because it relies on some extra struct members which are disabled in 
// that case -> Make sure TKN154_ACTIVE_MESSAGE_SUPPORT_DISABLED is not defined!
#error "TKN154_ACTIVE_MESSAGE_SUPPORT_DISABLED must not be defined if AM is used!"
#endif**/

  enum {
    S_IDLE,
    S_STOPPED,
    S_STARTING,
    S_STARTED,
    S_STOPPING,
  };

  enum {
    MAX_RX_ON_TIME = 0xFFFFFF,
    ADDRESS_NOT_PRESENT = 0,

    PAYLOAD_OFFSET = 1, // 1 byte for AM type 

    T2_6LOWPAN_NETWORK_ID = 0x3f, // The 6LowPAN NALP ID for a TinyOS network (TEP 125)
    FC1_RESERVED_BIT      = 0x80,
    FC2_RESERVED_BIT      = 0x01,

    RADIO_CHANNEL = 0x14,   // 20 in decimal
    PAN_ID = 0x1234,
    COORDINATOR_ADDRESS = 0x0001,
    BEACON_ORDER = 5,
    SUPERFRAME_ORDER = 5,
    TX_POWER_BEACON = 0,
    TX_POWER = 0, // in dBm
  };

  uint32_t m_CapDuration;
  uint32_t Gtime;
  uint32_t SFStartTime;
	
  ieee154_PANDescriptor_t m_PANDescriptor, m_PANDescriptor_ext;
  ieee154_GTSdb_t* GTSdb;
  
  bool m_wasScanSuccessful;

    
  void setDefaultGtsDescriptor();
  void setEmptyGtsDescriptor();
  void startApp();
  uint8_t last_assigned_ts = 0;
  
///////////////////////////////////////////////////////////
  	
  error_t status2Error(ieee154_status_t status)
  {
    switch (status)
    {
      case IEEE154_SUCCESS: return SUCCESS;
      case IEEE154_FRAME_TOO_LONG: return ESIZE;
      case IEEE154_NO_ACK: return ENOACK; break;
      case IEEE154_TRANSACTION_OVERFLOW: return EBUSY;
      case IEEE154_PURGED: return ECANCEL;
      case IEEE154_CHANNEL_ACCESS_FAILURE: // fall through
      default: return status;
    }
  }

  ieee154_header_t *getIeee154Header(message_t* m)
  {
    uint8_t *p = call Frame.getHeader(m);
    ieee154_header_t *header = (ieee154_header_t *) (p - offsetof(ieee154_header_t, mhr));
    return header;
  }

  /**************** Functions dealing with ACKs ****************/

  void setAckRequest(message_t *msg)
  {
    // We don't use Frame.getHeader() because this function
    // will be called from async context
    ieee154_header_t *hdr = (ieee154_header_t *) msg;
    uint8_t *fcf1 = &(((uint8_t*) hdr->mhr)[MHR_INDEX_FC1]);
    *fcf1 |= FC1_ACK_REQUEST;
  }

  void clearAckRequest(message_t *msg)
  {
    ieee154_header_t *hdr = (ieee154_header_t *) msg;
    uint8_t *fcf1 = &(((uint8_t*) hdr->mhr)[MHR_INDEX_FC1]);
    *fcf1 &= ~FC1_ACK_REQUEST;
  }

  bool isAckRequested(message_t *msg)
  {
    uint8_t *fcf1 = &(((uint8_t*) call Frame.getHeader(msg))[MHR_INDEX_FC1]);
    return (*fcf1 & FC1_ACK_REQUEST) ? TRUE : FALSE;
  }

  // We need to remember if a packet was acknowledeged, and use a 
  // reserved flag in the 802.15.4 MAC header for that purpose
  bool wasAcked(message_t *msg)
  {
    ieee154_header_t *hdr = (ieee154_header_t *) msg;
    uint8_t *fcf2 = &(((uint8_t*) hdr->mhr)[MHR_INDEX_FC2]);
    return (*fcf2 & FC2_RESERVED_BIT) ? TRUE : FALSE;
  }

  void setWasAcked(message_t *msg)
  {
    uint8_t *fcf2 = &(((uint8_t*) call Frame.getHeader(msg))[MHR_INDEX_FC2]);
    *fcf2 |= FC2_RESERVED_BIT;
  }

  void clearWasAcked(message_t *msg)
  {
    uint8_t *fcf2 = &(((uint8_t*) call Frame.getHeader(msg))[MHR_INDEX_FC2]);
    *fcf2 &= ~FC2_RESERVED_BIT;
  }


  /***************** SplitControl & related commands ****************/
  
  command error_t SplitControl.start() 
  {
    if (call SplitControlState.requestState(S_STARTING) == SUCCESS) {
      error_t result = status2Error(call MLME_RESET.request(TRUE));
      if (result != SUCCESS)
        call SplitControlState.toIdle();
      return result; 
    } else if (call SplitControlState.isState(S_STARTED)) {
      return EALREADY;
    } else if (call SplitControlState.isState(S_STARTING)) {
      return SUCCESS;
    }
    return EBUSY;
  }


  command error_t SplitControl.stop() 
  {
    // The 15.4 MAC doesn't provide a stop command - instead we disable Rx and 
    // switch state (MLME_RESET would be executed during next SplitControl.start())
    
    if (call SplitControlState.isState(S_STARTED)) {
      call SplitControlState.forceState(S_IDLE);
      call MLME_SET.macRxOnWhenIdle(FALSE);
      signal SplitControl.stopDone(SUCCESS);
      return SUCCESS;
    } else if(call SplitControlState.isState(S_STOPPED)) {
      return EALREADY;
    } else if(call SplitControlState.isState(S_STOPPING)) {
      return SUCCESS;
    }
    return EBUSY;
  }


  /***************** Scanning and Initialization commands ****************/

  event void MLME_RESET.confirm(ieee154_status_t status)
  {
    error_t result = status2Error(status);
    
    if (result == SUCCESS) {
    	startApp();
    } 
    
    if (result != SUCCESS) {
      // something went wrong -> reset state, signal error to upper layer
      if (call SplitControlState.isState(S_STARTING))
        call SplitControlState.toIdle();
      signal SplitControl.startDone(result);
    } else {
      // will continue in MLME_RX_ENABLE.confirm and signal startDone() there
    }
  }


  void startApp(){

    	//call MLME_SET.phyTransmitPower(TX_POWER_BEACON);
		call MLME_SET.macShortAddress(TOS_NODE_ID);		
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
    	
    	// I donot know this 
    	// call MLME_SET.macRxOnWhenIdle(TRUE);
  }

  event void MLME_START.confirm(ieee154_status_t status) {

	if (status == IEEE154_SUCCESS) {
        call SplitControlState.forceState(S_STARTED);
	    signal SplitControl.startDone(SUCCESS);
	} 
	else {
		startApp();
	}
	
  }


 /*****************************************************************************************
  *B E A C O N     F R A M E     E V EN T S
  *****************************************************************************************/

  event void IEEE154TxBeaconPayload.aboutToTransmit() {}

  event void IEEE154TxBeaconPayload.setBeaconPayloadDone(void *beaconPayload, uint8_t length) {}

  event void IEEE154TxBeaconPayload.modifyBeaconPayloadDone(uint8_t offset, void *buffer, uint8_t bufferLength) {}

  event void IEEE154TxBeaconPayload.beaconTransmitted()
  {
	ieee154_macBSN_t beaconSequenceNumber = call MLME_GET.macBSN();
	
	if (beaconSequenceNumber & 1) { call Leds.led2On(); }
	else { call Leds.led2Off(); }
	
	SFStartTime = call SF.sfStartTime();			
	m_CapDuration = ((uint32_t) call SF.numCapSlots()) * (uint32_t) call SF.sfSlotDuration(); ;
	Gtime = (uint32_t) call SF.sfSlotDuration();
			
	call CapOverAlarm.startAt( SFStartTime , m_CapDuration - Gtime );		
	
	printf("\n");
	/* printf("transmitted beacon no %i\n", beaconSequenceNumber); */
	printfflush();
	
	// 255 means this is the coordinator	
	signal Capisrunning.Caphasstarted( SFStartTime , m_CapDuration);
		
  }

  async event void CapOverAlarm.fired() {
  	
	//printf("Cap finshed \n ");
	//printfflush();

	signal Capisrunning.Caphasfinished();
  }

 
 /*****************************************************************************************
  * S C H E D U L I N G
  *****************************************************************************************/
  
  void setEmptyGtsDescriptor() {
	ieee154_GTSdb_t* GTSdb;
	GTSdb = call SetGtsCoordinatorDb.get();				
	GTSdb->numGtsSlots = 0;
	signal GtsSpecUpdated.notify(TRUE);
  }
	
  void setDefaultGtsDescriptor() {
	uint8_t i;
	ieee154_GTSdb_t* GTSdb;
	GTSdb = call SetGtsCoordinatorDb.get();
	GTSdb->numGtsSlots = 0;

	call GtsUtility.addGtsEntry(GTSdb, 55, 31, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 2, 30, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 3, 29, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 4, 28, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 5, 27, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 6, 26, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 7, 25, 1, GTS_TX_ONLY_REQUEST);	
 	call GtsUtility.addGtsEntry(GTSdb, 8, 24, 1, GTS_TX_ONLY_REQUEST);
 	call GtsUtility.addGtsEntry(GTSdb, 9, 23, 1, GTS_TX_ONLY_REQUEST);
 	
 	signal GtsSpecUpdated.notify(TRUE);
  }

  /*****************************************************************************************
   * G T S    F U N C T I O N S 
   *****************************************************************************************/

	event void MLME_GTS.confirm (
			uint8_t GtsCharacteristics,
			ieee154_status_t status) {}

	event void MLME_GTS.indication (
			uint16_t DeviceAddress,
			uint8_t GtsCharacteristics,
			ieee154_security_t *security) {}

  /***************************************************************************************
   * S U P E R F R A M E    E V E N T S
   ***************************************************************************************/
	
  event void IsEndSuperframe.notify( bool val ) {
  	setDefaultGtsDescriptor();
  }

  /***************************************************************************************
   * D E F A U L T S     S P E C     U P D A T E     C O M M A N D S
   ***************************************************************************************/

  command error_t GtsSpecUpdated.enable() {return FAIL;}
  command error_t GtsSpecUpdated.disable() {return FAIL;}
  default event void GtsSpecUpdated.notify( bool val ) {return;}



  /*********************************** AMSend Command  ************************************/

  command error_t AMSend.send[am_id_t id](am_addr_t addr, message_t* msg, uint8_t len) 
  {
 	uint8_t *p = call Frame.getPayload(msg);
    ieee154_address_t destAddr;
    uint8_t txOptions = 0;
    ieee154_status_t status;
    ieee154_header_t *header = getIeee154Header(msg);

 	//printf(" message length in AMSend.send is %u  also offset is %u \n",
 	//call Frame.getPayloadLength(msg), offsetof(ieee154_header_t, mhr));
 	
 	//printfflush();
 	
	//printf("Source ID in tkn is %i  Coord_ID is %i\n",call MLME_GET.macShortAddress(), COORDINATOR_ADDRESS );
	//printfflush();
 	
 	
    //printf("tkn154ActiveMessageP  AMSend.send AM_ID = %lu, addr = %lu\n", 
    //(uint32_t) call AMPacket.source(msg), (uint32_t) addr);
    //printfflush();
    
    if (!call SplitControlState.isState(S_STARTED))
      return EOFF;

    if (len > call AMSend.maxPayloadLength[id]())
      return ESIZE;

    memmove(p + PAYLOAD_OFFSET, p, len);
    header->type = p[PAYLOAD_OFFSET-1] = header->type;

	//printf("p[PAYLOAD_Offset-1]  %u  Frame length  %u\n",header->type, call Frame.getPayloadLength(msg));
	//printfflush();


    // We intentionally overwrite some previously set MAC header fields to
    // mimic the behavior of the standard CC2420 driver in TinyOS

    destAddr.shortAddress = addr;
	call AMPacket.setSource(msg, TOS_NODE_ID);
	
    call Frame.setAddressingFields(msg,
        ADDR_MODE_SHORT_ADDRESS,
        ADDR_MODE_SHORT_ADDRESS,
        PAN_ID,
        &destAddr,
        NULL);

    if (isAckRequested(msg) && addr != AM_BROADCAST_ADDR) { txOptions = TX_OPTIONS_GTS | TX_OPTIONS_ACK; }
    else { txOptions = 0; }
	  	 
	status = call MCPS_DATA.request (
						msg, // frame,
						len + PAYLOAD_OFFSET, // payloadLength,
						0, // msduHandle,
						txOptions,// TxOptions,
						1, // Change or not
						addr // New parent ID
				  ); 
	

	//printf("The length after MCPS_DATA.request is %u\n ",len);
	//printfflush();
	
	
	//if 	(status != IEEE154_SUCCESS ){  call Leds.led0Toggle(); }					
    
    return status2Error(status);
  }

  command error_t AMSend.cancel[am_id_t id](message_t* msg) 
  {
    // We don't support cancel(). There is the MCPS_PURGE interface, but it
    // would require that the next higher layer keeps track of the 'msduHandle'
    // and there's no such concept in TinyOS ...
    return FAIL;
  }

  command uint8_t AMSend.maxPayloadLength[am_id_t id]() 
  {
    return call Packet.maxPayloadLength();
  }

  command void* AMSend.getPayload[am_id_t id](message_t* m, uint8_t len) 
  {
    return call Packet.getPayload(m, len);
  }

  /***************** AMPacket Commands ****************/

  command am_addr_t AMPacket.address() 
  {
    //return call ActiveMessageAddress.amAddress();
    return TOS_NODE_ID;
  }
 
  command am_addr_t AMPacket.destination(message_t* msg) 
  {
    ieee154_address_t address;    
    if (call Frame.getDstAddr(msg, &address) != SUCCESS)
      address.shortAddress = ADDRESS_NOT_PRESENT; // not present
    return address.shortAddress;
  }

  command void AMPacket.setSource(message_t* msg, am_addr_t addr) 
  {
    // The 15.4 MAC doesn't allow to set the source address of a packet
    // explicitly, instead it will automatically set it to 'macShortAddress'
    // when the frame is passed down (Tx path) via MCPS_DATA.request().  
    // The next higher layer may still want to use setSource() and source() 
    // to store some temporary data. We use a fixed offset, because the
    // TinyOS frame format is fixed...
    nxle_uint16_t *src = (nxle_uint16_t*) &(((uint8_t*) call Frame.getHeader(msg))[MHR_INDEX_ADDRESS + 4]); 
    *src = addr; 
  }

  command am_addr_t AMPacket.source(message_t* msg) 
  {
    // see comment for AMPacket.setSource()
    return *((nxle_uint16_t*) &(((uint8_t*) call Frame.getHeader(msg))[MHR_INDEX_ADDRESS + 4]));
  }

  command void AMPacket.setDestination(message_t* msg, am_addr_t addr) 
  {
    ieee154_address_t dstAddr;
    uint16_t panID = ADDRESS_NOT_PRESENT; // default

    call Frame.getDstPANId(msg, &panID);
    dstAddr.shortAddress = addr;
    call Frame.setAddressingFields(msg,
        ADDR_MODE_SHORT_ADDRESS,
        ADDR_MODE_SHORT_ADDRESS,
        panID, 
        &dstAddr,
        NULL);
  }

  command bool AMPacket.isForMe(message_t* msg) 
  {
    return (call AMPacket.destination(msg) == call AMPacket.address() 
        || call AMPacket.destination(msg) == AM_BROADCAST_ADDR);
  }

  command am_id_t AMPacket.type(message_t* msg) 
  {
    ieee154_header_t *header = getIeee154Header(msg);
    
    //if (header->type > 4) {header->type = 0xff; }
   // else {header->type = 0x00; }
    
    return header->type;
  }

  command void AMPacket.setType(message_t* msg, am_id_t type) 
  {
    ieee154_header_t *header = getIeee154Header(msg);
    header->type = type;
  }
  
  command am_group_t AMPacket.group(message_t* msg) 
  {
    uint16_t panID = ADDRESS_NOT_PRESENT; // default (if not set)
    call Frame.getDstPANId(msg, &panID);
    return panID;
  }

  command void AMPacket.setGroup(message_t* msg, am_group_t grp) 
  {
    ieee154_address_t dstAddr;
    dstAddr.shortAddress = call AMPacket.destination(msg);
   
    call Frame.setAddressingFields(msg,
        call Frame.getSrcAddrMode(msg),
        call Frame.getDstAddrMode(msg),
        grp,
        &dstAddr,
        NULL);
  }

  command am_group_t AMPacket.localGroup() 
  {
    //return call ActiveMessageAddress.amGroup(); 
    //return m_PANDescriptor.CoordAddress.shortAddress;
    return COORDINATOR_ADDRESS;
  }

  async command error_t PacketAcknowledgements.requestAck( message_t* msg )
  {
    setAckRequest(msg);
    return SUCCESS;
  }

  async command error_t PacketAcknowledgements.noAck( message_t* msg )
  {
    clearAckRequest(msg);
    return SUCCESS;
  }

  async command bool PacketAcknowledgements.wasAcked(message_t* msg)
  {
    return wasAcked(msg);
  }

  /***************** Packet interface ****************/

  command void Packet.clear(message_t* msg)
  {
    call SubPacket.clear(msg);
  }

  command void Packet.setPayloadLength(message_t* msg, uint8_t len) 
  {
    call SubPacket.setPayloadLength(msg, len);
  }

  command uint8_t Packet.payloadLength(message_t* msg)
  {
    return call SubPacket.payloadLength(msg);
  }

  command uint8_t Packet.maxPayloadLength()
  {
    uint8_t len = call SubPacket.maxPayloadLength();

    // TinyOS uses a static header of 9 byte. The maximum allowed
    // MAC payload is then 116. The MAC would signal an error later 
    // (when you call MCPS_DATA.request()), which we must avoid...
    if (len > 116)
      len = 116;
    else if (len < PAYLOAD_OFFSET)
      return 0;

    // Reserve one (or two) byte for the AM type and network ID
    // (they are part of the IEEE 802.15.4 MAC payload, not header!)
    len -= PAYLOAD_OFFSET;   

    return len;
  }

  command void* Packet.getPayload(message_t* msg, uint8_t len)
  {
    if (len > call Packet.maxPayloadLength())
      return NULL;
    else
      return call Frame.getPayload(msg);
  }


 /********************* HasTime slot interface ***********************/	

  command bool HasTimeSlot.hasTS(){
  	return call IsGtsOngoing.getNow();
  }	

 
  command error_t HasTimeSlot.setSchedule(uint8_t* Scdle, uint8_t nodes){
  	uint8_t i,j,k;
  	uint16_t Nodes;
  	bool cango = TRUE;
  	
 	ieee154_GTSdb_t* GTSdb;
	GTSdb = call SetGtsCoordinatorDb.get();
	GTSdb->numGtsSlots = 0;
  	
  	last_assigned_ts = 0;
  	
  	call GtsUtility.addGtsEntry(GTSdb, 1000, 31  , 1, GTS_TX_ONLY_REQUEST);
  	
  	/**for(i=last_assigned_ts; i < last_assigned_ts + nodes; i++){
  		call GtsUtility.addGtsEntry(GTSdb, (uint16_t)Scdle[i], 30 - i , 1, GTS_TX_ONLY_REQUEST);
  		printf(" ID: %i  ts:%i   ",(uint16_t)Scdle[i],30 - i);
  		printfflush();
  	}**/
	
	/**call GtsUtility.addGtsEntry(GTSdb, 1000, 31  , 1, GTS_TX_ONLY_REQUEST);
	printf(" ID: %i  ts:%i   ",(uint16_t)Scdle[0],30);
	
	k = 1;
	
  	for(i=0; i < nodes/2  + 1; i++){
  		for (j = 0 ; j < 2 ; j++){
  			if (k<nodes){
  				call GtsUtility.addGtsEntry(GTSdb, (uint16_t)Scdle[k], 29 - i , 1, GTS_TX_ONLY_REQUEST);
  				printf(" ID: %i  ts:%i   ",(uint16_t)Scdle[k],29 - i);
  				printfflush();
  				k++;
  			}
  		}
  	}**/
	
	
	/**call GtsUtility.addGtsEntry(GTSdb, 13, 31, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 2, 30, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 12, 29, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 3, 28, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 12, 27, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 4, 26, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 12, 25, 1, GTS_TX_ONLY_REQUEST);
	call GtsUtility.addGtsEntry(GTSdb, 5, 24, 1, GTS_TX_ONLY_REQUEST);**/		
	
	signal GtsSpecUpdated.notify(TRUE);  	
  	
  	//printf("time slots allocated \n");
  	//printfflush();

  	return SUCCESS;
  }	

  command uint8_t HasTimeSlot.getSchedule(uint8_t* Schedule){
  	return 0;
  }

  /***************** Timestamping ****************/

  // TODO -> the Frame.getTimestamp() command will return a 62.5 KHz timestamp
  // which must be converted to 32 KHz or milli LocalTime.

/*  async command bool PacketTimeStamp32khz.isValid(message_t* msg);*/
/*  async command uint32_t PacketTimeStamp32khz.timestamp(message_t* msg);*/
/*  async command void PacketTimeStamp32khz.clear(message_t* msg);*/
/*  async command void PacketTimeStamp32khz.set(message_t* msg, uint32_t value);*/

/*  async command bool PacketTimeStampMilli.isValid(message_t* msg);*/
/*  async command uint32_t PacketTimeStampMilli.timestamp(message_t* msg);*/
/*  async command void PacketTimeStampMilli.clear(message_t* msg);*/
/*  async command void PacketTimeStampMilli.set(message_t* msg, uint32_t value);*/

  /***************** MCPS_DATA events ****************/

  event void MCPS_DATA.confirm(message_t *frame,
                          uint8_t msduHandle,
                          ieee154_status_t status,
                          uint32_t Timestamp)
  {
    void *payload = call Frame.getPayload(frame);
    uint8_t payloadLength = call Frame.getPayloadLength(frame);

    // Remember if this packet was acked because we might need this info later
    // (PacketAcknowledgements.wasAcked()): it was acked if the ACK_REQUEST
    // flag was set and the MCPS_DATA.confirm eventpacket had a status code
    // IEEE154_SUCCESS.

    if (isAckRequested(frame) && status == IEEE154_SUCCESS)
      setWasAcked(frame);
    else
      clearWasAcked(frame);

    // revert the memmove we did in MCPS_DATA.request() above
    memmove(payload, payload + PAYLOAD_OFFSET, payloadLength - PAYLOAD_OFFSET);
    call Packet.setPayloadLength(frame, payloadLength - PAYLOAD_OFFSET);
	
	//call Packet.setPayloadLength(frame, payloadLength);
	
    dbg_serial("TKN154ActiveMessageP", "AMSend.sendDone[%lu], status: %lu\n", (uint32_t) call AMPacket.type(frame), (uint32_t) status);
    dbg_serial("TKN154ActiveMessageP", "... TxTime: %lu (valid: %lu)\n", Timestamp, (uint32_t) call Frame.isTimestampValid(frame));
    signal AMSend.sendDone[call AMPacket.type(frame)](frame, status2Error(status));
    dbg_serial_flush();
  }

  event message_t* MCPS_DATA.indication ( message_t* frame )
  {
    void *payload = call Frame.getPayload(frame);
    uint8_t payloadLen = call Frame.getPayloadLength(frame);
    ieee154_header_t *header = getIeee154Header(frame);

    // We have to be a bit careful here, because the MAC will accept frames that
    // in TinyOS a next higher layer does not expect -> filter those out
    if (!call Frame.hasStandardCompliantHeader(frame) || 
        call Frame.getFrameType(frame) != 1) // must be a DATA frame
      return frame;

    header->type = ((uint8_t*) payload)[PAYLOAD_OFFSET-1];
    
    // Shift the payload and add the AM type + network byte, because in the
    // IEEE 802.15.4 these are part of the MAC payload, rather than MAC header
    // (as which message_t treats them). We do this, because we want to
    // support bridging between interfaces e.g. radio/serial.
    payloadLen -= PAYLOAD_OFFSET;
    memmove(payload, payload + PAYLOAD_OFFSET, payloadLen);
    call Packet.setPayloadLength(frame, payloadLen);

    //printf("tkn154ActiveMessageP MCPS_Data.indication, Source: %lu,  AMtype: %lu, payloadlen: %lu\n", 
    //(uint32_t) call AMPacket.source(frame),(uint32_t) call AMPacket.type(frame), (uint32_t) payloadLen);
    //printfflush();
    
    
    dbg_serial_flush();

    if (call AMPacket.isForMe(frame))
      return signal Receive.receive[call AMPacket.type(frame)](frame, call AMSend.getPayload[call AMPacket.type(frame)](frame,0), payloadLen);
    else
      return signal Snoop.receive[call AMPacket.type(frame)](frame, call AMSend.getPayload[call AMPacket.type(frame)](frame,0), payloadLen); 
  }
  
  /***************** Misc. / defaults ****************/

  default event message_t* Receive.receive[am_id_t id](message_t* msg, void* payload, uint8_t len) { return msg; }
  default event message_t* Snoop.receive[am_id_t id](message_t* msg, void* payload, uint8_t len) { return msg; }
  default event void AMSend.sendDone[uint8_t id](message_t* msg, error_t err) { } 
  //default async event void Capisrunning.Caphasstarted(uint32_t t0 , uint32_t dt);	
  //default async event void Capisrunning.Caphasfinished(void);
}
