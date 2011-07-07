/**
 * CfpTransmitP.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 * 	     Project: se.kth.tinyos2x.mac.tkn154
 *  	  Created on: 2010/04/19  
 * Last modification: 2010/08/18 
 *     		  @author: aitorhh
 *     
 */
/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @version $Revision: 1.0 $ $Date: 2010/08/18 $
 */

/** 
 * The MLME-SAP GTS management primitives define how GTSs are
 * requested and maintained. A device wishing to use these primitives
 * and GTSs in general will already be tracking the beacons of its PAN
 * coordinator. (IEEE 802.15.4-2006, Sect. 7.1.7)
 */

#include "TKN154_MAC.h"

module CfpTransmitP
{
	provides
	{
		interface Init;
		interface MLME_GTS;

		interface Notify<bool> as GtsSpecUpdated;
		interface GetNow<bool> as IsGtsRequestOngoing;
		interface GetNow<bool> as IsGtsOngoing;
		interface Notify<bool> as IsEndSuperframe;

		interface Get<ieee154_GTSentry_t*> as GetGtsRequestedDeviceEntry;

		interface FrameTx as CfpTx;
		interface FrameRx as CfpRx;

		interface Notify<bool> as WasRxEnabled;
	}
	uses
	{
		interface Leds;
		interface TransferableResource as RadioToken;
		interface GetNow<token_requested_t> as IsRadioTokenRequested;

		// TimeSlot timers
		interface Alarm<TSymbolIEEE802154,uint32_t> as CfpSlotAlarm;
		interface Alarm<TSymbolIEEE802154,uint32_t> as CfpEndAlarm;
		interface Alarm<TSymbolIEEE802154,uint32_t> as IsCfpRxSlot;
		interface Alarm<TSymbolIEEE802154,uint32_t> as IsCfpTxSlot;

		interface Alarm<TSymbolIEEE802154,uint32_t> as TrackAlarm;

		// timer to wait gtsDescriptor
		interface Timer<TSymbolIEEE802154> as GTSDescPersistenceTimeout;

		interface TimeCalc;
		interface GetNow<bool> as IsRxEnableActive;
		interface Notify<bool> as RxEnableStateChange;
		interface Notify<const void*> as PIBUpdateMacRxOnWhenIdle;

		interface SuperframeStructure as SF;
		interface RadioTx;
		interface RadioRx;
		interface RadioOff;
		interface Notify<ieee154_status_t> as HasRequestedCfpTimeSlot;
		interface Notify<ieee154_status_t> as CheckCfpTimeSlot;

		interface FrameTx as GtsRequestTx;
		interface Pool<ieee154_txframe_t> as TxFramePool;
		interface Pool<ieee154_txcontrol_t> as TxControlPool;
		interface MLME_GET;
		interface MLME_SET;
		interface FrameUtility;
		interface GtsUtility;

		interface IEEE154Frame as Frame;
		interface IEEE154BeaconFrame as BeaconFrame;

#ifndef IEEE154_BEACON_TX_DISABLED
		interface FrameRx as GtsRequestRx;
		interface GetSet<ieee154_GTSdb_t*> as GetSetGtsCoordinatorDb;
#else
		interface Get<ieee154_GTSentry_t*> as GetGtsDeviceDb;
#endif
	}
}
implementation {

#ifdef TKN154_DEBUG
	enum {
		HEADER_STR_LEN = 27,
		DBG_STR_SIZE = 250,
	};
	norace uint16_t m_dbgNumEntries;
	norace char m_dbgStr[HEADER_STR_LEN + DBG_STR_SIZE] = "updateState() transitions: ";
	void dbg_push_state(uint8_t state) {
		if (m_dbgNumEntries < DBG_STR_SIZE-3)
		m_dbgStr[HEADER_STR_LEN + m_dbgNumEntries++] = '0' + state;
	}
	void dbg_flush_state() {
		m_dbgStr[HEADER_STR_LEN + m_dbgNumEntries++] = '\n';
		m_dbgStr[HEADER_STR_LEN + m_dbgNumEntries++] = 0;
		dbg_serial("CfpTransmit",m_dbgStr);
		m_dbgNumEntries = 0;
	}
#else 
#define dbg_push_state(X)
#define dbg_flush_state()
#endif

	typedef enum {
		SWITCH_OFF,
		WAIT_FOR_RXDONE,
		WAIT_FOR_TXDONE,
		DO_NOTHING,
	}next_state_t;

#ifndef IEEE154_BEACON_TX_DISABLED
	enum {
		GTS_SLOT_PROCESS_DELAY = 2*IEEE154_MAX_BEACON_JITTER(0), //Delay to be sure that we have the radio state before the slot
		RADIO_TRANSFER_TO = RADIO_CLIENT_BEACONTRANSMIT,
	};

	/* ----------------------- Special Functions ----------------------- */
	task void processGtsRequest();
	task void processGtsRequestInvalid();

	/* ----------------------- Special vars ----------------------- */
#else
	enum {
		GTS_SLOT_PROCESS_DELAY = 2*IEEE154_MAX_BEACON_JITTER(0), //Delay to be sure that we have the radio state before the slot
		RADIO_TRANSFER_TO = RADIO_CLIENT_BEACONSYNCHRONIZE,
	};
#endif

	next_state_t tryReceive();
	next_state_t tryTransmit();
	next_state_t trySwitchOff();
	void updateState();
	void setCurrentFrame(ieee154_txframe_t *frame);
	task void signalTxDoneTask();
	void stopAllAlarms();
	task void wasRxEnabledTask();
	void hasStartSlot();

#ifdef TKN154_PERFORMANCE_PS
	/* ---------  Vars to compute the thoughtput ------------- */
	uint32_t packetsGtsRequest = 0;
	uint32_t packetsCfp = 0;
	uint32_t packetsCfpFAIL = 0;

#endif

	/* ----------------------- Vars to CFP tx ----------------------- */
	norace ieee154_status_t m_txStatus;
	norace uint32_t m_transactionTime;
	norace uint16_t m_slotDuration;
	norace uint16_t m_guardTime;
	norace uint32_t m_capDuration;
	norace uint32_t m_gtsDuration;
	norace uint32_t m_cfpInit;
	norace ieee154_macMaxFrameRetries_t m_macMaxFrameRetries;
	norace ieee154_macMaxFrameTotalWaitTime_t m_macMaxFrameTotalWaitTime;
	norace ieee154_macRxOnWhenIdle_t macRxOnWhenIdle;

	//TODO: norace deleted, check if is correct IN ALL!!!
	//TODO: separate commons vars from shared one
	norace bool m_lock;
	norace uint8_t m_slotNumber;
	norace ieee154_txframe_t *m_currentFrame;
	norace ieee154_txframe_t *m_lastFrame;

	/* ----------------------- Vars to GTS request ----------------------- */
	uint8_t m_payloadGtsRequest[2];
	norace bool m_gtsOngoingRequest = FALSE, m_gtsOngoing = FALSE;
	bool m_hasRequestedTimeSlot = FALSE;
	uint8_t m_gtsCharacteristicsType = GTS_DEALLOCATE_REQUEST;
	ieee154_GTSentry_t GtsEntryRequested;
	ieee154_GTSentry_t GtsEntryRequestedEmpty;

	/* ----------------------- Vars to expiration GTS ----------------------- */
	ieee154_GTSentry_t* actualEntry;

	/***************************************************************************************
	 * INITIALIZE functions
	 ***************************************************************************************/
	error_t reset(error_t error)
	{
		uint8_t i = 0;

		if (call RadioToken.isOwner()) // internal error! this must not happen!
		return FAIL;
		if (m_currentFrame)
		signal CfpTx.transmitDone(m_currentFrame, error);
		if (m_lastFrame)
		signal CfpTx.transmitDone(m_lastFrame, error);

		m_currentFrame = m_lastFrame = NULL;
		m_macMaxFrameTotalWaitTime = call MLME_GET.macMaxFrameTotalWaitTime();

		stopAllAlarms();
		return SUCCESS;
	}

	command error_t Init.init() {
		return reset(IEEE154_TRANSACTION_OVERFLOW);
	}

	/***************************************************************************************
	 * MLME_GTS commands
	 ***************************************************************************************/
#ifdef IEEE154_BEACON_TX_DISABLED

	command ieee154_status_t MLME_GTS.request (
			uint8_t GtsCharacteristics,
			ieee154_security_t *security
	) {

		ieee154_status_t status = IEEE154_SUCCESS;
		ieee154_txframe_t *txFrame=0;
		ieee154_txcontrol_t *txControl=0;
		ieee154_address_t srcAddress;
		ieee154_macPANId_t srcPANId = call MLME_GET.macPANId();
		uint8_t GtsCharacteristicsV[3];

		srcAddress.shortAddress = call MLME_GET.macShortAddress();

		if (security && security->SecurityLevel)
		status = IEEE154_UNSUPPORTED_SECURITY; //status = IEEE154_INVALID_PARAMETER; condition 11
		else if (m_gtsOngoingRequest || !(txFrame = call TxFramePool.get()))
		status = IEEE154_TRANSACTION_OVERFLOW;
		else if (!(txControl = call TxControlPool.get())) {
			call TxFramePool.put(txFrame);
			status = IEEE154_TRANSACTION_OVERFLOW;
		} else if(srcAddress.shortAddress == 0xFFFE || srcAddress.shortAddress == 0xFFFF) {
			status = IEEE154_NO_SHORT_ADDRESS;
		}

		if (status == IEEE154_SUCCESS) {

			txFrame->header = &txControl->header;
			txFrame->metadata = &txControl->metadata;

			txFrame->headerLen = call FrameUtility.writeHeader(
					txFrame->header->mhr,
					ADDR_MODE_NOT_PRESENT,
					0,
					0,
					ADDR_MODE_SHORT_ADDRESS,
					srcPANId,
					&srcAddress,
					0);

			txFrame->header->mhr[MHR_INDEX_FC1] = FC1_ACK_REQUEST | FC1_FRAMETYPE_CMD;
			txFrame->header->mhr[MHR_INDEX_FC2] = FC2_SRC_MODE_SHORT;

			m_payloadGtsRequest[0] = CMD_FRAME_GTS_REQUEST;
			m_payloadGtsRequest[1] = GtsCharacteristics;
			txFrame->payload = m_payloadGtsRequest;
			txFrame->payloadLen = 2;
			m_gtsOngoingRequest = TRUE;

			//Save requestGTSEntry
			call GtsUtility.parseGtsCharacteristicsFromPayload(m_payloadGtsRequest, GtsCharacteristicsV);
			GtsEntryRequested.shortAddress = srcAddress.shortAddress;
			GtsEntryRequested.length = GtsCharacteristicsV[0];
			GtsEntryRequested.direction = GtsCharacteristicsV[1];
			m_gtsCharacteristicsType = GtsCharacteristicsV[2];

			if ((status = call GtsRequestTx.transmit(txFrame)) != IEEE154_SUCCESS) {
				m_gtsOngoingRequest = FALSE;
				call TxFramePool.put(txFrame);
				call TxControlPool.put(txControl);
			}
		} else
		signal MLME_GTS.confirm(GtsCharacteristics, status);

		return status;
	}
#else
	command ieee154_status_t MLME_GTS.request (
			uint8_t GtsCharacteristics,
			ieee154_security_t *security
	) {

		// when a MLME_GTS start a GTS deallocations
		if ((GtsCharacteristics & GTS_REQUEST_CHARACTERISTICS_MASK ) >> GTS_REQUEST_CHARACTERISTICS_OFFSET == GTS_DEALLOCATE_REQUEST) {

			signal MLME_GTS.confirm(GtsCharacteristics, IEEE154_SUCCESS);
			call GtsUtility.purgeGtsEntry(call GetSetGtsCoordinatorDb.get(), actualEntry->shortAddress,
					actualEntry->direction);
			// notify the beacon modification
			signal GtsSpecUpdated.notify(TRUE);

			// indicate to the high layer the deallocation		
			signal MLME_GTS.indication(actualEntry->shortAddress,
					call GtsUtility.setGtsCharacteristics(actualEntry->length,
							actualEntry->direction,
							GTS_DEALLOCATE_REQUEST),
					NULL);
		}
		return IEEE154_SUCCESS;
	}
#endif
	/***************************************************************************************
	 * GTS requests commands
	 ***************************************************************************************/
	async command error_t IsGtsRequestOngoing.getNow() {atomic return m_gtsOngoingRequest;}
	async command error_t IsGtsOngoing.getNow() {atomic return m_gtsOngoing;}

	event void GtsRequestTx.transmitDone(ieee154_txframe_t *txFrame, ieee154_status_t status)
	{

		call TxControlPool.put((ieee154_txcontrol_t*) ((uint8_t*) txFrame->header - offsetof(ieee154_txcontrol_t, header)));
		call TxFramePool.put(txFrame);

		if (status != IEEE154_SUCCESS) {
			dbg_serial("CfsTransmitP", "transmitDone() failed!\n");
			signal MLME_GTS.confirm(call GtsUtility.getGtsCharacteristics(txFrame->payload, txFrame->payloadLen), status);

			atomic m_gtsOngoingRequest = FALSE;
		} else {
			dbg_serial("CfsTransmitP", "GtsRequestTx.transmitDone() ok\n");

#ifdef TKN154_PERFORMANCE_PS
			packetsGtsRequest ++;
#endif	

			if( m_gtsCharacteristicsType == GTS_DEALLOCATE_REQUEST) {
				signal MLME_GTS.confirm(call GtsUtility.setGtsCharacteristics(GtsEntryRequested.length,
								GtsEntryRequested.direction,
								m_gtsCharacteristicsType),
						IEEE154_SUCCESS);

				atomic m_gtsOngoingRequest = FALSE;
			} else {
				// GTS_ALLOCATE_REQUEST
				// start the IEEE154_aGTSDescPersistenceTimeout timer
				call GTSDescPersistenceTimeout.startOneShot(
						IEEE154_aGTSDescPersistenceTime *
						(((uint32_t) 1 << call MLME_GET.macBeaconOrder()) + (uint32_t) 1) *
						(uint32_t) IEEE154_aBaseSuperframeDuration);
			}
		}
	}

#ifndef IEEE154_BEACON_TX_DISABLED
	event message_t* GtsRequestRx.received(message_t* frame)
	{
		ieee154_address_t srcAddress;
		uint8_t gtsCharacteristics[3];

		//gtsCharacteristics[3] = [length, direction, gtsCharacteristics]
		call Frame.getSrcAddr(frame, &srcAddress);

		if (call GtsUtility.parseGtsCharacteristicsFromFrame(frame, gtsCharacteristics) == IEEE154_SUCCESS) {
			GtsEntryRequested.shortAddress = srcAddress.shortAddress;
			GtsEntryRequested.length = gtsCharacteristics[0];
			GtsEntryRequested.direction = gtsCharacteristics[1];
			m_gtsCharacteristicsType = gtsCharacteristics[2];

			post processGtsRequest();
		}

		return frame;
	}

	task void processGtsRequest() {
		ieee154_GTSdb_t* GTS_db = call GetSetGtsCoordinatorDb.get();
		uint8_t lastSlot = (GTS_db->numGtsSlots > 0 ?
				GTS_db->db[ GTS_db->numGtsSlots-1 ].startingSlot : IEEE154_aNumSuperframeSlots);
		uint8_t startingSlot = lastSlot - GtsEntryRequested.length;
		uint8_t beaconOrder = (uint8_t) call MLME_GET.macBeaconOrder();
		uint8_t index;

		if(m_gtsCharacteristicsType == GTS_ALLOCATE_REQUEST) {

			//TODO: 1. aMinCAPLength OK?
			if ( (15 - GTS_db->numGtsSlots - GtsEntryRequested.length) * call SF.sfSlotDuration() < IEEE154_aMinCAPLength) {
				post processGtsRequestInvalid();
				return;
			}

			//2. 7 slots maximun
			if ( startingSlot < (IEEE154_aNumSuperframeSlots - CFP_NUMBER_SLOTS)) {
				//post processGtsRequestInvalid(); 
				return;
			}

			//3. Exists?
			if( (index = call GtsUtility.getGtsEntryIndex(GTS_db, GtsEntryRequested.shortAddress, GtsEntryRequested.direction)
					)!= GTS_db->numGtsSlots) {
				return;
			}

			call GtsUtility.addGtsEntry(GTS_db, GtsEntryRequested.shortAddress, startingSlot,
					GtsEntryRequested.length, GtsEntryRequested.direction );

		} else {
			call GtsUtility.purgeGtsEntry(GTS_db, GtsEntryRequested.shortAddress,
					GtsEntryRequested.direction);
		}
		// generate the GTS descriptor with the requested specifications
		signal GtsSpecUpdated.notify(TRUE);
		signal MLME_GTS.indication(GtsEntryRequested.shortAddress,
				call GtsUtility.setGtsCharacteristics(GtsEntryRequested.length,
						GtsEntryRequested.direction,
						m_gtsCharacteristicsType ),
				NULL);
		return;
	}

	task void processGtsRequestInvalid() {
		uint8_t length = 0;
		call GtsUtility.addGtsEntry(call GetSetGtsCoordinatorDb.get(), GtsEntryRequested.shortAddress, 0, length, GtsEntryRequested.direction );
		signal GtsSpecUpdated.notify(TRUE);

		// empty gts descriptor remains in the beacon IEEE154_aGTSDescPersistenceTime then remove it
		call GTSDescPersistenceTimeout.startOneShot(
				IEEE154_aGTSDescPersistenceTime *
				(((uint32_t) 1 << call MLME_GET.macBeaconOrder()) + (uint32_t) 1) *
				(uint32_t) IEEE154_aBaseSuperframeDuration);

		memcpy(&GtsEntryRequestedEmpty, &GtsEntryRequested, sizeof(ieee154_GTSentry_t));
	}
#endif

	command ieee154_GTSentry_t* GetGtsRequestedDeviceEntry.get() {return &GtsEntryRequested;}

	/***************************************************************************************
	 * DEFAULTS MLME_GTS commands
	 ***************************************************************************************/
	default event void MLME_GTS.confirm (
			uint8_t GtsCharacteristics,
			ieee154_status_t status
	) {}

	default event void MLME_GTS.indication (
			uint16_t DeviceAddress,
			uint8_t GtsCharacteristics,
			ieee154_security_t *security
	) {}

	/***************************************************************************************
	 * DEFAULTS spec updated commands
	 ***************************************************************************************/
	command error_t GtsSpecUpdated.enable() {return FAIL;}
	command error_t GtsSpecUpdated.disable() {return FAIL;}
	default event void GtsSpecUpdated.notify( bool val ) {return;}

#ifdef IEEE154_BEACON_TX_DISABLED
	event void HasRequestedCfpTimeSlot.notify( ieee154_status_t val ) {
		actualEntry = call GetGtsDeviceDb.get();

		m_hasRequestedTimeSlot = (val == IEEE154_SUCCESS || val == IEEE154_DENIED ? TRUE : FALSE);

		dbg_serial("CfpTransmit", "IsGtsRequestOngoing -> HasRequestedCfpTimeSlot.notify( %u)\n",m_hasRequestedTimeSlot);

		if (m_hasRequestedTimeSlot) {
			// we have sent a request command and we have received the GTS descriptor
			if (m_gtsCharacteristicsType == GTS_ALLOCATE_REQUEST) {
				// the returned status could be SUCCESS or DENIED
				signal MLME_GTS.confirm(call GtsUtility.setGtsCharacteristics(GtsEntryRequested.length,
								GtsEntryRequested.direction,
								m_gtsCharacteristicsType),
						val);
				m_gtsOngoingRequest = FALSE;
				m_gtsOngoing = TRUE;

				call GTSDescPersistenceTimeout.stop(); // we receive the gts descriptor that we expect before timeout
			}
		}
	}

	event void CheckCfpTimeSlot.notify( ieee154_status_t val ) {
		uint8_t i = 0;

		m_hasRequestedTimeSlot = (val == IEEE154_SUCCESS || val == IEEE154_DENIED ? TRUE : FALSE);
		dbg_serial("CfpTransmit", "IsGtsOngoing -> CheckCfpTimeSlot.notify( %u)\n",m_hasRequestedTimeSlot);

		if (m_hasRequestedTimeSlot) {
			// we are just trying to check if we still have the slot
			for (i = 0; i < 2; i++)
			if ( actualEntry[i].startingSlot == 0) {
				signal MLME_GTS.indication(actualEntry[i].shortAddress,
						call GtsUtility.setGtsCharacteristics(actualEntry[i].length,
								actualEntry[i].direction, GTS_DEALLOCATE_REQUEST),
						NULL);
				call GtsUtility.setNullGtsEntry(actualEntry + i);
			}
			if ( actualEntry[GTS_RX_ONLY_REQUEST].startingSlot == 0 && actualEntry[GTS_TX_ONLY_REQUEST].startingSlot == 0)
				m_gtsOngoing = FALSE;
			else
				m_gtsOngoing = TRUE;

			return;
		}
		m_gtsOngoing = FALSE;
	}

#else
	event void HasRequestedCfpTimeSlot.notify( ieee154_status_t val ) {}
	event void CheckCfpTimeSlot.notify( ieee154_status_t val ) {}

#endif

	/***************************************************************************************
	 * GTS SEND FUNCTIONS
	 ***************************************************************************************/
	command ieee154_status_t CfpTx.transmit(ieee154_txframe_t *frame)
	{

		// request to send a frame in a GTS slot (triggered by MCPS_DATA.request())
		if (m_currentFrame != NULL) {
			// we've not finished transmitting the current frame yet
			dbg_serial("CfpTransmitP", "Overflow\n");
			return IEEE154_TRANSACTION_OVERFLOW;
		} else {
			setCurrentFrame(frame);
			dbg("CfpTransmitP", "New frame to transmit, DSN: %lu\n", (uint32_t) MHR(frame)[MHR_INDEX_SEQNO]);

			updateState();
			return IEEE154_SUCCESS;
		}
	}

	void setCurrentFrame(ieee154_txframe_t *frame)
	{
		ieee154_macDSN_t dsn = call MLME_GET.macDSN();
		frame->header->mhr[MHR_INDEX_SEQNO] = dsn++;
		call MLME_SET.macDSN(dsn);
		m_macMaxFrameRetries = call MLME_GET.macMaxFrameRetries();

		m_transactionTime = IEEE154_SHR_DURATION +
		(frame->headerLen + frame->payloadLen + 2) * IEEE154_SYMBOLS_PER_OCTET; // extra 2 for CRC
		if (frame->header->mhr[MHR_INDEX_FC1] & FC1_ACK_REQUEST)
		m_transactionTime += (IEEE154_aTurnaroundTime + IEEE154_aUnitBackoffPeriod +
				11 * IEEE154_SYMBOLS_PER_OCTET); // 11 byte for the ACK PPDU
		if (frame->headerLen + frame->payloadLen > IEEE154_aMaxSIFSFrameSize)
		m_transactionTime += call MLME_GET.macMinLIFSPeriod();
		else
		m_transactionTime += call MLME_GET.macMinSIFSPeriod();
		m_macMaxFrameTotalWaitTime = call MLME_GET.macMaxFrameTotalWaitTime();

		m_currentFrame = frame;

#ifdef TKN154_PERFORMANCE_PS
		(m_currentFrame->payload)[21] = packetsGtsRequest & 0xFF;
		(m_currentFrame->payload)[22] = packetsGtsRequest >> 8 & 0xFF;
		(m_currentFrame->payload)[23] = packetsGtsRequest >> 16 & 0xFF;
		(m_currentFrame->payload)[24] = packetsGtsRequest >> 24 & 0xFF;

		(m_currentFrame->payload)[25] = packetsCfp & 0xFF;
		(m_currentFrame->payload)[26] = packetsCfp >> 8 & 0xFF;
		(m_currentFrame->payload)[27] = packetsCfp >> 16 & 0xFF;
		(m_currentFrame->payload)[28] = packetsCfp >> 24 & 0xFF;

		(m_currentFrame->payload)[29] = packetsCfpFAIL & 0xFF;
		(m_currentFrame->payload)[30] = packetsCfpFAIL >> 8 & 0xFF;
		(m_currentFrame->payload)[31] = packetsCfpFAIL >> 16 & 0xFF;
		(m_currentFrame->payload)[32] = packetsCfpFAIL >> 24 & 0xFF;
#endif
	}

	/** 
	 * The updateState() function is called whenever something happened that
	 * might require a state transition; it implements a lock mechanism (m_lock)
	 * to prevent race conditions. Whenever the lock is set a "done"-event (from
	 * the SlottedCsmaCa/RadioRx/RadioOff interface) is pending and will "soon"
	 * unset the lock (and then updateState() will called again).  The
	 * updateState() function decides about the next state by checking a list of
	 * possible current states ordered by priority, e.g. it first always checks
	 * whether the CAP is still active. Calling this function more than necessary
	 * can do no harm.
	 */

	void updateState()
	{
		next_state_t next;

		atomic {
			// long atomics are bad... but in this block, once the/ current state has
			// been determined only one branch will/ be taken (there are no loops)
			if (m_lock || !call RadioToken.isOwner())
			return;
			m_lock = TRUE; // lock

			// Check 1: has the CFP finished?
			if (call TimeCalc.hasExpired(m_cfpInit,
							m_gtsDuration - call SF.guardTime()) || !call CfpEndAlarm.isRunning()) {

				dbg_push_state(1);
				if (call RadioOff.isOff()) {
					stopAllAlarms(); // may still fire, but is locked through isOwner()
					m_lock = FALSE; // unlock
					dbg_flush_state();

					dbg_serial("CfpTransmitP", "Handing over to Inactive Period in %lu.\n", call CfpEndAlarm.getNow() );
					signal IsEndSuperframe.notify(TRUE);

					call RadioToken.transferTo(RADIO_CLIENT_BEACONSYNCHRONIZE);
					return;
				} else
				next = SWITCH_OFF;
			}

			// Check 4: is some other operation (like MLME-SCAN or MLME-RESET) pending? 
			else if (call IsRadioTokenRequested.getNow()) {
				dbg_push_state(4);
				if (call RadioOff.isOff()) {
					stopAllAlarms(); // may still fire, but is locked through isOwner()
					// nothing more to do... just release the Token
					m_lock = FALSE; // unlock
					dbg_serial("CfpTransmitP", "Token requested: Handing over to Beacon Transmit/Synchronize.\n");
					signal IsEndSuperframe.notify(TRUE);

					call RadioToken.release();
					return;
				} else
				next = SWITCH_OFF;
			}

			// Check 6: is there a frame ready to transmit and where are in the correct slot?
			else if (call IsCfpTxSlot.isRunning() && m_currentFrame != NULL) {
				dbg_push_state(6);
				next = tryTransmit();
			}

			// Check 7: should we be in receive mode?
			//TODO: just be in the transmision mode in we have an slot asinged
			else if (call IsCfpRxSlot.isRunning() && !(call IsCfpTxSlot.isRunning())) {
				dbg_push_state(7);
				next = tryReceive();
				if (next == DO_NOTHING) {
					// if there was an active MLME_RX_ENABLE.request then we'll
					// inform the next higher layer that radio is now in Rx mode
					post wasRxEnabledTask();
				}
			}

			// Check 8: just make sure the radio is switched off  
			else {
				dbg_push_state(8);
				next = trySwitchOff();
			}

			// if there is nothing to do, then we must clear the lock
			if (next == DO_NOTHING)
			m_lock = FALSE;
		} // atomic

		// put next state in operation (possibly keeping the lock)
		switch (next)
		{
			case SWITCH_OFF: ASSERT(call RadioOff.off() == SUCCESS); break;
			case WAIT_FOR_RXDONE: break;
			case WAIT_FOR_TXDONE: break;
			case DO_NOTHING: break;
		}
	}

	next_state_t tryTransmit()
	{
		next_state_t next;

		if (!call RadioOff.isOff())
		next = SWITCH_OFF;
		else {
			uint32_t actualSlotDuration = m_slotDuration;// actualEntry->length;
			uint32_t dtMax = actualSlotDuration - m_transactionTime;

			//to debug: show in the packet payload the slotNumber
			//*(m_currentFrame-> payload) = call MLME_GET.macBeaconOrder();;

			// round to backoff boundary
			dtMax = dtMax + (IEEE154_aUnitBackoffPeriod - (dtMax % IEEE154_aUnitBackoffPeriod));
			if (dtMax > actualSlotDuration)
			dtMax = 0;
			if (call SF.battLifeExtDuration() > 0) {
				// battery life extension
				uint16_t bleLen = call SF.battLifeExtDuration();
				if (bleLen < dtMax)
				dtMax = bleLen;
			}
			if (call TimeCalc.hasExpired(m_cfpInit, (m_slotNumber*m_slotDuration) + dtMax )) {
				next = DO_NOTHING; // frame doesn't fit in the remaining CFP slot
			} else {
				error_t res;
				res = call RadioTx.transmit(m_currentFrame, call SF.sfStartTimeRef(), dtMax);
				next = WAIT_FOR_TXDONE; // this will NOT clear the lock
			}
		}
		return next;
	}

	next_state_t tryReceive()
	{
		next_state_t next;
		if (call RadioRx.isReceiving())
		next = DO_NOTHING;
		else if (!call RadioOff.isOff())
		next = SWITCH_OFF;
		else {
			call RadioRx.enableRx(0, 0);
			next = WAIT_FOR_RXDONE;
		}
		return next;
	}

	next_state_t trySwitchOff()
	{
		next_state_t next;
		if (call RadioOff.isOff())
		next = DO_NOTHING;
		else
		next = SWITCH_OFF;
		return next;
	}

	task void signalTxDoneTask()
	{
		ieee154_txframe_t *lastFrame = m_lastFrame;
		ieee154_status_t status = m_txStatus;
		m_lastFrame = NULL; // only now can the next transmission can begin 
		if (lastFrame) {
			dbg("CfpTransmitP", "Transmit done, DSN: %lu, result: 0x%lx\n",
					(uint32_t) MHR(lastFrame)[MHR_INDEX_SEQNO], (uint32_t) status);
			signal CfpTx.transmitDone(lastFrame, status);
		}
		updateState();
	}

	uint8_t getSlotNumber() {
		return (call SF.numCapSlots() + ((float) (call CfpEndAlarm.getNow()-m_cfpInit)/call SF.sfSlotDuration()) );
	}
	/***************************************************************************************
	 * ALARMS FUNCTIONS {Fired and init}
	 ***************************************************************************************/
	void stopAllAlarms() {
		call CfpEndAlarm.stop();
		call CfpSlotAlarm.stop();
		call IsCfpTxSlot.stop();
		call IsCfpRxSlot.stop();
	}

	async event void CfpEndAlarm.fired() {
		//dbg_serial("CfTransmit","CfpEndAlarm.fired() in %lu.\n", call CfpEndAlarm.getNow() );
		updateState();
	}

	async event void CfpSlotAlarm.fired() {
		//Check in which slot we are?
		hasStartSlot();
		//Fired next slot
		updateState();
	}

	async event void IsCfpRxSlot.fired() {}
	async event void IsCfpTxSlot.fired() {}

#ifndef IEEE154_BEACON_TX_DISABLED

	void hasStartSlot() {
		ieee154_address_t deviceShortAddress;
		bool txSlot;

		// to gts slot number = 6 (9) ..  0 (15)
		m_slotNumber = getSlotNumber();

		actualEntry = &((call GetSetGtsCoordinatorDb.get())->db[15 - m_slotNumber]);
		//		printf("[%u] sh=%u ; d=%u\n", m_slotNumber, actualEntry->shortAddress, actualEntry->direction);

		// the direction is refered to the device
		if (actualEntry->direction == GTS_RX_ONLY_REQUEST) {
			txSlot = (call Frame.getDstAddrTxFrame(m_currentFrame,&deviceShortAddress) == SUCCESS
					? (deviceShortAddress.shortAddress == actualEntry->shortAddress ? TRUE : FALSE)
					: FALSE);
			if (txSlot)
			call IsCfpTxSlot.start(actualEntry->length * m_slotDuration);

			call IsCfpRxSlot.stop();
			//call Leds.led2Toggle();

		} else if(actualEntry->direction == GTS_TX_ONLY_REQUEST) {
			call IsCfpRxSlot.start(actualEntry->length * m_slotDuration);
			call IsCfpTxSlot.stop();

			//call Leds.led1Toggle();
		} else {
			//call Leds.led0Toggle();
		}
		call CfpSlotAlarm.start(m_slotDuration);

	}

	event void GTSDescPersistenceTimeout.fired() {
		dbg_serial("CfpTransmitP","GTSDescPersistenceTimeout.fired()\n");

		call GtsUtility.purgeGtsEntry(call GetSetGtsCoordinatorDb.get(), GtsEntryRequestedEmpty.shortAddress,
				GtsEntryRequestedEmpty.direction);
	}

#else

	event void GTSDescPersistenceTimeout.fired() {
		dbg_serial("CfpTransmitP","GTSDescPersistenceTimeout.fired()\n");

		signal MLME_GTS.confirm(call GtsUtility.setGtsCharacteristics(GtsEntryRequested.length,
						GtsEntryRequested.direction,
						m_gtsCharacteristicsType),
				IEEE154_NO_DATA);
	}

	void hasStartSlot() {
		m_slotNumber = getSlotNumber();

		if (m_slotNumber == actualEntry[GTS_TX_ONLY_REQUEST].startingSlot) {
			//call Leds.led1Toggle();
			call IsCfpRxSlot.stop();

			call IsCfpTxSlot.start(m_slotDuration * actualEntry[GTS_TX_ONLY_REQUEST].length);
			dbg_serial("CfptransmitP", "TxStart: m_slotNumber = %u\n", m_slotNumber);
		} else if(m_slotNumber == actualEntry[GTS_RX_ONLY_REQUEST].startingSlot) {
			//call Leds.led2Toggle();
			call IsCfpTxSlot.stop();

			call IsCfpRxSlot.start(m_slotDuration * actualEntry[GTS_RX_ONLY_REQUEST].length);
			dbg_serial("CfptransmitP", "RxStart: m_slotNumber = %u\n", m_slotNumber);
		} else {
			//call Leds.led0Toggle();
		}

		call CfpSlotAlarm.start(m_slotDuration);

	}
#endif

	async event void TrackAlarm.fired() {

		dbg_serial("CfpTransmitP", "CFP time: %lu\n", m_gtsDuration);
		m_gtsDuration -= m_guardTime;
		dbg_serial("CfpTransmitP", "Got token, remaining CFP time: %lu\n",
				call SF.sfStartTime() + m_capDuration + m_gtsDuration - call CfpEndAlarm.getNow());

		dbg("CfpTransmitP", "m_slotDuration: %u\n", m_slotDuration);

		call CfpEndAlarm.startAt(m_cfpInit, m_gtsDuration);

		hasStartSlot();
		updateState();
	}
	/***************************************************************************************
	 * RADIO FUNCTIONS
	 ***************************************************************************************/

	async event void RadioToken.transferredFrom(uint8_t fromClient)
	{
		m_gtsDuration = ((uint32_t) call SF.numGtsSlots()) *
		(uint32_t) call SF.sfSlotDuration();
		//printf("sfO=%u\n", call MLME_GET.macSuperframeOrder());	
		//printf("bO=%u\n", call MLME_GET.macBeaconOrder());		

		m_guardTime = call SF.guardTime();

		//printfflush();

		if (m_gtsDuration < m_guardTime) {
			// CFP is too short to do something
			dbg_serial("CfpTransmitP", "CFP too short!\n");
			signal IsEndSuperframe.notify(TRUE);

			call RadioToken.transferTo(RADIO_TRANSFER_TO);
			return;
		}
		m_capDuration = ((uint32_t) call SF.numCapSlots()) *
		(uint32_t) call SF.sfSlotDuration();
		m_slotDuration = call SF.sfSlotDuration();
		m_cfpInit = call SF.sfStartTime() + m_capDuration;// - m_guardTime;

		if (!call TimeCalc.hasExpired(call SF.sfStartTime(), m_capDuration))
		call TrackAlarm.start(call TimeCalc.timeElapsed(call CfpEndAlarm.getNow(), m_cfpInit));
		else
		signal TrackAlarm.fired();
	}

	async event void RadioTx.transmitDone(ieee154_txframe_t *frame, const ieee154_timestamp_t *timestamp, error_t result) {
		bool done = TRUE;
		dbg("CfpTransmitP", "CfpTransmitP.transmitDone() in %lu -> %lu\n", call CfpEndAlarm.getNow(), (uint32_t) result);

		switch (result)
		{
			case SUCCESS:
			// frame was successfully transmitted, if ACK was requested
			// then a matching ACK was successfully received as well   
#ifdef TKN154_PERFORMANCE_PS
			packetsCfp ++;
#endif
			m_txStatus = IEEE154_SUCCESS;
			break;
			case FAIL:
#ifdef TKN154_PERFORMANCE_PS
			packetsCfpFAIL ++;
#endif
			m_txStatus = IEEE154_DISABLE_TRX_FAILURE;
			break;
			case ENOACK:
			// frame was transmitted, but we didn't receive an ACK (although
			// we requested an one). note: coordinator never retransmits an 
			// indirect transmission (see above) 
			if (m_macMaxFrameRetries > 0) {
				// retransmit
				done = FALSE;
				m_macMaxFrameRetries -= 1;
			} else {
				m_txStatus = IEEE154_NO_ACK;
#ifdef TKN154_PERFORMANCE_PS
				packetsCfpFAIL ++;
#endif
			}
			break;
			case EINVAL: // DEBUG!!!
			dbg_serial("CfpTransmitP", "EINVAL returned by transmitDone()!\n");
			// fall through
			default:
			ASSERT(0);
			break;
		}

		if (done) {
			m_lastFrame = m_currentFrame;
			m_currentFrame = NULL;
			post signalTxDoneTask();
		}

		m_lock = FALSE;
		updateState();
	}

	async event void RadioOff.offDone()
	{
		m_lock = FALSE;
		updateState();
	}

	async event void RadioRx.enableRxDone()
	{
		m_lock = FALSE;
		updateState();
	}

	event void RadioToken.granted()
	{
		ASSERT(0); // should never happen, because we never call RadioToken.request()
	}
	event message_t* RadioRx.received(message_t* frame, const ieee154_timestamp_t *timestamp)
	{
		// received a frame -> find out frame type and
		// signal it to responsible client component
		uint8_t *mhr = MHR(frame);
		uint8_t frameType = mhr[MHR_INDEX_FC1] & FC1_FRAMETYPE_MASK;

		if (frameType == FC1_FRAMETYPE_DATA) {
			dbg("CfpTransmit", "Received frame, DSN: %lu, type: 0x%lu\n",
					(uint32_t) mhr[MHR_INDEX_SEQNO], (uint32_t) frameType);
			return signal CfpRx.received(frame);
		} else
		return frame;
	}

	default event message_t* CfpRx.received(message_t* data) {return data;}

	/***************************************************************************************
	 * RxEnabled FUNCTIONS
	 ***************************************************************************************/

	task void wasRxEnabledTask()
	{
		signal WasRxEnabled.notify(TRUE);
	}
	command error_t WasRxEnabled.enable() {return FAIL;}
	command error_t WasRxEnabled.disable() {return FAIL;}

	event void RxEnableStateChange.notify(bool whatever) {updateState();}
	event void PIBUpdateMacRxOnWhenIdle.notify( const void* val ) {
		atomic macRxOnWhenIdle = *((ieee154_macRxOnWhenIdle_t*) val);
		updateState();
	}
	command error_t IsEndSuperframe.enable() {return FAIL;}
	command error_t IsEndSuperframe.disable() {return FAIL;}
	default event void IsEndSuperframe.notify( bool val ) {return;}
}
