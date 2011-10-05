/*
 * NoCfpTransmitP.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 * 	     Project: se.kth.tinyos2x.mac.tkn154
 *  	  Created on: 2010/06/22  
 * Last modification:  
 *     		  @author: aitorhh
 *     
 */

/** 
 * The MLME-SAP GTS management primitives define how GTSs are
 * requested and maintained. A device wishing to use these primitives
 * and GTSs in general will already be tracking the beacons of its PAN
 * coordinator. (IEEE 802.15.4-2006, Sect. 7.1.7)
 */

#include "TKN154_MAC.h"

module NoCfpTransmitP
{
	provides
	{
		interface Init;
		interface MLME_GTS;

		interface Notify<bool> as GtsSpecUpdated;
		interface GetNow<bool> as IsGtsRequestOngoing;
		interface GetNow<bool> as IsGtsOngoing;

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

		// Timer to sync with cfp start time
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


	/***************************************************************************************
	 * INITIALIZE functions
	 ***************************************************************************************/
	error_t reset(error_t error)
	{
		return SUCCESS;
	}

	command error_t Init.init() {
		return reset(IEEE154_TRANSACTION_OVERFLOW);
	}

	/***************************************************************************************
	 * MLME_GTS commands
	 ***************************************************************************************/
	command ieee154_status_t MLME_GTS.request (
			uint8_t GtsCharacteristics,
			ieee154_security_t *security
	) {	return IEEE154_INVALID_GTS;}

	/***************************************************************************************
	 * GTS requests commands
	 ***************************************************************************************/
	async command error_t IsGtsRequestOngoing.getNow() {return FALSE;}
	async command error_t IsGtsOngoing.getNow() {return FALSE;}

	event void GTSDescPersistenceTimeout.fired() {}

#ifndef IEEE154_BEACON_TX_DISABLED
	event message_t* GtsRequestRx.received(message_t* frame) {return frame;}
#endif
	
	event void GtsRequestTx.transmitDone(ieee154_txframe_t *txFrame, ieee154_status_t status)	{}

	command ieee154_GTSentry_t* GetGtsRequestedDeviceEntry.get() {return NULL;}

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

	event void HasRequestedCfpTimeSlot.notify( ieee154_status_t val ) {}
	event void CheckCfpTimeSlot.notify( ieee154_status_t val ) {}


	/***************************************************************************************
	 * GTS SEND FUNCTIONS
	 ***************************************************************************************/
	command ieee154_status_t CfpTx.transmit(ieee154_txframe_t *frame)
	{	return IEEE154_INVALID_GTS;}

	/***************************************************************************************
	 * ALARMS FUNCTIONS {Fired and init}
	 ***************************************************************************************/
	async event void TrackAlarm.fired() {}
	async event void CfpEndAlarm.fired() {}
	async event void CfpSlotAlarm.fired() {}
	async event void IsCfpRxSlot.fired() {}
	async event void IsCfpTxSlot.fired() {}

	/***************************************************************************************
	 * RADIO FUNCTIONS
	 ***************************************************************************************/

	async event void RadioToken.transferredFrom(uint8_t fromClient)
	{	
	#ifndef IEEE154_BEACON_TX_DISABLED
		call RadioToken.transferTo(RADIO_CLIENT_BEACONTRANSMIT);
	#else
		call RadioToken.transferTo(RADIO_CLIENT_BEACONSYNCHRONIZE);
	#endif
	}

	async event void RadioTx.transmitDone(ieee154_txframe_t *frame, const ieee154_timestamp_t *timestamp, error_t result) {}

	async event void RadioOff.offDone() {}

	async event void RadioRx.enableRxDone() {}

	event void RadioToken.granted()
	{
		ASSERT(0); // should never happen, because we never call RadioToken.request()
	}
	event message_t* RadioRx.received(message_t* frame, const ieee154_timestamp_t *timestamp) {return frame;}

	default event message_t* CfpRx.received(message_t* data) {return data;}

	/***************************************************************************************
	 * RxEnabled FUNCTIONS
	 ***************************************************************************************/
	command error_t WasRxEnabled.enable() {return FAIL;}
	command error_t WasRxEnabled.disable() {return FAIL;}

	event void RxEnableStateChange.notify(bool whatever) {}
	event void PIBUpdateMacRxOnWhenIdle.notify( const void* val ) {}

}
