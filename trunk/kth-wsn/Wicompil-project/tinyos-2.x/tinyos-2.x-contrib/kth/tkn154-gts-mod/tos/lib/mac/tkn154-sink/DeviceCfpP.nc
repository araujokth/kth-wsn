/*
 * Copyright (c) 2010, KTH Royal Institute of Technology
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
 * - Neither the name of the KTH Royal Institute of Technology nor the names 
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
 */
/**
 * The DeviceCfp component contains the functions
 * and provides the components needed exclusively
 * for the device node. It gives the functionalities
 * to be able to use the GTS with a device node in
 * conjuntion with the CfpTransmiP, which has the common
 * functions that coordinator and device need to share.
 * 
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @version  $Revision: 1.0 Date: 2010/06/05 $
 * @modified 2011/06/01
 */

#include "TKN154_MAC.h"

module DeviceCfpP
{
	provides {
		interface Init;
		interface MLME_GTS;
	}uses {
		interface MLME_SYNC_LOSS;
		interface MLME_GET;

		interface GtsUtility;
		interface FrameUtility;

		interface Pool<ieee154_txframe_t> as TxFramePool;
		interface Pool<ieee154_txcontrol_t> as TxControlPool;

		// timer to wait gtsDescriptor
		interface Timer<TSymbolIEEE802154> as GTSDescPersistenceTimeout;

		interface FrameTx as GtsRequestTx;
		interface PinDebug;

	}
}
implementation
{
	
	ieee154_GTSdb_t* db;

	/***************************************************************************************
	 * INITIALIZE FUNCTIONS
	 ***************************************************************************************/
	error_t reset(error_t error){return SUCCESS;}

	command error_t Init.init()
	{
		return reset(IEEE154_TRANSACTION_OVERFLOW);
	}

	event void MLME_SYNC_LOSS.indication (
			ieee154_status_t lossReason,
			uint16_t PANId,
			uint8_t LogicalChannel,
			uint8_t ChannelPage,
			ieee154_security_t *security
	)
	{
		// we lost sync to the coordinator -> spool out current packet
		reset(IEEE154_NO_BEACON);
		dbg_serial("DeviceCfp", "MLME_SYNC_LOSS.indication: GTS deallocated\n");
	}

	/***************************************************************************************
	 * GTS on going?
	 ***************************************************************************************/

	/***************************************************************************************
	 * Send GTS requests commands
	 ***************************************************************************************/
	event void GtsRequestTx.transmitDone(ieee154_txframe_t *txFrame, ieee154_status_t status){}

	/*
	 * GTS allocation/deallocation check
	 ***************************************************************************************/

	/***************************************************************************************
	 * Timers
	 ***************************************************************************************/
	event void GTSDescPersistenceTimeout.fired() {}

	/***************************************************************************************
	 * MLME_GTS commands
	 ***************************************************************************************/
	command ieee154_status_t MLME_GTS.request (
			uint8_t GtsCharacteristics,
			ieee154_security_t *security
	) {	return IEEE154_INVALID_PARAMETER;}
	/*
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

}
