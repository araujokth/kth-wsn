/**
 * DeviceCfpP.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.mac.tkn154
 *        Created on: 2010/04/19  
 * Last modification: 2010/05/03  
 *                @author: aitorhh
 *     
 */
/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @version $Revision: 1.0 $ $Date: 2010/05/03   $
 */


/**
 * This components would deal with the GTS for an incoming superframe from 
 * the perspective of a device. It contains the functions and variables 
 * necessaries for the device to be able to transmit in the CFP
 */

#include "TKN154_MAC.h"
#warning "IEEE154_BEACON_TX_DISABLED: beacon transmission disabled -> DEVICE"
#warning "GTS enabled"

module DeviceCfpP
{
	provides {
		interface Init;

		interface Get<ieee154_GTSentry_t*> as GetGtsDeviceDb;

	}uses {
		interface MLME_SYNC_LOSS;

		interface GtsUtility;

	}
}
implementation
{

	norace ieee154_GTSentry_t GTSentry[2];

	/***************************************************************************************
	 * INITIALIZE FUNCTIONS
	 ***************************************************************************************/
	error_t reset(error_t error)
	{
		call GtsUtility.setNullGtsEntry(GTSentry);
		call GtsUtility.setNullGtsEntry(GTSentry + 1 );
		return SUCCESS;
	}

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

	command ieee154_GTSentry_t* GetGtsDeviceDb.get() {return GTSentry;}

}
