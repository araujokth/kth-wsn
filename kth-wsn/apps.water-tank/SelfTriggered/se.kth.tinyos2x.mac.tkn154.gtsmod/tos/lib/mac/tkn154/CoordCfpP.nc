/**
 * CoordCfpP.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 * 		  	 Project: se.kth.tinyos2x.mac.tkn154
 *  	  Created on: 2010/04/19  
 * Last modification: 2010/06/30  
 *     		  @author: aitorhh
 *     
 */

/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @version $Revision: 1.0 $ $Date: 2010/06/30   $
 */
/**
 * This components would deal with the GTS for an outcoming superframe from 
 * the perspective of a coordinator. It contains the functions and variables 
 * necessaries for the coordinator to be able to transmit, receive and manage
 * the GTS.
 */

#include "TKN154_MAC.h"

#warning "Beacon transmission enabled -> COORDINADOR"
#warning "GTS enabled"

module CoordCfpP
{
	provides
	{
		interface Init;
		interface WriteBeaconField as GtsInfoWrite;

		interface GetSet<ieee154_GTSdb_t*> as GetSetGtsCoordinatorDb;
	}
	uses
	{
		interface MLME_GET;
		interface MLME_SET;
		interface IEEE154Frame as Frame;
		interface IEEE154BeaconFrame as BeaconFrame;
		interface GtsUtility;
	}
}
implementation
{

	/* variables that store the GTS entries */
	ieee154_GTSentry_t db[CFP_NUMBER_SLOTS];
	//uint8_t GTSdb.numGtsSlots;
	ieee154_GTSdb_t GTSdb;
	ieee154_macGTSPermit_t m_gtsPermit;

	/***************************************************************************************
	 * INITIALIZE FUNCTIONS
	 ***************************************************************************************/
	command error_t Init.init()
	{
		GTSdb.db = db;
		GTSdb.numGtsSlots = 0;

		if(call MLME_GET.macGTSPermit()) {
			//test_GTS(3);
			//call GtsUtility.addGtsEntry(call GetSetGtsCoordinatorDb.get(), 0x01, 15, 1, GTS_RX_ONLY_REQUEST);
			//call GtsUtility.addGtsEntry(call GetSetGtsCoordinatorDb.get(), 0x01, 14, 1, GTS_TX_ONLY_REQUEST);
			
			//call GtsUtility.addGtsEntry(call GetSetGtsCoordinatorDb.get(), 0x07, 13, 1, GTS_TX_ONLY_REQUEST);
			//call GtsUtility.addGtsEntry(call GetSetGtsCoordinatorDb.get(), 0x08, 12, 1, GTS_TX_ONLY_REQUEST);
			//call GtsUtility.addGtsEntry(call GetSetGtsCoordinatorDb.get(), 0x09, 11, 1, GTS_TX_ONLY_REQUEST);
			//call GtsUtility.addGtsEntry(call GetSetGtsCoordinatorDb.get(), 0x11, 10, 1, GTS_TX_ONLY_REQUEST);
		}

		return SUCCESS;
	}

	/***************************************************************************************
	 * BEACON FUNCTIONS
	 ***************************************************************************************/
	command uint8_t GtsInfoWrite.write(uint8_t *gtsSpecField, uint8_t maxlen)
	{
		// write the current GTS spec at the given address
		//this is the place to update the GTS Fields with db
		uint8_t len = call GtsInfoWrite.getLength();
		uint8_t directionMask = 0x00;
		uint8_t i;

		if (len > maxlen)
		return 0;

		//GTS Specification
		gtsSpecField[0] = ( GTSdb.numGtsSlots << 0) | (call MLME_GET.macGTSPermit() << 7);

		if(GTSdb.numGtsSlots > 0) {

			//GTS List
			for(i = 0; i < GTSdb.numGtsSlots; i++) {
				directionMask = directionMask | (db[i].direction << i);
				gtsSpecField[2] = db[i].shortAddress;
				gtsSpecField[3] = db[i].shortAddress >> 8;
				gtsSpecField[4] = ( (db[i].length << GTS_LENGTH_OFFSET) & GTS_LENGTH_MASK )
				| ( db[i].startingSlot & GTS_STARTING_SLOT_MASK );

				gtsSpecField = gtsSpecField + GTS_LIST_MULTIPLY;
			}

			//GTS Directions
			gtsSpecField = gtsSpecField - GTS_LIST_MULTIPLY*i;
			gtsSpecField[1] = directionMask & GTS_DIRECTIONS_MASK;
		}		
		return len;
	}

	command uint8_t GtsInfoWrite.getLength()
	{
		// GTS Specification (1byte) + GTS Directions (0/1 byte) + GTS List (3bytes * #Slots)
		return 1 + ((GTSdb.numGtsSlots > 0) ? 1 + GTSdb.numGtsSlots * GTS_LIST_MULTIPLY: 0);
	}

	command ieee154_GTSdb_t* GetSetGtsCoordinatorDb.get() {return &GTSdb;}

	command void GetSetGtsCoordinatorDb.set(ieee154_GTSdb_t* dbParam) {
		memcpy(dbParam, &GTSdb, sizeof(ieee154_GTSdb_t));
	}

}
