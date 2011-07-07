/**
 * GtsUtility.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 * 		  	 Project: se.kth.tinyos2x.mac.tkn154
 *  	  Created on: 2010/06/21  
 * Last modification: 
 *    		  Author: Aitor Hernandez <aitorhh@kth.se>
 *     
 */

/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @version $Revision: 1.0 $ $Date: 2010/06/21 $
 */

/** 
 * The GtsUtility interface allows to access and provides useful functions for the
 * management and information about the GTS descriptor, and everything related with 
 * the GTS
 */

#include <TKN154.h>
#include <message.h>

interface GtsUtility
{

	/***************************************************************************************
	 * Frame functions
	 ***************************************************************************************/

	/** 
	 * Get the Gts Characteristics from a frame payload
	 * 
	 * @param *payloadGts	Pointer to the frame payload with the GTS information
	 * @param payloadLen		Size of the payload
	 * 
	 * @return uint8_t	the GTS characteristics
	 *  */
	command uint8_t getGtsCharacteristics(uint8_t* payloadGts, uint8_t payloadLen);

	/**
	 * Return the GtsCharacteristics uint8_t to create a frame
	 * 
	 * @param length		Length of the requested slot
	 * @param direction	Direction of the requested slot 
	 * 					{0: Device to Coord; 1: Coord to Device}
	 * @param characteristicsType	Indicates if the requested slot, is to allocate 
	 * 								or deallocate
	 * 
	 * @return uint8_t the GTS characteristics to be writen in the payload
	 */
	command uint8_t setGtsCharacteristics(uint8_t length, uint8_t direction, uint8_t characteristicsType);

	/**
	 * Parse a frame to get the GtsCharacteristics
	 * 
	 * @param *frame		Pointer to the frme, where we get the characteristics vector
	 * @param *GtsCharacteristics	Vector to store the characteristics
	 * 
	 * @return ieee154_status_t
	 */
	command ieee154_status_t parseGtsCharacteristicsFromFrame(message_t *frame, uint8_t* GtsCharacteristics);

	/**
	 * Parse a payload to get the GtsCharacteristics
	 *
	 * @param *payloadGts		Pointer to the payload where we get the characteristics vector
	 * @param *GtsCharacteristics	Vector to store the characteristics
	 * 
	 * @return ieee154_status_t
	 */
	command ieee154_status_t parseGtsCharacteristicsFromPayload(uint8_t* payloadGts, uint8_t* GtsCharacteristics);
	
	

	/***************************************************************************************
	 * BeaconFrame functions
	 ***************************************************************************************/
	
	/**
	 * Get the GTS entry index, in case we have it, otherwise return the new one
	 * 
	 * @param GTS_db	Coordinator db with all the gts entries
	 * @param shortAddress	Adress of the GTS entry we want to find
	 * @param direction		Directio nof the GTS entry we want to find
	 * 
	 * @return uint8_t Index of the slot, if it doesn't exist the numGtsSlots is return
	 */
	command uint8_t getGtsEntryIndex(ieee154_GTSdb_t* GTS_db, uint16_t shortAddress, uint8_t direction );

	/**
	 * Add the new entry to the coordinator db
	 * 
	 * @param GTS_db	Coordinator db with all the gts entries
	 * @param shortAddress	Adress of the GTS entry
	 * @param startingSlot	Starting slot, provided to that entry
	 * @param length	Length of the slot
	 * @param direction		Direction of the GTS entry
	 * 
	 * @return              <code>FAIL</code> if the frame is not a beacon frame,
	 *                      <code>SUCCESS</code> otherwise
	 */	
	command error_t addGtsEntry(ieee154_GTSdb_t* GTS_db, uint16_t shortAddress, uint8_t startingSlot,
			uint8_t length, uint8_t direction);


	/**
	 * Purge the entry in the coordinator db. It reallocates the rest of the slots, to keep it in order
	 * 
	 * @param GTS_db	Coordinator db with all the gts entries
	 * @param shortAddress	Adress of the GTS entry
	 * @param direction		Direction of the GTS entry
	 * 
	 * @return              <code>FAIL</code> if the frame is not a beacon frame,
	 *                      <code>SUCCESS</code> otherwise
	 */	
	command error_t purgeGtsEntry(ieee154_GTSdb_t* GTS_db, uint16_t shortAddress,
			uint8_t direction);

	/**
	 * Reads the GTS Fields of a beacon frame (except GTS List).
	 *
	 * @param macPayloadField         the beacon MAC payload
	 * @param gtsDescriptorCount      a pointer to where the GTS Descriptor
	 * 			            count should be written
	 * @param gtsDirectionsMask       a pointer to where the GTS Directions Mask
	 *                                should be written
	 * 
	 * @return              <code>FAIL</code> if the frame is not a beacon frame,
	 *                      <code>SUCCESS</code> otherwise
	 */
	command error_t getGtsFields(uint8_t *macPayloadField,
			uint8_t* gtsDescriptorCount, uint8_t* gtsDirectionsMask);
	/**
	 * Reads the GTS List of a beacon frame
	 *
	 * @param macPayloadField         the beacon MAC paylaod
	 * @param gtsDescriptorCount	    the number of gts slots busy
	 * @param gtsDirectionsMask	    the mask for the directions
	 * @param GTSentry	            a pointer to where the GTS entries
	 * 				    should be written
	 * 
	 * @return              <code>SUCCESS</code> because we assure that we have a
	 * 					  valid beacon frame
	 */
	command error_t getGtsList(uint8_t *macPayloadField,
			uint8_t gtsDescriptorCount, uint8_t gtsDirectionsMask, ieee154_GTSentry_t* GTSentry);


	/**
	 * Return the requested timeslot configuration in the ieee154_GTSentry_t*
	 *
	 * @param payload       payload of the beacon frame
	 * @param numGtsSlots   number of slots that we have in the GTS descriptor
	 * @param GTSrequested* Pointer to the GTSentry in the device.
	 * @param GTS_db		Coordinator db with all the gts entries
	 * 
	 * @return ieee154_status_t
	 */
	command ieee154_status_t hasRequestedTimeSlot(uint8_t* payload, uint8_t numGtsSlots, ieee154_GTSentry_t* GTSrequested, ieee154_GTSentry_t* GTSdb);
	
	/**
	 * Once we have the resourse, we just need to check if our slot is still in the beacon
	 *
	 * @param payload       payload of the beacon frame
	 * @param numGtsSlots   number of slots that we have in the GTS descriptor
	 * @param GTS_db		Coordinator db with all the gts entries
	 * 
	 * @return ieee154_status_t
	 */
	command ieee154_status_t checkTimeSlot(uint8_t* payload, uint8_t numGtsSlots, ieee154_GTSentry_t* GTSdb);

	
	/**
	 * Empty the slot. Set the length and startting slot to zero; directio to two (invalid)
	 * and expiration to FALSE
	 *
	 * @param GTSentry*	   entry to emptye
	 * 
	 */
	command void setNullGtsEntry(ieee154_GTSentry_t* GTSentry);

}
