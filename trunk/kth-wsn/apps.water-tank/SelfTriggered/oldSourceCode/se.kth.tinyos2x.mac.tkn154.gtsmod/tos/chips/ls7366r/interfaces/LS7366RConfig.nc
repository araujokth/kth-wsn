/*
 * LS7366RConfig.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 * 		  	 Project: se.kth.tinyos2x.mac.tkn154
 *  	  Created on: 2010/06/02  
 * Last modification:  
 *     		  Author: Aitor Hernandez <aitorhh@kth.se>
 *     
 */

interface LS7366RConfig {

  /**
   * Sync configuration changes with the ls7366r hardware. This only
   * applies to set commands below.
   *
   * @return SUCCESS if the request was accepted, FAIL otherwise.
   */
  command error_t sync();
  event void syncDone( error_t error );

  /**
   * Change the state
   */
  command uint8_t getState();
  command void setState( uint8_t state );

  /**
   * Change the quadrature mode
   */
  command uint8_t getQuadMode();
  command void setQuadMode( uint8_t mode );
  
  /**
   * Change the counter size
   */
  command uint8_t getSize();
  command void setSize( uint8_t size );
  

  /**
   * Enable/Disable the counter
   */
  command bool getDisabled();
  command void setDisabled(bool enabled);
  
}
