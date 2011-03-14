/*
 * LS7366RStrobe.nc
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
#include "LS7366R.h"

interface LS7366RStrobe {

	/*
	 * Send a strobe. It sends an instruction to the chip
	 * 
	 * 
	 * @return SUCCESS if the read was done, FAIL otherwise.
	 */
  async command error_t strobe();

}
