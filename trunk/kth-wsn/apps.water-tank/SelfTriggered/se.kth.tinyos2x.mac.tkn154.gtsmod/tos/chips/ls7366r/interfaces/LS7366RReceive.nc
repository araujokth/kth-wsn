/*
 * LS7366RReceive.nc
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

interface LS7366RReceive {

	command error_t receive(uint8_t* data);
	event void receiveDone(uint8_t* data);
}

