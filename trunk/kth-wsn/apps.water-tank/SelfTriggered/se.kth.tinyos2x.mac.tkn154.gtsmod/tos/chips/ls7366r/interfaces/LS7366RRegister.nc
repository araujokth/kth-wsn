/*
 * LS7366RRegister.nc
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

interface LS7366RRegister {

	/*
	 * Read an undefined number of bytes
	 * 
	 * @param
	 * 		*data	vector of bytes to return 
	 * 		length	number of bytes to read
	 * 
	 * @return SUCCESS if the read was done, FAIL otherwise.
	 */
	async command error_t read( uint8_t* data, uint8_t length);

	/*
	 * Read a word, 2 bytes
	 * 
	 * @param
	 * 		*data	variable to store the result
	 * 
	 * @return SUCCESS if the read was done, FAIL otherwise.
	 */
	async command error_t readWord(uint16_t* data);
	
	/*
	 * Read a byte
	 * 
	 * @param
	 * 		*data	variable to store the result
	 * 
	 * @return SUCCESS if the read was done, FAIL otherwise.
	 */
	async command error_t readByte(uint8_t* data);

	
	/*
	 * Write a word, 2 bytes
	 * 
	 * @param
	 * 		data	data to write in a register
	 * 
	 * @return SUCCESS if the read was done, FAIL otherwise.
	 */
	async command error_t writeWord(uint16_t data);

	/*
	 * Write a byte
	 * 
	 * @param
	 * 		data	data to write in a register
	 * 
	 * @return SUCCESS if the read was done, FAIL otherwise.
	 */
	async command error_t writeByte(uint8_t data);

}
