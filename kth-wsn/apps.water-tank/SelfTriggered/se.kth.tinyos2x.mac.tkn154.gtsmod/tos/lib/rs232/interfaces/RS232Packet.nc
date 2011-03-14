 /*
 * RS232Packet.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.mac.tkn154
 *        Created on: 2010/06/02  
 * Last modification:  
 *            Author: aitorhh
 *     
 */

interface RS232Packet {

  async command error_t readWord(uint16_t* data);
    
  async command error_t sendWord(uint16_t data);

}
