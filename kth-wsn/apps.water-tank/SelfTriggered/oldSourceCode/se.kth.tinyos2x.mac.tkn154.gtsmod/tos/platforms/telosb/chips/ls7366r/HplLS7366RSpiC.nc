/*
 * HplLS7366RSpiC.nc
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

generic configuration HplLS7366RSpiC() {
  
  provides interface Resource;
  provides interface SpiByte;
  provides interface SpiPacket;
  
}

implementation {

  components new Msp430Spi0C() as SpiC;
  
  Resource = SpiC;
  SpiByte = SpiC;
  SpiPacket = SpiC;
  
}