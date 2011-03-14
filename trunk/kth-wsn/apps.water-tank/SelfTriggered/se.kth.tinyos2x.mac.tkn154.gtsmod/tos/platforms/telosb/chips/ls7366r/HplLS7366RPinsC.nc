/*
 * HplLS7366RPinsC.nc
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

configuration HplLS7366RPinsC {
  provides interface GeneralIO as SS;
  provides interface GeneralIO as SOMI;

}


implementation {

  components HplMsp430GeneralIOC as GeneralIOC;
  components new Msp430GpioC() as SSM;
  components new Msp430GpioC() as SOMIM;

  SSM -> GeneralIOC.Port63;
  SOMIM -> GeneralIOC.Port32;

  SS = SSM; 
  SOMI = SOMIM; 

}

