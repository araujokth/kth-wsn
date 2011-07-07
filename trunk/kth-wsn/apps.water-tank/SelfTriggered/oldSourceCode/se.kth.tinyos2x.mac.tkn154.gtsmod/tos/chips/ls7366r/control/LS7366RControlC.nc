/*
 * LS7366RControlC.nc
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
#include "IEEE802154.h"

configuration LS7366RControlC {

  provides interface Resource;
  provides interface LS7366RConfig;
  provides interface LS7366RReceive;  
}


implementation {
  

  components LS7366RControlP;
  Resource = LS7366RControlP;
  LS7366RConfig = LS7366RControlP;
  LS7366RReceive = LS7366RControlP;
		  
  components MainC;
  MainC.SoftwareInit -> LS7366RControlP;

  components HplLS7366RPinsC as Pins;
  LS7366RControlP.SS -> Pins.SS;
  LS7366RControlP.SOMI -> Pins.SOMI;


  components new LS7366RSpiC() as Spi;
  LS7366RControlP.SpiResource -> Spi;
  
  components BusyWaitMicroC as BusyWait;
  LS7366RControlP.BusyWait -> BusyWait;

  // registers
  LS7366RControlP.MDR0 -> Spi.MDR0;
  LS7366RControlP.MDR1 -> Spi.MDR1;
  LS7366RControlP.DTR -> Spi.DTR;
  LS7366RControlP.CNTR -> Spi.CNTR;
  LS7366RControlP.OTR -> Spi.OTR;
  LS7366RControlP.STR -> Spi.STR;
  
  // strobe
  LS7366RControlP.LDOTR -> Spi.LDOTR;
  LS7366RControlP.CLRCNTR -> Spi.CLRCNTR;
  
  components new LS7366RSpiC() as SyncSpiC;
  LS7366RControlP.SyncResource -> SyncSpiC;
  
  components LedsC as Leds;
  LS7366RControlP.Leds -> Leds;

}

