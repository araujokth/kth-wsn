/*
 * RS232ControlC.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.ieee802154.tkn154
 *        Created on: 2010/06/10 
 * Last modification: 
 *            Author: aitorhh
 *     
 */

#include "msp430usart.h"

generic configuration RS232ControlC() {

	  provides interface Resource;
}

implementation {
	  components RS232ControlP;
	  Resource = RS232ControlP;
	  
	  components MainC;
	  MainC.SoftwareInit -> RS232ControlP;
	  components LedsC as Leds;
	  RS232ControlP.Leds -> Leds;
	  
	  components new Msp430Uart0C() as UART;
	  UART.Resource -> RS232ControlP;
	  UART.UartStream -> RS232ControlP;
}
