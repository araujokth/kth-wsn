

#include "Timer.h"

module RS232ControlP @safe() {

  provides interface Init;
  provides interface Resource;
  provides interface RS232Packet;

	uses {
		interface Resource as UARTResource;
		interface Leds;
		interface UartStream;
	}
}

implementation {
  
      
  
  /***************** Prototypes ****************/


  /***************** Init Commands ****************/
  command error_t Init.init() {
	call Leds.led1On();
    
    return SUCCESS;
  }

  /***************** Resource Commands ****************/
  async command error_t Resource.immediateRequest() {
    error_t error = call UARTResource.immediateRequest();
    if ( error == SUCCESS ) {}
    return error;
  }

  async command error_t Resource.request() {
    return call UARTResource.request();
  }

  async command uint8_t Resource.isOwner() {
    return call UARTResource.isOwner();
  }

  async command error_t Resource.release() {
    atomic {
      return call UARTResource.release();
    }
  }

 default event void Resource.granted() {
	 call Resource.release();
 }
  
  
  /***************** Resources Events ****************/
  event void UARTResource.granted() {
    signal Resource.granted();
  }
 
  /***************** Tasks ****************/
  /***************** Functions ****************/
  /***************** Receive Commands ****************/
  /***************** Defaults ****************/
  /***************** Config Commands ****************/
  /***************** Defaults ****************/

  
}
