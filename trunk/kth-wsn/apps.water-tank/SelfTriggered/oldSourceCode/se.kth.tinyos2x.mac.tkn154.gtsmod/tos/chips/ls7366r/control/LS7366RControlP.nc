/*
 * LS7366RControlP.nc
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

#include "Timer.h"

module LS7366RControlP @safe() {

  provides interface Init;
  provides interface Resource;
  provides interface LS7366RConfig;
  provides interface LS7366RReceive;

  uses interface GeneralIO as SS;
  uses interface GeneralIO as SOMI;


  uses interface LS7366RRegister as MDR0;
  uses interface LS7366RRegister as MDR1;
  uses interface LS7366RRegister as DTR;
  uses interface LS7366RRegister as CNTR;
  uses interface LS7366RRegister as OTR;
  uses interface LS7366RRegister as STR;
  
  uses interface LS7366RStrobe as LDOTR;
  uses interface LS7366RStrobe as CLRCNTR;

  uses interface Resource as SpiResource;
  uses interface Resource as SyncResource;
  
  uses interface BusyWait<TMicro,uint16_t> as BusyWait;
  
  uses interface Leds;
}

implementation {
  
  uint8_t m_size;
  uint8_t m_quad_mode;
  bool m_disabled;
  bool m_sync_busy;
      
  
  /***************** Prototypes ****************/

  task void sync();
  task void syncDone();
  void slopeHighLow();

  /***************** Init Commands ****************/
  command error_t Init.init() {
	call Leds.led1On();
	  
    call SS.makeOutput();
    call SOMI.makeInput();
    
    m_disabled = LS7366R_DEF_DISABLED;
    m_size = LS7366R_DEF_SIZE;
    m_quad_mode = LS7366R_DEF_QUAD_MODE;
            
    //Disable the chip enalbe by default
    call SS.set();
    
    post sync();
    
    return SUCCESS;
  }

  /***************** Resource Commands ****************/
  async command error_t Resource.immediateRequest() {
    error_t error = call SpiResource.immediateRequest();
    if ( error == SUCCESS ) {}
    return error;
  }

  async command error_t Resource.request() {
    return call SpiResource.request();
  }

  async command uint8_t Resource.isOwner() {
    return call SpiResource.isOwner();
  }

  async command error_t Resource.release() {
    atomic {
      return call SpiResource.release();
    }
  }

 default event void Resource.granted() {
	 call Resource.release();
 }
  
  
  /**
   * Sync must be called to commit software parameters configured on
   * the microcontroller (through the LS7366RConfig interface) to the
   * LS7366R encoder chip.
   */
  command error_t LS7366RConfig.sync() {
    call SyncResource.request();
    return SUCCESS;
  }
  
  
  /***************** Spi Resources Events ****************/
  event void SyncResource.granted() {
	  
    // One WR cycle for MDR0
	 call SS.clr();
	 atomic call MDR0.writeByte(m_quad_mode | 
	    		LS7366R_DEF_RUNNING_MODE | LS7366R_DEF_INDEX |
	    		LS7366R_DEF_CLK_DIV | LS7366R_DEF_SYNC_INDEX);
	 
	 slopeHighLow();    

    // One WR cycle for MDR1
    atomic call MDR1.writeByte(m_disabled << LS7366R_MDR1_ENABLE_COUNTER_SHIFT |
    			   (m_size & LS7366R_MDR1_COUNTER_BYTES_MASK));
	slopeHighLow();    
    
    // One WR cycle for DTR
   // atomic call DTR.writeWord( LS7366R_DEF_MOD_N_LIMIT-1);
	//slopeHighLow();    

    atomic call CLRCNTR.strobe();
    call SS.set();
    
    call SyncResource.release();
    post syncDone();
  }

  event void SpiResource.granted() {
    signal Resource.granted();
  }
 
  /***************** Tasks ****************/
  /**
   * Attempt to synchronize our current settings with the LS7366R
   */
  task void sync() {
    call LS7366RConfig.sync();
  }
  
  task void syncDone() {
    atomic m_sync_busy = FALSE;
    signal LS7366RConfig.syncDone( SUCCESS );
  }
  
  void slopeHighLow(){
	call SS.set();
	call BusyWait.wait(1);
	call SS.clr();
  }
  
  /***************** Functions ****************/
  
  /***************** LS7366RReceive Commands ****************/
	command error_t LS7366RReceive.receive(uint8_t* data){
		// To receive we have to send to instruction of which 
		// register we want to read, and then send a empty or
		// a dummy bytes. During the dummy bytes we have the
		// data in the other bus
		atomic{
			call SS.clr();
			call CNTR.read(data,call LS7366RConfig.getSize() );

			call SS.set();
		}
		call Resource.release();
		signal LS7366RReceive.receiveDone(data);
		
		return SUCCESS;
	}
  /***************** Defaults ****************/
  default event void LS7366RReceive.receiveDone( uint8_t* data ) {
		call Resource.release();
  }

  /***************** LS7366RConfig Commands ****************/
    command uint8_t LS7366RConfig.getState(){
  	  uint8_t state;
  	  
  	  call SS.clr();
  	  call STR.read(&state, 1);
  	  call SS.set();
  	  
  	  return state;
    }
    
    command void LS7366RConfig.setState( uint8_t state ){
  	  call SS.clr();
  	  call STR.writeByte(state);
  	  call SS.set();
    }
    
    command uint8_t LS7366RConfig.getQuadMode(){
  	  atomic return m_quad_mode;
    }
    
    command void LS7366RConfig.setQuadMode( uint8_t mode ){
  	    atomic m_quad_mode = mode;
  	    post sync();
    }

    command bool LS7366RConfig.getDisabled(){
  	    atomic return m_disabled;
    }
    command void LS7366RConfig.setDisabled( bool disabled ){
  	    atomic m_disabled = disabled;
  	    post sync();
    }
    
    command uint8_t LS7366RConfig.getSize(){
    	return m_size;
    }

    command void LS7366RConfig.setSize( uint8_t size ){
    	atomic m_size = size;
  	    post sync();

    }
  /***************** Defaults ****************/
  default event void LS7366RConfig.syncDone( error_t error ) {}
  
}
