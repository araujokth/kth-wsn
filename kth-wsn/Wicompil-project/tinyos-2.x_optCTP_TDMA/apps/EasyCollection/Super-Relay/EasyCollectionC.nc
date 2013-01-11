#include <Timer.h>
#include "EColl.h"
#include "printf.h"
#include "TreeRouting.h"
#include "CC2420.h"

module EasyCollectionC {
  uses interface Boot;
  uses interface SplitControl as RadioControl;
  uses interface StdControl as RoutingControl;
  uses interface Send;
  uses interface Leds;
  uses interface Timer<TMilli>;
  uses interface RootControl;
  uses interface Receive;
  uses interface Capisrunning;
  
}
implementation {
  
  message_t packet;
  
  bool sendBusy = FALSE;
  bool newPacket = FALSE;
  
  uint16_t seqno = 0;
   
  bool msg_missed = FALSE;
  bool route_found  = FALSE;
  
  
  event void Boot.booted() {
    call RadioControl.start();
  }
  
  event void RadioControl.startDone(error_t err) {
    if (err != SUCCESS){
      call RadioControl.start();
    }
    else {
      call RoutingControl.start();
      
      if (TOS_NODE_ID == COORDINATOR_ADDRESS) { 
      	call RootControl.setRoot();
      	printf("This is root \n"); 
      	printfflush();
      }
      else  { call Timer.startPeriodic(1000); }
    }
    
  }

  event void RadioControl.stopDone(error_t err) {}
   
  async event void Capisrunning.Caphasstarted(uint32_t t0 , uint32_t dt){ }

  async event void Capisrunning.Caphasfinished(){
  	  		
	/**
	 * if (route_found == TRUE && newPacket == TRUE){

		//printf("App Layer control data relayed \n");
		//printfflush();
		
		newPacket = FALSE;
		call Send.send(&packet, sizeof(ECollMessage));
	}**/
	
  }
  
  async event void Capisrunning.MyTShasStarted(){ }
  

  event void Timer.fired() {
    //call Leds.led2Toggle();    
  }
  
  event void Send.sendDone(message_t* m, error_t err) { sendBusy = FALSE; route_found = TRUE;} 
 
  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
	
	ECollMessage* pkt;
	ECollMessage* msg1;
		  
    memcpy(&packet,msg,sizeof(ECollMessage));
    
    pkt = (ECollMessage*) payload;  
    msg1 = (ECollMessage*)call Send.getPayload(&packet, sizeof(ECollMessage));
	
    msg1->seqno = pkt->seqno; 
    
    printf("msg->seqno  %i   pkt->seqno   %i\n",msg1->seqno, pkt->seqno);
    printfflush();
        	  
	if (route_found == TRUE && sendBusy == FALSE){
	 	call Send.send(&packet, sizeof(ECollMessage));
	}
	  	  
	return msg;

  }
 
}  // Implementation ends