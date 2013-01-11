#include <Timer.h>
#include "EColl.h"
#include "printf.h"
#include "TreeRouting.h"
#include "CC2420.h"

module SenseCollectionC {
  uses interface Boot;
  uses interface SplitControl as RadioControl;
  uses interface StdControl as RoutingControl;
  uses interface Send;
  uses interface Leds;
  uses interface Timer<TMilli>;
  uses interface RootControl;
  uses interface Receive;
  uses interface CtpInfo as GetCTPData;
  uses interface CollectionPacket;
  uses interface CtpCongestion as Cngstn;
  uses interface CC2420Packet as RadioInterface;
  uses interface Capisrunning;
  
  //uses interface LinkEstimator;
}
implementation {
  /* message_t packet; */
  bool sendBusy = FALSE;
 
  //am_addr_t source_r , parent_r, l_parent;
  //uint32_t curr_time;
  //uint16_t metric_r, etx_metric, data_r, seqno=0, seqno_r;
  //uint8_t hopcount_r, tx_power;
  //int8_t RSS;
  
  /* uint16_t seqno = 0; */
   
  /* bool msg_missed = FALSE; */
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
      //else  { call Timer.startPeriodic(1000); }
    }
    
  }

  event void RadioControl.stopDone(error_t err) {}
   
  async event void Capisrunning.Caphasstarted(uint32_t t0 , uint32_t dt){ }

  async event void Capisrunning.Caphasfinished(){ }

  async event void Capisrunning.MyTShasStarted(){ }
  

  event void Timer.fired() {
    //call Leds.led2Toggle();    
  }
  
  event void Send.sendDone(message_t* m, error_t err) { sendBusy = FALSE; route_found = TRUE;}
  /**  if (err != SUCCESS) { call Leds.led0On(); }    
		sendBusy = FALSE;
   	    route_found = TRUE;       
  }**/
 
 
  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {return msg;}
  /**if (len == sizeof(ECollMessage)) {
    ECollMessage* pkt = (ECollMessage*) payload;  
    source_r = pkt -> source;
    call Leds.led1Toggle();
  }
  return msg;
 }**/
 
}  // Implementation ends
