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
  //uses interface LocalTime<TMilli>;
  uses interface RootControl;
  uses interface Receive;
  uses interface CtpInfo as GetCTPData;
  uses interface CollectionPacket;
  uses interface CtpCongestion as Cngstn;
  uses interface CC2420Packet as RadioInterface;
  //uses interface LinkEstimator;
}
implementation {
  message_t packet;
  bool sendBusy = FALSE, CongStatus;
 
  am_addr_t source_r , parent_r, l_parent;
  uint32_t curr_time;
  uint16_t metric_r, etx_metric, data_r, seqno=0, seqno_r;
  uint8_t hopcount_r, tx_power;
  int8_t RSS;
  
  event void Boot.booted() {
    call RadioControl.start();
  }
  
  event void RadioControl.startDone(error_t err) {
    if (err != SUCCESS){
      call RadioControl.start();
    }
    else {
      call RoutingControl.start();
      
      if (TOS_NODE_ID == 1) { call RootControl.setRoot();
      printf("This is root \n"); printfflush();}
      else if ( (TOS_NODE_ID > 1) && (TOS_NODE_ID < 10) ){}
      else  { call Timer.startPeriodic(1000); }
    }
  }

  event void RadioControl.stopDone(error_t err) {}

  void sendMessage() {
    ECollMessage* msg = (ECollMessage*)call Send.getPayload(&packet, sizeof(ECollMessage));

    seqno++;
    //call GetCTPData.getParent(&parent);
    //call GetCTPData.getEtx(&metric);

    msg->seqno = seqno;
    
    if (call Send.send(&packet, sizeof(ECollMessage)) != SUCCESS) { 
    	call Leds.led1Toggle(); 
    }
    else {	sendBusy = TRUE;  }
    
  }
  
  
  event void Timer.fired() {
  
    call Leds.led2Toggle();
    
    if (!sendBusy){ 
		sendMessage();				
	} ;
      
  }
  
  event void Send.sendDone(message_t* m, error_t err) {
    if (err != SUCCESS) { call Leds.led0On(); }
    sendBusy = FALSE;
  }
 
 
  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
  if (len == sizeof(ECollMessage)) {
    //ECollMessage* pkt = (ECollMessage*) payload;

    //seqno_r = pkt -> seqno;
    //printf("Data received from %i\n",source_r);
   	//printfflush();
     
    //printf("Source ID:   %i\n", source_r);
    /*printf("Parent ID:   %i\n", parent_r);
    printf("ETX:   %i\n", metric_r);
	printf("Sequence No.:   %i\n", seqno_r);   
    printf("Hop count:   %i\n\n\n\n", hopcount_r);
    //printf("RSSI is:    %i\n\n\n\n", RSS);*/
    
    /*Source ID --- Parent ID --- Sequence number --- Time */
    //printf("%i %i %i T",-source_r,parent_r,seqno_r);
	//printfflush();
	
	call Leds.led1Toggle();
  } 
  return msg;
 }
 
}  // Implementation ends
