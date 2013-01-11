/* $Id: CtpForwardingEngineP.nc,v 1.24 2010-04-11 23:27:30 gnawali Exp $ */
/*
 * Copyright (c) 2008-9 Stanford University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL STANFORD
 * UNIVERSITY OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  This component contains the forwarding path of CTP Noe, the
 *  standard CTP implementation packaged with TinyOS 2.x. The CTP
 *  specification can be found in TEP 123.  The paper entitled
 *  "Collection Tree Protocol," by Omprakash Gnawali et al., in SenSys
 *  2009, describes the implementation and provides detailed
 *  performance results of CTP Noe.</p>
 *
 *  <p>The CTP ForwardingEngine is responsible for queueing and
 *  scheduling outgoing packets. It maintains a pool of forwarding
 *  messages and a packet send queue. A ForwardingEngine with a
 *  forwarding message pool of size <i>F</i> and <i>C</i>
 *  CollectionSenderC clients has a send queue of size <i>F +
 *  C</i>. This implementation several configuration constants, which
 *  can be found in <code>ForwardingEngine.h</code>.</p>
 *
 *  <p>Packets in the send queue are sent in FIFO order, with
 *  head-of-line blocking. Because this is a tree collection protocol,
 *  all packets are going to the same destination, and so the
 *  ForwardingEngine does not distinguish packets from one
 *  another. Packets from CollectionSenderC clients are sent
 *  identically to forwarded packets: only their buffer handling is
 *  different.</p>
 *
 *  <p>If ForwardingEngine is on top of a link layer that supports
 *  synchronous acknowledgments, it enables them and retransmits packets
 *  when they are not acked. It transmits a packet up to MAX_RETRIES times
 *  before giving up and dropping the packet. MAX_RETRIES is typically a
 *  large number (e.g., >20), as this implementation assumes there is
 *  link layer feedback on failed packets, such that link costs will go
 *  up and cause the routing layer to pick a next hop. If the underlying
 *  link layer does not support acknowledgments, ForwardingEngine sends
 *  a packet only once.</p> 
 *
 *  <p>The ForwardingEngine detects routing loops and tries to correct
 *  them. Routing is in terms of a cost gradient, where the collection
 *  root has a cost of zero and a node's cost is the cost of its next
 *  hop plus the cost of the link to that next hop.  If there are no
 *  loops, then this gradient value decreases monotonically along a
 *  route. When the ForwardingEngine sends a packet to the next hop,
 *  it puts the local gradient value in the packet header. If a node
 *  receives a packet to forward whose gradient value is less than its
 *  own, then the gradient is not monotonically decreasing and there
 *  may be a routing loop. When the ForwardingEngine receives such a
 *  packet, it tells the RoutingEngine to advertise its gradient value
 *  soon, with the hope that the advertisement will update the node
 *  who just sent a packet and break the loop. It also pauses the
 *  before the next packet transmission, in hopes of giving the
 *  routing layer's packet a priority.</p>
 *  
 *  <p>ForwardingEngine times its packet transmissions. It
 *  differentiates between four transmission cases: forwarding,
 *  success, ack failure, and loop detection. In each case, the
 *  ForwardingEngine waits a randomized period of time before sending
 *  the next packet. This approach assumes that the network is
 *  operating at low utilization; its goal is to prevent correlated
 *  traffic -- such as nodes along a route forwarding packets -- from
 *  interfering with itself.</p>
 *
 *  <p>While this implementation can work on top of a variety of link
 *  estimators, it is designed to work with a 4-bit link estimator
 *  (4B). Details on 4B can be found in the HotNets paper "Four Bit
 *  Link Estimation" by Rodrigo Fonseca et al. The forwarder provides
 *  the "ack" bit for each sent packet, telling the estimator whether
 *  the packet was acknowledged.</p>
 *
 *  @author Philip Levis
 *  @author Kyle Jamieson
 *  @date   $Date: 2010-04-11 23:27:30 $
 */

#include <CtpForwardingEngine.h>
#include <CtpDebugMsg.h>
#include "printf.h"
/* #include "/home/ziyang/local/src/tinyos-2.x/apps/EasyCollection/Sink/EColl.h" */
#include "../../../../apps/EasyCollection/Sink/EColl.h"
   
generic module CtpForwardingEngineP() {
  provides {
    interface Init;
    interface StdControl;
    interface Send[uint8_t client];
    interface Receive[collection_id_t id];
    interface Receive as Snoop[collection_id_t id];
    interface Intercept[collection_id_t id];
    interface Packet;
    interface CollectionPacket;
    interface CtpPacket;
    interface CtpCongestion;
  }
  uses {
    // These five interfaces are used in the forwarding path
    //   SubSend is for sending packets
    //   PacketAcknowledgements is for enabling layer 2 acknowledgments
    //   RetxmitTimer is for timing packet sends for improved performance
    //   LinkEstimator is for providing the ack bit to a link estimator
    interface AMSend as SubSend;
    interface PacketAcknowledgements;
    interface Timer<TMilli> as RetxmitTimer;
    interface LinkEstimator; 
    interface UnicastNameFreeRouting;
    interface Packet as SubPacket;

    interface LocalTime<TSymbolIEEE802154>;

    // These four data structures are used to manage packets to forward.
    // SendQueue and QEntryPool are the forwarding queue.
    // MessagePool is the buffer pool for messages to forward.
    // SentCache is for suppressing duplicate packet transmissions.
    interface Queue<fe_queue_entry_t*> as SendQueue;
    //interface Queue<message_t*> as RxMessageQueue;

    interface Pool<fe_queue_entry_t> as QEntryPool;
    interface Pool<message_t> as MessagePool;
    interface Cache<message_t*> as SentCache;
    
    interface Receive as SubReceive;
    interface Receive as SubSnoop;
    interface CtpInfo;
    interface RootControl;
    interface CollectionId[uint8_t client];
    interface AMPacket;
    interface Leds;
    interface Random;
    interface Capisrunning;
    //interface MyTimeSlot;

    // This implementation has extensive debugging instrumentation.
    // Wiring up the CollectionDebug interface provides information
    // on important events, such as transmissions, receptions,
    // and cache checks. The TinyOS release includes scripts for
    // parsing these messages.
    interface CollectionDebug;

    
    // The ForwardingEngine monitors whether the underlying
    // radio is on or not in order to start/stop forwarding
    // as appropriate.
    interface SplitControl as RadioControl;
  }
}
implementation {
  /* Helper functions to start the given timer with a random number
   * masked by the given mask and added to the given offset.
   */
  static void startRetxmitTimer(uint16_t mask, uint16_t offset);
  void clearState(uint8_t state);
  bool hasState(uint8_t state);
  void setState(uint8_t state);
  void aggpacketComplete(message_t* msg, bool success);
   
  bool canReTX = FALSE;
	
  // CTP state variables.
  enum {
    QUEUE_CONGESTED  = 0x1,  // Need to set C bit?
    ROUTING_ON       = 0x2,  // Forwarding running?
    RADIO_ON         = 0x4,  // Radio is on?
    ACK_PENDING      = 0x8,  // Have an ACK pending?
    SENDING          = 0x10, // Am sending a packet?
    WAIT_RETX		 = 0x20  // Waiting for re-tx in CAP
  };

  // Start with all states false
  uint8_t forwardingState = 0; 
  
  /* Network-level sequence number, so that receivers
   * can distinguish retransmissions from different packets. */
  uint8_t seqno;

  // The ETX at last successful transmission
  uint16_t LAST_NORMAL_ETX = 0;
  
  // The ETX bound after which the immidiate route update is triggered
  uint16_t BETA = 50;
  
  // If there is some data to be re-transmitted	
  bool hasReTxData = FALSE;
  
  // If we have new data to Tx
  bool newData = FALSE;
  
  // If we are having repeated failures
  bool doing_retries = FALSE;
  
  // Number of total re-tries
  uint8_t total_rets = 0;
  
  bool route_found = FALSE;

  uint32_t total_pkt_sent = 0;

  uint8_t payloadLen;
  am_addr_t dest;
  
  uint32_t Tprev[4] = {0, 0, 0, 0}; /* m14, m15, m16, m17 */
  
  // Helps in reception and concatenation
  typedef enum {
  	before_TS,
  	after_TS,   
   } SF_portion_t;

  norace SF_portion_t SF_portion = before_TS;

  // Total messages
  uint8_t AGGMSGS;
  uint8_t LSideMsgs;
  uint8_t RSideMsgs;
  
  // Retries
  uint8_t re_tries = MAXRETS;
  	
  // This is the length of the ctp data block
  const uint8_t ctp_data_frame_len = sizeof(ctp_data_header_t) + MSGLEN;
  
  // The maximum size of the aggregated data
  const uint8_t aggDataLen = MAXMSG*( sizeof(ctp_data_header_t) + MSGLEN ); 
     
  enum {
    CLIENT_COUNT = uniqueCount(UQ_CTP_CLIENT)
  };

  /* Each sending client has its own reserved queue entry.
     If the client has a packet pending, its queue entry is in the 
     queue, and its clientPtr is NULL. If the client is idle,
     its queue entry is pointed to by clientPtrs. */

 	fe_queue_entry_t* qe_rx;

	uint8_t* payload_temp = NULL;
	uint8_t* payload_temp_init = NULL;
	
	uint8_t* payload_aggtemp = NULL;
	uint8_t* payload_aggtemp_init = NULL;
	
	uint8_t* payload_agg = NULL;
	uint8_t* payload_agg_init = NULL;
	
	uint8_t* payload_lside = NULL;
	uint8_t* payload_lside_init = NULL;
	
	uint8_t* payload_rside = NULL;
	uint8_t* payload_rside_init = NULL;
	
	uint8_t* payload_reTx = NULL;
	uint8_t* payload_reTx_init = NULL;
  	  	
 	//message_t* newMsg;
  	ctp_data_header_t* hdr;
  

  fe_queue_entry_t clientEntries[CLIENT_COUNT];
  fe_queue_entry_t* ONE_NOK clientPtrs[CLIENT_COUNT];

  /* The loopback message is for when a collection roots calls
     Send.send. Since Send passes a pointer but Receive allows
     buffer swaps, the forwarder copies the sent packet into 
     the loopbackMsgPtr and performs a buffer swap with it.
     See sendTask(). */
     
  message_t loopbackMsg;
  message_t* ONE_NOK loopbackMsgPtr;
  
  message_t aggMsg;
  message_t* ONE_NOK aggMsgPtr;
  
  message_t aggtempMsg;
  message_t* ONE_NOK aggtempMsgPtr;  
  
  message_t tempMsg;
  message_t* ONE_NOK tempMsgPtr;  
    
  message_t lsideMsg;
  message_t* ONE_NOK lsideMsgPtr;  

  message_t rsideMsg;
  message_t* ONE_NOK rsideMsgPtr;  
  
  message_t reTxMsg;
  message_t* ONE_NOK reTxMsgPtr;

  
  command error_t Init.init() {
    int i;
    uint32_t Tinit;
    for (i = 0; i < CLIENT_COUNT; i++) {
      clientPtrs[i] = clientEntries + i;
      dbg("Forwarder", "clientPtrs[%hhu] = %p\n", i, clientPtrs[i]);
    }

    loopbackMsgPtr = &loopbackMsg;
    call SubPacket.setPayloadLength(loopbackMsgPtr, aggDataLen );
        
    reTxMsgPtr = &reTxMsg;
    call SubPacket.setPayloadLength(reTxMsgPtr, aggDataLen );    
	payload_reTx = call SubPacket.getPayload(reTxMsgPtr, aggDataLen);
	payload_reTx_init = payload_reTx;

    aggMsgPtr = &aggMsg;
    call SubPacket.setPayloadLength(aggMsgPtr, aggDataLen );    
	payload_agg = call SubPacket.getPayload(aggMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
	payload_agg_init = payload_agg;
    
    aggtempMsgPtr = &aggtempMsg;
    call SubPacket.setPayloadLength(aggtempMsgPtr, aggDataLen ); 
    
    tempMsgPtr = &tempMsg;
    call SubPacket.setPayloadLength(tempMsgPtr, ctp_data_frame_len );
    
    lsideMsgPtr =&lsideMsg;
    call SubPacket.setPayloadLength(lsideMsgPtr, aggDataLen );
	payload_lside = call SubPacket.getPayload(lsideMsgPtr, ctp_data_frame_len);
	payload_lside_init = payload_lside;

    rsideMsgPtr =&rsideMsg;
    call SubPacket.setPayloadLength(rsideMsgPtr, aggDataLen );
	payload_rside = call SubPacket.getPayload(rsideMsgPtr, ctp_data_frame_len);
	payload_rside_init = payload_rside;

	AGGMSGS = 0;
	
    seqno = 0;
    
    if (call RootControl.isRoot()){
      Tinit = call LocalTime.get();
      Tprev[0] = Tinit;
      Tprev[1] = Tinit;
      Tprev[2] = Tinit;
      Tprev[3] = Tinit;
    }
    
    return SUCCESS;
  }

  command error_t StdControl.start() {
    setState(ROUTING_ON);
    return SUCCESS;
  }

  command error_t StdControl.stop() {
    clearState(ROUTING_ON);
    return SUCCESS;
  }

  /* sendTask is where the first phase of all send logic
   * exists (the second phase is in SubSend.sendDone()). */
  task void sendTask();
  task void formPacketTask();
  
  /* ForwardingEngine keeps track of whether the underlying
     radio is powered on. If not, it enqueues packets;
     when it turns on, it then starts sending packets. */ 
  event void RadioControl.startDone(error_t err) {
    if (err == SUCCESS) {
      setState(RADIO_ON);
      if (!call SendQueue.empty()) {
	dbg("FHangBug", "%s posted sendTask.\n", __FUNCTION__);
        //post sendTask();
      }
    }
  }

  static void startRetxmitTimer(uint16_t window, uint16_t offset) {
    uint16_t r = call Random.rand16();
    r %= window;
    r += offset;
    call RetxmitTimer.startOneShot(window);  //r
    dbg("Forwarder", "Rexmit timer will fire in %hu ms\n", r);
  }
  
  /* 
   * If the ForwardingEngine has stopped sending packets because
   * these has been no route, then as soon as one is found, start
   * sending packets.
   */ 
  event void UnicastNameFreeRouting.routeFound() {
    
    /* printf("UnicastNameFreeRouting.routeFound()  \n"); */
    /* printfflush(); */
    
    route_found = TRUE;
    
    if (isRelay == FALSE || isController == TRUE)
    	signal Send.sendDone[0](tempMsgPtr, SUCCESS);

    //post sendTask();
  }

  event void UnicastNameFreeRouting.noRoute() {
    // Depend on the sendTask to take care of this case;
    // if there is no route the component will just resume
    // operation on the routeFound event
  }
  
  event void RadioControl.stopDone(error_t err) {
    if (err == SUCCESS) {
      clearState(RADIO_ON);
    }
  }

  ctp_data_header_t* getHeader(message_t* m) {
    return (ctp_data_header_t*)call SubPacket.getPayload(m, sizeof(ctp_data_header_t));
  }

  async event void Capisrunning.Caphasstarted(uint32_t t0 , uint32_t dt){
	atomic{
		canReTX = TRUE;  // This flag indicates that CAP is still running
		re_tries = MAXRETS;
		
		if (hasReTxData == TRUE){ // If data was not Tx in the last TS			
			 
			if (hasState(WAIT_RETX)){  // If this re_Tx is due to failed ACK
				//re_tries = re_tries - 1;
      			clearState(WAIT_RETX);	
      		}
      		
			payloadLen = call SubPacket.payloadLength(reTxMsgPtr);      
    		dest = call UnicastNameFreeRouting.nextHop();        		
    		call SubSend.send(dest, reTxMsgPtr, payloadLen);
			setState(SENDING);			
			//post sendTask();
		}
		
	}
	
	//printf("CAP started \n");
	//printfflush();
	
			
  }

  async event void Capisrunning.Caphasfinished(){		
	
	atomic{
		
		canReTX = FALSE;
		SF_portion = before_TS;		
		
		if(TOS_NODE_ID!=COORDINATOR_ADDRESS &&
			(hasState(RADIO_ON) && !hasState(SENDING)) ){ }
		
  }
  
 }

   async event void Capisrunning.MyTShasStarted(){
		atomic{
			SF_portion = after_TS;
		
			LSideMsgs = payload_lside - payload_lside_init;
  			RSideMsgs = payload_rside - payload_rside_init;
		    AGGMSGS = (LSideMsgs + RSideMsgs)/ctp_data_frame_len;
			
			clearState(SENDING);
			
			//printf("FE ts  %i  %i\n", hasState(RADIO_ON),!hasState(SENDING));
			//printfflush();
			
			if(TOS_NODE_ID!=COORDINATOR_ADDRESS &&
				( hasState(RADIO_ON) && !hasState(SENDING) ) && ( newData == TRUE || AGGMSGS!=0) ){   // Either we have own data to Tx or some thing to be relayed 	
				
				//printf("hahaha \n");
				//printfflush();
				
				post formPacketTask();				
				post sendTask();
				
		 } 
	}
   }

  /*
   * The send call from a client. Return EBUSY if the client is busy
   * (clientPtrs is NULL), otherwise configure its queue entry
   * and put it in the send queue. If the ForwardingEngine is not
   * already sending packets (the RetxmitTimer isn't running), post
   * sendTask. It could be that the engine is running and sendTask
   * has already been posted, but the post-once semantics make this
   * not matter. What's important is that you don't post sendTask
   * if the retransmit timer is running; this would circumvent the
   * timer and send a packet before it fires.
   */ 
   
  command error_t Send.send[uint8_t client](message_t* msg, uint8_t len) {
  	fe_queue_entry_t *qe;
  	uint8_t* payload = NULL;
  	
	newData = TRUE;          // We have new data from the application layer
	
    if (!hasState(ROUTING_ON)) {return EOFF;}
    if (len > call Send.maxPayloadLength[client]()) {return ESIZE;}
    
	call Packet.setPayloadLength(msg, len);
    
	hdr = getHeader(msg);
    hdr->origin = TOS_NODE_ID;
    hdr->originSeqNo  = seqno++;
    hdr->type = call CollectionId.fetch[client]();
    hdr->thl = 0;  
	
	call SubPacket.setPayloadLength(aggMsgPtr, aggDataLen );	
	payload_agg = payload_agg_init;

	// Filling up the big message, first own message	
	payload = call SubPacket.getPayload(msg, sizeof(ctp_data_header_t) + MSGLEN);	
	memcpy(payload_agg, payload , sizeof(ctp_data_header_t) + MSGLEN );
	payload_agg = payload_agg + sizeof(ctp_data_header_t) + MSGLEN;	
    
    clientPtrs[client] = NULL;
    
    return SUCCESS;
  }


  command error_t Send.cancel[uint8_t client](message_t* msg) {
    // cancel not implemented. will require being able
    // to pull entries out of the queue.
    return FAIL;
  }

  command uint8_t Send.maxPayloadLength[uint8_t client]() {
    return call Packet.maxPayloadLength();
  }

  command void* Send.getPayload[uint8_t client](message_t* msg, uint8_t len) {
    return call Packet.getPayload(msg, len);
  }

  /**
   * This task fills up the aggregated message before it is send
   * It is seperated from the Send.send()
   *  **/

  task void formPacketTask(){

  	uint8_t* payload = NULL;

  	LSideMsgs = payload_lside - payload_lside_init;
  	RSideMsgs = payload_rside - payload_rside_init;
        
    AGGMSGS = (LSideMsgs + RSideMsgs)/ctp_data_frame_len;
    
    //printf("FE formPackettask AGGMSGS %i\n",AGGMSGS);
    //printfflush();
    
    if(isRelay == FALSE ){    																	// This is originator node
    	call CtpPacket.setNumber(aggMsgPtr,AGGMSGS); 
    }   
    else {
    	call SubPacket.setPayloadLength(aggMsgPtr, aggDataLen );
    	payload = call SubPacket.getPayload(lsideMsgPtr, LSideMsgs );   	
		payload_agg = payload_agg_init;
    }  


	//printf("LSideMsgs %i   RSideMsgs %i  AGGMSGS %i\n",LSideMsgs,RSideMsgs,  AGGMSGS);
	//printfflush();
		
	// First the left side (before own TS) messages
	payload = call SubPacket.getPayload(lsideMsgPtr, LSideMsgs );	
	memcpy(payload_agg, payload , LSideMsgs );
	payload_agg = payload_agg + LSideMsgs;


	// Then the right side (after own TS) messages
	payload = call SubPacket.getPayload(rsideMsgPtr, RSideMsgs );	
	memcpy(payload_agg, payload , RSideMsgs );
	payload_agg = payload_agg + RSideMsgs;
	
	if(isRelay == TRUE ){ call CtpPacket.setNumber(aggMsgPtr,AGGMSGS-1);} 
	
    payload_lside = payload_lside_init;
	payload_rside = payload_rside_init;    
    payload_agg = payload_agg_init;
  	
  }


  /*
   * These is where all of the send logic is. When the ForwardingEngine
   * wants to send a packet, it posts this task. The send logic is
   * independent of whether it is a forwarded packet or a packet from
   * a send clientL the two cases differ in how memory is managed in
   * sendDone.
   *
   * The task first checks that there is a packet to send and that
   * there is a valid route. It then marshals the relevant arguments
   * and prepares the packet for sending. If the node is a collection
   * root, it signals Receive with the loopback message. Otherwise,
   * it sets the packet to be acknowledged and sends it. It does not
   * remove the packet from the send queue: while sending, the 
   * packet being sent is at the head of the queue; a packet is dequeued
   * in the sendDone handler, either due to retransmission failure
   * or to a successful send.
   */

  task void sendTask() {

	    uint16_t gradient;
	    		
	    if (hasState(SENDING)) {	      		
	      //printf("hasState(SENDING) in sendTask:   hasReTxData %i \n",hasReTxData);
      	  //printfflush();	
	    }

	    else if ((!call RootControl.isRoot() && 
		      !call UnicastNameFreeRouting.hasRoute()) ||
		     (call CtpInfo.getEtx(&gradient) != SUCCESS)) {
	      
	      //printf("Forwarder : no route, don't send, try again in %i.\n", NO_ROUTE_RETRY);
	      //printfflush();
	      return;
	    }
	    
    else {
	      /* We can send a packet.
		 First check if it's a duplicate;
		 if not, try to send/forward. */
	      error_t subsendResult;
	      
	      payloadLen = call SubPacket.payloadLength(aggMsgPtr);      
	      dest = call UnicastNameFreeRouting.nextHop();
		
	      if (call RootControl.isRoot()) {
			/* Code path for roots: copy the packet and signal receive. */
	        collection_id_t collectid = getHeader(aggMsgPtr)->type;
			
			uint8_t* payload = NULL;
			uint8_t* payload_lb = NULL;
			uint8_t* payload_lb_init = NULL;
			uint8_t payloadLength;
	        uint8_t aggMsgs,i;
	        
	        //printf("Forwarder : I'm a root, so loopback and signal receive.\n");
	        //printfflush();
			
			call SubPacket.setPayloadLength(loopbackMsgPtr, aggDataLen );
			call SubPacket.setPayloadLength(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN );
	
			memcpy(loopbackMsgPtr, aggMsgPtr, sizeof(message_t));		
	 		aggMsgs = call CtpPacket.getNumber(loopbackMsgPtr);		
	 	
	 		payload_lb = call SubPacket.getPayload(loopbackMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
	 		payload_lb_init = payload_lb;
	 			
			for (i = aggMsgs + 1; i>0; i--){   
	 			
				payload = call SubPacket.getPayload(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
	 			
	 			memcpy(payload , payload_lb , ctp_data_frame_len);	
			 	payloadLength =  call Packet.payloadLength(tempMsgPtr);
	    	 	
	    	 	tempMsgPtr = signal Receive.receive[collectid](tempMsgPtr,
								payload,
								payloadLength);	
									
				payload_lb = payload_lb + ctp_data_frame_len;				
				 	
			}
			
			payload_lb = payload_lb_init;		 
	        
	        signal SubSend.sendDone(aggMsgPtr, SUCCESS);
	      
	      }
	      else {
		/* The basic forwarding/sending case. */
		 
		//printf("The basic forwarding/sending case \n");
		//printfflush();
		 
		call CtpPacket.setEtx(aggMsgPtr, gradient);
		call CtpPacket.clearOption(aggMsgPtr, CTP_OPT_ECN | CTP_OPT_PULL);
		
		if (call PacketAcknowledgements.requestAck(aggMsgPtr) == SUCCESS) {
		  setState(ACK_PENDING);
		}
		
		if (hasState(QUEUE_CONGESTED)) {
		  call CtpPacket.setOption(aggMsgPtr, CTP_OPT_ECN);
		  
		  //printf("Forwarder:  QUEUE_CONGESTED\n ");
		  //printfflush();
		   
		  clearState(QUEUE_CONGESTED);
		}
		
		//printf("payloadLen  %i  no_of_msgs %i\n",payloadLen,call CtpPacket.getNumber(aggMsgPtr) );
		//printfflush();

	    //printf("FE Sendtask AGGMSGS %i\n",AGGMSGS);
    	//printfflush();
				
		subsendResult = call SubSend.send(dest, aggMsgPtr, payloadLen);
		if (subsendResult == SUCCESS) {
		  // Successfully submitted to the data-link layer.
		  setState(SENDING);
		  
		  //printf("Forwarder : subsend succeeded \n");
		  //printfflush();
		  
		  return;
		}
		// The packet is too big: truncate it and retry.
		else if (subsendResult == ESIZE) {
		  
		  //printf("Forwarder: subsend failed from ESIZE: truncate packet.\n");
		  //printfflush();
		  
		  call Packet.setPayloadLength(aggMsgPtr, call Packet.maxPayloadLength());		  
		  call CollectionDebug.logEvent(NET_C_FE_SUBSEND_SIZE);
		}
		else {
		  
		 // printf("Forwarder: subsend failed from %i\n", (int)subsendResult);
		  //printfflush();
		  
		}  // else
		
		
	   }  // Else basic forwarding/sending case	   
	 }  // We can send packet
  }  // post send task


  /*
   * The second phase of a send operation; based on whether the transmission was
   * successful, the ForwardingEngine either stops sending or starts the
   * RetxmitTimer with an interval based on what has occured. If the send was
   * successful or the maximum number of retransmissions has been reached, then
   * the ForwardingEngine dequeues the current packet. If the packet is from a
   * client it signals Send.sendDone(); if it is a forwarded packet it returns
   * the packet and queue entry to their respective pools.
   * 
   */

   void aggpacketComplete(message_t* msg, bool success) {
   	   	 
 	uint8_t* payload = NULL;
	uint8_t* payload_aggtemp = NULL;
	uint8_t* payload_aggtemp_init = NULL;
	uint8_t* payload1 = NULL;
    ECollMessage* pkt;
    uint16_t* etx_value;
    uint16_t* bn_etx_value;
    am_addr_t* bn_id;
    uint16_t* beacon_cnt;
    
	uint8_t aggMsgs = (LSideMsgs + RSideMsgs)/ctp_data_frame_len; // call CtpPacket.getNumber(qe->msg);
    uint8_t i;
    bool ownmsg = TRUE;    
    uint16_t org,sqn, time_hl;
    am_addr_t dest;
        
    call CtpInfo.getEtx(&etx_value);    
    call CtpInfo.getBestNeighborID(&bn_id);
    call CtpInfo.getBestNeighborEtx(&bn_etx_value);
    call CtpInfo.getBeaconCount(&beacon_cnt);
        
	call SubPacket.setPayloadLength(aggtempMsgPtr, aggDataLen );
	call SubPacket.setPayloadLength(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN );
    
    memcpy(aggtempMsgPtr, reTxMsgPtr, sizeof(message_t));	
	payload_aggtemp = call SubPacket.getPayload(aggtempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
	
	//printf("\n");
	//printf(" aggMsgs in aggpacketComplete %i\n",aggMsgs);
    //printfflush();
      
      	
	if(isRelay != TRUE) {i = aggMsgs + 1;}  // In case of a originator node, we have (own msg + aggregated msg)
	else {i = aggMsgs;}                     // In case of a relay node, we have just the aggregated messages
	
	for (; i>0; i--){
		
		payload = call SubPacket.getPayload(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
		memcpy(payload , payload_aggtemp , ctp_data_frame_len);
		
		//call Packet.setPayloadLength(tempMsgPtr, MSGLEN);
		hdr = getHeader(tempMsgPtr);
		
		org = hdr->origin;
		sqn = hdr->originSeqNo;
		time_hl = hdr->thl;
		
		payload1 = call Packet.getPayload(tempMsgPtr, call Packet.payloadLength(tempMsgPtr));
		pkt = (ECollMessage*) payload1;
		
		dest = call UnicastNameFreeRouting.nextHop();
		
		// For own data only
		if (ownmsg == TRUE && isRelay != TRUE){    // This is a originator node
										 		
	 		signal Send.sendDone[0](tempMsgPtr, success);	
	 		
	 		ownmsg = FALSE;
	 		newData = FALSE;
	 		
	 		if (success) {
	 			
		   		// NodeID	Event   Time	ParentID    ParentETX    BestNeighborID     BestNeighborETX   SourceID    Seqno   THL  retries   
				/* printf("%i  %i  %i  %i  %i  %i %i  %i  %i  %i %i\n",TOS_NODE_ID , 0 ,  */
				/* 	dest, etx_value, bn_id, bn_etx_value, org , pkt->seqno, time_hl ,beacon_cnt,total_pkt_sent); */
				/* printfflush();    	    	 */
            	
            	//printf("%s%i  %s%i  %s%i  %s%i  %s%i  %s%i %s%i T\n","Success_tx_from_node ", org ,"to_node ", dest ,"SeqNo ", sqn, "time_to_live " , time_hl , "AL_Seqno " , pkt->seqno, "isRelay " , isRelay , "re_tries ", re_tries );
            	//printfflush();
      		} else {
				//hasReTxData = TRUE;
		   		// NodeID	Event   Time	ParentID    ParentETX    BestNeighborID     BestNeighborETX   SourceID    Seqno   THL  retries   
				//printf("%i  %i  %i  %i  %i  %i %i  %i  %i  %i \n",TOS_NODE_ID , 399 ,
				//	dest, etx_value, bn_id, bn_etx_value, org , pkt->seqno, time_hl ,beacon_cnt,total_pkt_sent);
		        //printfflush();    	    	

      		}
	 				 		
	 	} else {                          // This is a relay node or a forwarded message from a coordinator
	 		if (success) {
				call SentCache.insert(tempMsgPtr);

		   		// NodeID	Event   Time	ParentID    ParentETX    BestNeighborID     BestNeighborETX   SourceID    Seqno   THL  retries   
				/* printf("%i  %i  %i  %i  %i  %i %i  %i  %i   %i %i\n",TOS_NODE_ID , 2 ,  */
				/* 	dest, etx_value, bn_id, bn_etx_value, org , pkt->seqno, time_hl ,beacon_cnt,total_pkt_sent); */
				/* printfflush();    	    	 */

      		}
      		else {
				//printf("%i  %i  %i  %i  %i  %i %i  %i  %i  %i \n",TOS_NODE_ID , 389 , 
				//	dest, etx_value, bn_id, bn_etx_value, org , pkt->seqno, time_hl ,re_tries);
		        //printfflush();    	    	

      		}
      		      		
	 	}	 	 	
	 	// Now next msg
		payload_aggtemp = payload_aggtemp + ctp_data_frame_len;
				
	} // For loop
	
	payload_aggtemp = payload_aggtemp_init;	
	payload_reTx = payload_reTx_init;
	
	if (hasReTxData == FALSE){
		call SubPacket.clear(reTxMsgPtr);
	}
	call SubPacket.clear(aggMsgPtr);
	
	AGGMSGS = 0;
	
 }



 event void SubSend.sendDone(message_t* msg, error_t error) {
  	
  	/** At this stage, there should only be the aggregated
  	 * message in the queue **/ 
    uint16_t* etx_value;
    uint8_t* payload1 = NULL;
    ECollMessage* pkt;
    uint16_t* bn_etx_value;
    am_addr_t* bn_id;
    uint16_t* beacon_cnt;
    
    uint16_t org,sqn, time_hl,gradient;
    am_addr_t dest;
        
    call CtpInfo.getEtx(&etx_value);    
    call CtpInfo.getBestNeighborID(&bn_id);
    call CtpInfo.getBestNeighborEtx(&bn_etx_value);
	call CtpInfo.getBeaconCount(&beacon_cnt);
		
	hdr = getHeader(aggMsgPtr);		
	org = hdr->origin;
	sqn = hdr->originSeqNo;
	time_hl = hdr->thl;
		
	payload1 = call Packet.getPayload(msg, call Packet.payloadLength(aggMsgPtr));
	pkt = (ECollMessage*) payload1;	
	dest = call UnicastNameFreeRouting.nextHop();

	AGGMSGS=0;
	payload_agg = payload_agg_init;
	
	call SubPacket.setPayloadLength(reTxMsgPtr, aggDataLen );
    memcpy(reTxMsgPtr, aggMsgPtr, sizeof(message_t));	
    payload_agg = payload_agg_init;    
	
	total_pkt_sent++;
	
    if (error != SUCCESS && error != ENOACK)
    {
      // The radio wasn't able to send the packet: retransmit it.
      hasReTxData = TRUE;    
      clearState(SENDING);
      
      call CollectionDebug.logEventMsg(NET_C_FE_SENDDONE_FAIL, 
				       call CollectionPacket.getSequenceNumber(msg), 
				       call CollectionPacket.getOrigin(msg), 
				       call AMPacket.destination(msg));

      //printf("Forwarder: send failed\n");
      //printfflush();
            
    }
    //else if (hasState(ACK_PENDING) && !call PacketAcknowledgements.wasAcked(msg))
    //else if (error != SUCCESS && error == ENOACK)
    else if (error == ENOACK)
    {    	
      // if re-tries are > 0
      //   if CAP is running
      //		start re-Tx timer  			
      //   else 	 	 
      //		if re_tries == MAXRETS 
      //			wait for next CAP
      //		else
      //			tried re_Tx some times but CAP is over, discard the packet
      // else
      //   tried re_Tx MAXRETS times, no success, discard the packet
      	  		
      /* No ack: if countdown is not 0, retransmit, else drop the packet. */
      call LinkEstimator.txNoAck(call AMPacket.destination(msg));
      call CtpInfo.recomputeRoutes();      
	  hasReTxData = TRUE;
      
      total_rets++;
      
       
      // Check about the ETX,
      // If that is greater than LAST_NORMAL_ETX + BETA
      // 	if we have a best neighbor
      // 		post the Route update task
      // 	else 
      //		send the Beacon with CTP_UN_HEALTHY 
      
      call CtpInfo.getEtx(&gradient);
      
      if ( gradient >= LAST_NORMAL_ETX + BETA ){
		
		//printf("FE  Gradient above threshold \n");
		//printfflush();
    
      	if (bn_id > 0 && bn_id < 1000){  					// If we have a valid best neighbor   
      	
	        clearState(SENDING); 
		    clearState(WAIT_RETX);      	
      		call CtpInfo.triggerImmediateRouteUpdate(1);    // Code for immidiate parent update  
      		 	
      	} else {

	        clearState(SENDING); 
		    clearState(WAIT_RETX);      	
      		call CtpInfo.triggerImmediateRouteUpdate(2);    // Code for immiddaite beacon sending      		

      	}	   
      	
		hasReTxData = FALSE;
		payload_aggtemp = payload_aggtemp_init;	
		payload_reTx = payload_reTx_init;
		
		call SubPacket.clear(reTxMsgPtr);
		call SubPacket.clear(aggMsgPtr);
		
		AGGMSGS = 0;
      	
      	//aggpacketComplete(msg, FALSE);
      	
      	   	
      } 
      else {  //gradient is less then allowable limit (normal case)
     	      
	      // If we have a increasing ETX trend
	      if (total_rets  > 3) {
	      	doing_retries = TRUE;
	      }
	      
	      if (re_tries > 0 )   // First re-Tx from the timer Capisrunning
	      { 
	      
	        call CtpInfo.getEtx(&etx_value);  
	      
	        //printf("Retx:  re_tries  %i  , etx   %i\n",re_tries , etx_value);
	        //printfflush();
	
	        call CollectionDebug.logEventMsg(NET_C_FE_SENDDONE_WAITACK, 
						 call CollectionPacket.getSequenceNumber(msg), 
						 call CollectionPacket.getOrigin(msg), 
	                                         call AMPacket.destination(msg));
	        
	        if ( canReTX == TRUE ) {       // CAP is running
				
		   		// NodeID	Event   Time	ParentID    ParentETX    BestNeighborID     BestNeighborETX   SourceID    Seqno   THL  retries   
				/* printf("%i  %i  %i  %i  %i  %i %i  %i  %i  %i  %i\n",TOS_NODE_ID , 311 ,  */
				/* 	dest, etx_value, bn_id, bn_etx_value, org , pkt->seqno, time_hl ,beacon_cnt,total_pkt_sent); */
				/* printfflush();    	    	 */
				
	        	re_tries = re_tries -1;  // Trying  once again
	        	startRetxmitTimer(SENDDONE_NOACK_WINDOW, SENDDONE_NOACK_OFFSET);
	
	        } else {                     // CAP is not running
				
				if (re_tries == MAXRETS){	// We have not re-Tx yet and CAP has not started either (Still in the CFP of the same SF)
			
		   		    /// NodeID	Event   Time	ParentID    ParentETX    BestNeighborID     BestNeighborETX   SourceID    Seqno   THL  retries   
					/* printf("%i  %i  %i  %i  %i  %i %i  %i  %i %i %i\n",TOS_NODE_ID , 311 ,  */
					/* 	dest, etx_value, bn_id, bn_etx_value, org , pkt->seqno, time_hl ,beacon_cnt,total_pkt_sent); */
					/* printfflush();    	    	 */
					
					hasReTxData = TRUE;
	        		clearState(SENDING);
	        		setState(WAIT_RETX);
	        
	        	} else{  // if (re_tries < MAXRETS) {                   // We have tried some times (not MAXRETS times) but our time is over (CAP is over)
	   							
				    hasReTxData = FALSE;	   // Indicate that there is no data for re-Tx	
					payload_agg = payload_agg_init;
			    
			        clearState(SENDING); 
			        clearState(WAIT_RETX);       
			        re_tries = MAXRETS;
			        
					printf("%i  %i  %i  %i  %i  %i %i  %i  %i  %i  %i \n",TOS_NODE_ID , 322 , 
						dest, etx_value, bn_id, bn_etx_value, org , pkt->seqno, time_hl ,beacon_cnt,total_pkt_sent);
			        printfflush();    	    	

			        payload_agg = payload_agg_init;
			        payload_reTx = payload_reTx_init;
		
			   		aggpacketComplete(msg, FALSE);        		
	        	}
	
	        }
	              
	      }  else {      
				/* Hit max retransmit threshold: drop the packet. */
				//AGGMSGS=0;
		        hasReTxData = FALSE;	   // Indicate that there is no data for re-Tx
				payload_agg = payload_agg_init;
		        clearState(SENDING);    
		        clearState(WAIT_RETX);    
		        re_tries = MAXRETS;
		
				printf("%i  %i  %i  %i  %i  %i %i  %i  %i  %i  %i\n",TOS_NODE_ID , 333 , 
					dest, etx_value, bn_id, bn_etx_value, org , pkt->seqno, time_hl ,beacon_cnt,total_pkt_sent);
		        printfflush();    	    	
	
	
		        //printf("Failed_retransmission_____discarding_packet \n");
		        //printfflush();
				
				//doing_retries = FALSE;
				//total_rets = 0;
				
		        payload_agg = payload_agg_init;
		        payload_reTx = payload_reTx_init;
				
				//call SubPacket.clear(reTxMsgPtr);
		   		aggpacketComplete(msg, FALSE);
	      }
	      
	    }
    
    }
    else // We sent the packet successfully 
    {
      call Leds.led1Toggle();	
      hasReTxData = FALSE;	   // Indicate that there is no data for re-Tx
      clearState(SENDING);
      re_tries = MAXRETS;
      
      doing_retries = FALSE;
      total_rets = 0;
      
      call LinkEstimator.txAck(call AMPacket.destination(msg));
      call CtpInfo.getEtx(&gradient);
      
      LAST_NORMAL_ETX = gradient; 

      payload_agg = payload_agg_init;
      payload_reTx = payload_reTx_init;

      //call SubPacket.clear(reTxMsgPtr);
      aggpacketComplete(msg, TRUE);

	  
      printf("T 222\n");
      printfflush();
		
      AGGMSGS = 0;
    } 
  }

  /*
   * Function for preparing a packet for forwarding. Performs
   * a buffer swap from the message pool. If there are no free
   * message in the pool, it returns the passed message and does not
   * put it on the send queue.
   */

  message_t* ONE forward(message_t* ONE_NOK m) {

    uint16_t org,sqn, time_hl;
    uint8_t* payload = NULL;
    uint16_t gradient;
    
	am_addr_t* bn_id;
    call CtpInfo.getBestNeighborID(&bn_id);    
    
      	
	AGGMSGS++;
  	payload = call SubPacket.getPayload(m, sizeof(ctp_data_header_t) + MSGLEN);
	
	//printf("SF_portion in forward  %i\n",SF_portion);
	//printfflush();
	
	if (SF_portion == before_TS){
		memcpy(payload_lside, payload , sizeof(ctp_data_header_t) + MSGLEN );
		payload_lside = payload_lside + sizeof(ctp_data_header_t) + MSGLEN;
	}

	if (SF_portion == after_TS){
		memcpy(payload_rside, payload , sizeof(ctp_data_header_t) + MSGLEN );
		payload_rside = payload_rside + sizeof(ctp_data_header_t) + MSGLEN;
	}
	
	//memcpy(payload_agg, payload , sizeof(ctp_data_header_t) + MSGLEN );
	//payload_agg = payload_agg + sizeof(ctp_data_header_t) + MSGLEN;
			
    //printf("FE:  fwd psize %i\n", AGGMSGS);
    //printfflush();

    // Loop-detection code:
    if (call CtpInfo.getEtx(&gradient) == SUCCESS) {
      if (call CtpPacket.getEtx(m) <= gradient) {   // ETX of child is less than own ETX
      	
      	
      	// If, the loop was due to repeated no ACK
      	if (doing_retries == TRUE){ 
 
	       	//printf("FE loop detected due to increasing Etx trend \n");
    	  	//printfflush();
      	
	      	if (bn_id > 0 && bn_id < 1000){  					// If we have a valid best neighbor   
	      		call CtpInfo.triggerImmediateRouteUpdate(1);    // Code for immidiate parent update   	
	      	} else {
	      		call CtpInfo.triggerImmediateRouteUpdate(2);    // Code for immiddaite beacon sending      		
	      	}	   
	      	
			hasReTxData = FALSE;
			payload_aggtemp = payload_aggtemp_init;	
			payload_reTx = payload_reTx_init;
			
			call SubPacket.clear(reTxMsgPtr);
			call SubPacket.clear(aggMsgPtr);
			
			AGGMSGS = 0;
		
		} else {   // else, the loop was due to our old ETX information at Child (Our parent updated us but we could not update our child)

	       	//printf("FE loop detected due to old Etx update \n");
    	  	//printfflush();
		
			call CtpInfo.triggerImmediateRouteUpdate(0);
		}
        

	     
      }
    }
        // Successful function exit point:
      return m; //newMsg;
  }

  /*
   * Received a message to forward. Check whether it is a duplicate by
   * checking the packets currently in the queue as well as the 
   * send history cache (in case we recently forwarded this packet).
   * The cache is important as nodes immediately forward packets
   * but wait a period before retransmitting after an ack failure.
   * If this node is a root, signal receive.
   */ 

  event message_t* 
  SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
  	
  	ECollMessage* pkt;    
    
    uint8_t i, thl;
    uint8_t aggMsgs = call CtpPacket.getNumber(msg);
	collection_id_t collectid;
    bool duplicate = FALSE;
    uint16_t gradient;
	
    uint16_t org,sqn, time_hl;
    am_addr_t dest;
	
    uint32_t Tcurr, Tdiff;
	
	//printf("Msg rxed \n");
	//printfflush();
	
	/* if (!call RootControl.isRoot()){ */
	/* 	printf("aggMsgs in Rx.rx is  %i \n",aggMsgs); */
	/* 	printfflush(); */
	/* } */
    
	// Setting the message length
	call SubPacket.setPayloadLength(aggtempMsgPtr, aggDataLen );
	call SubPacket.setPayloadLength(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN );
	
	// Copying the data and getting the pointer to payload region
	memcpy(aggtempMsgPtr, msg, sizeof(message_t));			
	payload_aggtemp= call SubPacket.getPayload(aggtempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
	
	payload_aggtemp_init = payload_aggtemp;

	for (i = aggMsgs + 1; i>0; i--){
		payload_temp_init = payload_temp;
		payload_temp = call SubPacket.getPayload(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
		memcpy(payload_temp , payload_aggtemp , ctp_data_frame_len);
		
		hdr = getHeader(tempMsgPtr);
		org = hdr->origin;
		sqn = hdr->originSeqNo;
		time_hl = hdr->thl;
		
   	    if ( org == 0 || org > 100){  
   	    
   	    	printf("W_the_F ........ \n"); 
   	    	printfflush(); 
   	    	break; 
   	    
   	    } else{
			dest = call UnicastNameFreeRouting.nextHop();	    
		    collectid = call CtpPacket.getType(tempMsgPtr);	
	
		    thl = call CtpPacket.getThl(tempMsgPtr);
		    thl++;		
			call CtpPacket.setThl(tempMsgPtr, thl);
		
		    if (len > call SubSend.maxPayloadLength()) {
		    	
		    	//printf("In SubReceive.receive, len > call SubSend.maxPayloadLength()  \n");
		    	//printfflush();
		    	break;
		        return msg;
		        
		    }
		
		    //See if we remember having seen this packet
		    //We look in the sent cache ...
		    /**if (call SentCache.lookup(tempMsgPtr)) {
		        call CollectionDebug.logEvent(NET_C_FE_DUPLICATE_CACHE);
		        
		        if (i == 1) {
			    	payload_temp_init  = payload_temp;
			    	payload_aggtemp = payload_aggtemp_init;    	
		        	return tempMsgPtr;
		        } else {
		        	continue;
		        }
		        
		    }
		    //... and in the queue for duplicates
		    if (call SendQueue.size() > 0) {
		      for (i = call SendQueue.size(); i >0; i--) {
				qe_rx = call SendQueue.element(i-1);
				if (call CtpPacket.matchInstance(qe_rx->msg, tempMsgPtr)) {
			  		duplicate = TRUE;
			  		break;
				}
		      }
		    }
		    
		    if (duplicate) {
		        call CollectionDebug.logEvent(NET_C_FE_DUPLICATE_QUEUE);
	
		        if (i == 1) {
			    	payload_temp_init  = payload_temp;
			    	payload_aggtemp = payload_aggtemp_init;    	
		        	return tempMsgPtr;
		        } else {
		        	continue;
		        }
	
		    }**/
		
		    // If I'm the root, signal receive. 
		    else if (call RootControl.isRoot()) {		
			   
			   payload = call Packet.getPayload(tempMsgPtr, call Packet.payloadLength(tempMsgPtr));	
			   pkt = (ECollMessage*) payload; 	
			   
		       //printf("%s%i  %s%i  %s%i  %s%i  %s%i T\n","Success_rx_from_node ", org ,"to_node ", TOS_NODE_ID,"SeqNo ", sqn, "time_to_live " , time_hl, "AL_seq_no " , pkt->seqno);
		       //printfflush();    	    	
			
			   // Time	  NodeID	Event    SourceID    Seqno   THL 
			   //printf("%i  %i   T  %i  %i  %i %i  %i \n", TOS_NODE_ID, 1 , org , pkt->seqno, pkt->data[0], pkt->data[1], time_hl);
			   //printfflush();    	    	
				
				Tcurr = call LocalTime.get();
				Tdiff = Tcurr - Tprev[org-6];
				Tprev[org-6] = Tcurr;
	
				printf("T %i %i %i %lu\n", org, pkt->seqno, time_hl, Tdiff);
				printfflush(); 
		       
		       if ( org == 0 || org > 100){  return msg; }
		       
			   if (i == 1 ){	    // The last of the aggregated messages
		
			    	payload_temp_init  = payload_temp;
			    	payload_aggtemp = payload_aggtemp_init;
		
			      return signal Receive.receive[collectid](tempMsgPtr, 
								       call Packet.getPayload(tempMsgPtr, call Packet.payloadLength(tempMsgPtr)), 
								       call Packet.payloadLength(tempMsgPtr));
			   } else {
					
					signal Receive.receive[collectid](tempMsgPtr, 
								       call Packet.getPayload(tempMsgPtr, call Packet.payloadLength(tempMsgPtr)), 
								       call Packet.payloadLength(tempMsgPtr));
			   }
							       					       
		   	}
		   	
		   	else if (isController){
	
			    	payload_temp_init  = payload_temp;
			    	payload_aggtemp = payload_aggtemp_init;			  
					
					//printf("FE this is super-relay \n");
					//printfflush();
					
					return signal Receive.receive[collectid](tempMsgPtr, 
								       call Packet.getPayload(tempMsgPtr, call Packet.payloadLength(tempMsgPtr)), 
								       call Packet.payloadLength(tempMsgPtr));	   		
		   	
		   	}
		    // I'm on the routing path and Intercept indicates that I
		    // should not forward the packet.
		    else if (!signal Intercept.forward[collectid](tempMsgPtr, 
								  call Packet.getPayload(tempMsgPtr, call Packet.payloadLength(tempMsgPtr)), 
								  call Packet.payloadLength(tempMsgPtr))){ 	
								  
		    	payload_temp_init  = payload_temp;
		    	payload_aggtemp = payload_aggtemp_init;			  
								  
		    	return tempMsgPtr;
		    }
		    else {
			       /* printf("%i  %i  %i %i \n", org ,TOS_NODE_ID, pkt->seqno, time_hl); */
			       /* printfflush();    	 */
			       
			       if (route_found == TRUE){	   // Only process the message if the route is found	       
	  				   
	  				   if ( org == 0 || org > 100){  return msg; }
									       
				       if (i==1 ){
				    		payload_temp_init  = payload_temp;
					    	payload_aggtemp = payload_aggtemp_init;			       	
				       		return forward(tempMsgPtr);
					   } else {
					   		forward(tempMsgPtr);
					   }
					   
					} else { return msg; }
					
				   
		    }  // End of the If statement
	    	
	    	// Move the pointer
	    	if (route_found == TRUE){
				payload_aggtemp = payload_aggtemp + ctp_data_frame_len;
			}
	    	
    	}  // Sanity check IF
    }  
       // End of the for loop        
       payload_aggtemp = payload_aggtemp_init;
       
       return msg;
        
  } // End of the function Subreceive.receive


  event message_t* 
  SubSnoop.receive(message_t* msg, void *payload, uint8_t len) {
    // Check for the pull bit (P) [TEP123] and act accordingly.  This
    // check is made for all packets, not just ones addressed to us.

	/*hdr = getHeader(msg);    
	uint16_t org = hdr->origin;
	uint16_t sqn = hdr->originSeqNo;
	uint16_t time_hl = hdr->thl;	

    printf("%s%i  %s%i  %s%i %s%i  T","Success_sn_from_node ", org ,"to_node ", TOS_NODE_ID, "SeqNo ", sqn, "time_to_live " , time_hl);
    printfflush();    	*/
    

    if (call CtpPacket.option(msg, CTP_OPT_PULL)) {
    	
    	//printf("Routing_info_requested \n");
        //printfflush();    	
    	
      call CtpInfo.triggerRouteUpdate();
      
    }


//    printf("Routing infor requested \n");
 //   printfflush();    	
    
    
    return signal Snoop.receive[call CtpPacket.getType(msg)] 
      (msg, payload + sizeof(ctp_data_header_t), 
       len - sizeof(ctp_data_header_t));
  }
  
  event void RetxmitTimer.fired() {
    clearState(SENDING);
		    
    //printf("Retxmittimer.fired:  posted sendtask.\n");
    //printfflush();
	 if ( canReTX == TRUE ) { 
    	payloadLen = call SubPacket.payloadLength(reTxMsgPtr);      
    	dest = call UnicastNameFreeRouting.nextHop();    
    	call SubSend.send(dest, reTxMsgPtr, payloadLen);
    	setState(SENDING);
     }
    //post sendTask();
  }

  command bool CtpCongestion.isCongested() {
    return FALSE;
  }

  command void CtpCongestion.setClientCongested(bool congested) {
    // Do not respond to congestion.
  }
  
  /* signalled when this neighbor is evicted from the neighbor table */
  event void LinkEstimator.evicted(am_addr_t neighbor) {}

  
  // Packet ADT commands
  command void Packet.clear(message_t* msg) {
    call SubPacket.clear(msg);
  }

  command uint8_t Packet.payloadLength(message_t* msg) {
    return call SubPacket.payloadLength(msg) - sizeof(ctp_data_header_t);
  }

  command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
    call SubPacket.setPayloadLength(msg, len + sizeof(ctp_data_header_t));
  }
  
  command uint8_t Packet.maxPayloadLength() {
    return call SubPacket.maxPayloadLength() - sizeof(ctp_data_header_t);
  }

  command void* Packet.getPayload(message_t* msg, uint8_t len) {
    uint8_t* payload = call SubPacket.getPayload(msg, len + sizeof(ctp_data_header_t));
    if (payload != NULL) {
      payload += sizeof(ctp_data_header_t);
    }
    return payload;
  }

  // CollectionPacket ADT commands
  command am_addr_t       CollectionPacket.getOrigin(message_t* msg) {return getHeader(msg)->origin;}
  command collection_id_t CollectionPacket.getType(message_t* msg) {return getHeader(msg)->type;}
  command uint8_t         CollectionPacket.getSequenceNumber(message_t* msg) {return getHeader(msg)->originSeqNo;}
  command void CollectionPacket.setOrigin(message_t* msg, am_addr_t addr) {getHeader(msg)->origin = addr;}
  command void CollectionPacket.setType(message_t* msg, collection_id_t id) {getHeader(msg)->type = id;}
  command void CollectionPacket.setSequenceNumber(message_t* msg, uint8_t _seqno) {getHeader(msg)->originSeqNo = _seqno;}

  // CtpPacket ADT commands
  command uint8_t       CtpPacket.getType(message_t* msg) {return getHeader(msg)->type;}
  command am_addr_t     CtpPacket.getOrigin(message_t* msg) {return getHeader(msg)->origin;}
  command uint16_t      CtpPacket.getEtx(message_t* msg) {return getHeader(msg)->etx;}
  command uint8_t       CtpPacket.getSequenceNumber(message_t* msg) {return getHeader(msg)->originSeqNo;}
  command uint8_t       CtpPacket.getThl(message_t* msg) {return getHeader(msg)->thl;}
  command void CtpPacket.setThl(message_t* msg, uint8_t thl) {getHeader(msg)->thl = thl;}
  command void CtpPacket.setOrigin(message_t* msg, am_addr_t addr) {getHeader(msg)->origin = addr;}
  command void CtpPacket.setType(message_t* msg, uint8_t id) {getHeader(msg)->type = id;}
  command void CtpPacket.setEtx(message_t* msg, uint16_t e) {getHeader(msg)->etx = e;}
  command void CtpPacket.setSequenceNumber(message_t* msg, uint8_t _seqno) {getHeader(msg)->originSeqNo = _seqno;}
  
  command bool CtpPacket.option(message_t* msg, ctp_options_t opt) {
  	return ( ( (getHeader(msg)->options & CTP_PULL_ECN  ) & opt ) == opt) ? TRUE : FALSE;
    //return ((getHeader(msg)->options & opt) == opt) ? TRUE : FALSE;
  }
  command void CtpPacket.setOption(message_t* msg, ctp_options_t opt) {
    getHeader(msg)->options |= opt;
  }
  command void CtpPacket.clearOption(message_t* msg, ctp_options_t opt) {
    getHeader(msg)->options &= ~opt;
  }

 command void CtpPacket.setNumber(message_t* msg, uint8_t agg) {
    getHeader(msg)->options = agg;
  }

 command uint8_t CtpPacket.getNumber(message_t* msg) {
   return getHeader(msg)->options &= CTP_HOW_MANY ;
  }


  // A CTP packet ID is based on the origin and the THL field, to
  // implement duplicate suppression as described in TEP 123.
  command bool CtpPacket.matchInstance(message_t* m1, message_t* m2) {
    return (call CtpPacket.getOrigin(m1) == call CtpPacket.getOrigin(m2) &&
	    call CtpPacket.getSequenceNumber(m1) == call CtpPacket.getSequenceNumber(m2) &&
	    call CtpPacket.getThl(m1) == call CtpPacket.getThl(m2) &&
	    call CtpPacket.getType(m1) == call CtpPacket.getType(m2));
  }

  command bool CtpPacket.matchPacket(message_t* m1, message_t* m2) {
    return (call CtpPacket.getOrigin(m1) == call CtpPacket.getOrigin(m2) &&
	    call CtpPacket.getSequenceNumber(m1) == call CtpPacket.getSequenceNumber(m2) &&
	    call CtpPacket.getType(m1) == call CtpPacket.getType(m2));
  }


  void clearState(uint8_t state) {
    forwardingState = forwardingState & ~state;
  }
  bool hasState(uint8_t state) {
    return forwardingState & state;
  }
  void setState(uint8_t state) {
    forwardingState = forwardingState | state;
  }
  
  
  event void LinkEstimator.receivedMB(am_addr_t neighbor){ } //return SUCCESS; }
  
  
  /******** Defaults. **************/
   
  default event void
  Send.sendDone[uint8_t client](message_t *msg, error_t error) {
  }

  default event bool
  Intercept.forward[collection_id_t collectid](message_t* msg, void* payload, 
                                               uint8_t len) {
    return TRUE;
  }

  default event message_t *
  Receive.receive[collection_id_t collectid](message_t *msg, void *payload,
                                             uint8_t len) {
    return msg;
  }

  default event message_t *
  Snoop.receive[collection_id_t collectid](message_t *msg, void *payload,
                                           uint8_t len) {
    return msg;
  }

  default command collection_id_t CollectionId.fetch[uint8_t client]() {
    return 0;
  }
  
  /* Default implementations for CollectionDebug calls.
   * These allow CollectionDebug not to be wired to anything if debugging
   * is not desired. */
  
  default command error_t CollectionDebug.logEvent(uint8_t type) {
    return SUCCESS;
  }
  default command error_t CollectionDebug.logEventSimple(uint8_t type, uint16_t arg) {
    return SUCCESS;
  }
  default command error_t CollectionDebug.logEventDbg(uint8_t type, uint16_t arg1, uint16_t arg2, uint16_t arg3) {
    return SUCCESS;
  }
  default command error_t CollectionDebug.logEventMsg(uint8_t type, uint16_t msg, am_addr_t origin, am_addr_t node) {
    return SUCCESS;
  }
  default command error_t CollectionDebug.logEventRoute(uint8_t type, am_addr_t parent, uint8_t hopcount, uint16_t metric) {
    return SUCCESS;
  }
   
}
