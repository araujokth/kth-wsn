/* $Id: LinkEstimatorP.nc,v 1.17 2010-06-29 22:07:50 scipio Exp $ */
/*
 * Copyright (c) 2006 University of Southern California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 @ author Omprakash Gnawali
 @ Created: April 24, 2006
 */

#include "LinkEstimator.h"
#include "printf.h"
#include "/home/anser/local/src/tinyos-2.x/apps/EasyCollection/Sink/EColl.h"

module LinkEstimatorP {
  provides {
    interface StdControl;
    interface AMSend as Send;
    interface Receive;
    interface LinkEstimator;
    interface Init;
    interface Packet;
    interface CompareBit;
  }

  uses {
    interface AMSend;
    interface AMPacket as SubAMPacket;
    interface Packet as SubPacket;
    interface Receive as SubReceive;
    interface LinkPacketMetadata;
    interface Random;
	interface Capisrunning;    
	interface HasTimeSlot;
  }
}

implementation {

  // configure the link estimator and some constants
  enum {
    // If the eetx estimate is below this threshold
    // do not evict a link
    EVICT_EETX_THRESHOLD = 55,
    // maximum link update rounds before we expire the link
    MAX_AGE = 6,
    // if received sequence number if larger than the last sequence
    // number by this gap, we reinitialize the link
    MAX_PKT_GAP = 10,
    BEST_EETX = 0,
    INVALID_RVAL = 0xff,
    INVALID_NEIGHBOR_ADDR = 0xff,
  
    // if we don't know the link quality, we need to return a value so
    // large that it will not be used to form paths
    VERY_LARGE_EETX_VALUE = 0xff,
  
    // decay the link estimate using this alpha
    // we use a denominator of 10, so this corresponds to 0.1
    ALPHA_BEACON = 5,
  
    // decay the link estimate using this alpha
    // we use a denominator of 10, so this corresponds to 0.1
    ALPHA = 5,
  
    // number of packets to wait before computing a new
    // DLQ (Data-driven Link Quality)
    DLQ_PKT_WINDOW = 3,
    
    // number of beacons to wait before computing a new
    // BLQ (Beacon-driven Link Quality)
    BLQ_PKT_WINDOW = 2,
    
    // Factor used in filter
    FILT_FACTOR = (10 - ALPHA)*5,
    
    // largest EETX value that we feed into the link quality EWMA
    // a value of 60 corresponds to having to make six transmissions
    // to successfully receive one acknowledgement
    LARGE_EETX_VALUE = 60
  };

  // keep information about links from the neighbors
  neighbor_table_entry_t NeighborTable[NEIGHBOR_TABLE_SIZE];
  // link estimation sequence, increment every time a beacon is sent
  uint8_t linkEstSeq = 0;
  // if there is not enough room in the packet to put all the neighbor table
  // entries, in order to do round robin we need to remember which entry
  // we sent in the last beacon
  uint8_t prevSentIdx = 0;
 
  am_addr_t current_parent;			  // Address of the current parent 
  am_addr_t child_addr;               // Address of the children who has send the modified beacon
  am_addr_t CONST_NODE_ID = 1000;     // Identifies that this node has alredy sent the first modified beacon
  
  bool mod_beacon_to_post = FALSE;    // IS there ant modified beacon to be sent, checked at the start of the CAP interval
  bool last_sent_beacon = FALSE;   	  // What was the last sent beacon, FALSE means normal beacon, TRUE means modified beacon, checked at AMSend.sendDone
  bool last_rx_beacon_MB = FALSE;     // What was the last received beacon, FALSE means normal beacon, TRUE means modified beacon
  bool nb_sending = FALSE;            // TRUE when sending the normal beacon 
  bool first_mb_sent = FALSE;		  // Controls the ID field in the tuple of a modified beacon, 
  									  // ID is TOS_NODE_ID if first_mb_sent is FALSE, CONST_NODE_ID is first_mb_sent is TRUE

  bool cp_beacon_re = TRUE;	          // This indicates that whether this received beacon can be passed to RE, 
  									  //not if this is from coord and does ot have our Node ID in footer
  											
  
  
  uint8_t retries;
  uint8_t no_of_messages = 1;
  
  uint8_t MAX_MOD_BEACON_SIZE = 100;
  uint8_t MAX_BEACON_RETS = 5;
  uint8_t msgToMAClen = 0;
  
  uint8_t hop_nodes[6][15]={0};    	  // rows keeps the nodes with same hop_count, column for different hop_counts
  uint8_t hop1_nodes_idx[6] = {0};	  // keeps track of nodes at specific hop_count (count of nodes at a specific hop-count)
  uint8_t Schedule[32] = {0};					  
  
  error_t eval;
  
  message_t modified_Beacon;
  message_t* ONE_NOK modified_Beacon_Ptr;  
  
  message_t modified_ReTX_Beacon;
  message_t* ONE_NOK modified_ReTX_Beacon_Ptr;  
  
  //message_t message_to_MAC;
  //message_t* ONE_NOK message_to_MAC_Ptr;  
      
  
  ///////////////////////////
  
  //uint8_t* payload_copy = NULL;
  //uint8_t* payload_copy_init = NULL;
    
  uint8_t* payload_mb = NULL;
  uint8_t* payload_mb_init = NULL;
  
  //uint8_t* payload_msg_to_MAC = NULL;
  //uint8_t* payload_msg_to_MAC_init = NULL;
  
  uint8_t* payload_retx_mb = NULL;
  uint8_t* payload_retx_mb_init = NULL;

  ////////////////////////////
  
    
  topology_info_t* tuple;
  topology_info_t* tuple_rx;
  topology_info_header_t* mod_hdr;
  
  void process_before_MAC();
  void clear_schedule();
  uint8_t getBranchOffset(uint8_t);
  uint8_t search_Schedule(am_addr_t);
  
  task void re_init_msg();
  
  // get the link estimation header in the packet
  linkest_header_t* getHeader(message_t* m) {
    return (linkest_header_t*)call SubPacket.getPayload(m, sizeof(linkest_header_t));
  }

  // get the link estimation footer (neighbor entries) in the packet
  linkest_footer_t* getFooter(message_t* ONE m, uint8_t len) {
    // To get a footer at offset "len", the payload must be len + sizeof large.
    return (linkest_footer_t* ONE)(len + (uint8_t *)call Packet.getPayload(m,len + sizeof(linkest_footer_t)));
  }
  
  // get the pointer to <NodeID,Hopcount> ith tuple
  topology_info_t* getTuple(message_t* m , uint8_t idx ) {
    return (topology_info_t* ONE)( sizeof(topology_info_header_t) + 3*idx + (uint8_t *)call SubPacket.getPayload( m , sizeof(topology_info_t) ) );
  }

  // gets the pointer to the header of modified beacon
  topology_info_header_t* getModHeader(message_t* m) {
    return (topology_info_header_t*)call SubPacket.getPayload(m, sizeof(topology_info_header_t));
  }  	
  	
  // add the link estimation header (seq no) and link estimation
  // footer (neighbor entries) in the packet. Call just before sending
  // the packet.
  uint8_t addLinkEstHeaderAndFooter(message_t * ONE msg, uint8_t len) {
    uint8_t newlen;
    linkest_header_t *hdr;
    linkest_footer_t *footer;
    uint8_t i, j, k;
    uint8_t maxEntries, newPrevSentIdx;
    dbg("LI", "newlen1 = %d\n", len);
    hdr = getHeader(msg);
    footer = getFooter(msg, len);

    maxEntries = ((call SubPacket.maxPayloadLength() - len - sizeof(linkest_header_t))
		  / sizeof(linkest_footer_t));

    // Depending on the number of bits used to store the number
    // of entries, we can encode up to NUM_ENTRIES_FLAG using those bits
    if (maxEntries > NUM_ENTRIES_FLAG) {
      maxEntries = NUM_ENTRIES_FLAG;
    }
    dbg("LI", "Max payload is: %d, maxEntries is: %d\n", call SubPacket.maxPayloadLength(), maxEntries);

    j = 0;
    newPrevSentIdx = 0;
    for (i = 0; i < NEIGHBOR_TABLE_SIZE && j < maxEntries; i++) {
      uint8_t neighborCount;
      neighbor_stat_entry_t * COUNT(neighborCount) neighborLists;
      if(maxEntries <= NEIGHBOR_TABLE_SIZE)
        neighborCount = maxEntries;
      else
        neighborCount = NEIGHBOR_TABLE_SIZE;
      
      neighborLists = TCAST(neighbor_stat_entry_t * COUNT(neighborCount), footer->neighborList);

      k = (prevSentIdx + i + 1) % NEIGHBOR_TABLE_SIZE;
      if ((NeighborTable[k].flags & VALID_ENTRY) &&
	  (NeighborTable[k].flags & MATURE_ENTRY)) {
	neighborLists[j].ll_addr = NeighborTable[k].ll_addr;
	neighborLists[j].inquality = NeighborTable[k].inquality;
	newPrevSentIdx = k;
	
	//printf("LE T Loaded on footer: %d %d %d \n", j, neighborLists[j].ll_addr, neighborLists[j].inquality);
	//printfflush();
	
	j++;
      }
    }
    prevSentIdx = newPrevSentIdx;

	//printf("\n");
	//printfflush();
	
    hdr->seq = linkEstSeq++;
    hdr->flags = 0;
    hdr->flags |= (NUM_ENTRIES_FLAG & j);
    newlen = sizeof(linkest_header_t) + len + j*sizeof(linkest_footer_t);
    dbg("LI", "newlen2 = %d\n", newlen);
    return newlen;
  }


  // initialize the given entry in the table for neighbor ll_addr
  void initNeighborIdx(uint8_t i, am_addr_t ll_addr) {
    neighbor_table_entry_t *ne;
    ne = &NeighborTable[i];
    ne->ll_addr = ll_addr;
    ne->lastseq = 0;
    ne->rcvcnt = 0;
    ne->failcnt = 0;
    ne->flags = (INIT_ENTRY | VALID_ENTRY);
    ne->inage = MAX_AGE;
    ne->outage = MAX_AGE;
    ne->inquality = 0;
    ne->outquality = 0;
    ne->eetx = 0;
  }

  // find the index to the entry for neighbor ll_addr
  uint8_t findIdx(am_addr_t ll_addr) {
    uint8_t i;
    for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
      if (NeighborTable[i].flags & VALID_ENTRY) {
	if (NeighborTable[i].ll_addr == ll_addr) {
	  return i;
	}
      }
    }
    return INVALID_RVAL;
  }

  // find an empty slot in the neighbor table
  uint8_t findEmptyNeighborIdx() {
    uint8_t i;
    for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
      if (NeighborTable[i].flags & VALID_ENTRY) {
      } else {
	return i;
      }
    }
      return INVALID_RVAL;
  }

  // find the index to the worst neighbor if the eetx
  // estimate is greater than the given threshold
  uint8_t findWorstNeighborIdx(uint8_t thresholdEETX) {
    uint8_t i, worstNeighborIdx;
    uint16_t worstEETX, thisEETX;

    worstNeighborIdx = INVALID_RVAL;
    worstEETX = 0;
    for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
      if (!(NeighborTable[i].flags & VALID_ENTRY)) {
	dbg("LI", "Invalid so continuing\n");
	continue;
      }
      if (!(NeighborTable[i].flags & MATURE_ENTRY)) {
	dbg("LI", "Not mature, so continuing\n");
	continue;
      }
      if (NeighborTable[i].flags & PINNED_ENTRY) {
	dbg("LI", "Pinned entry, so continuing\n");
	continue;
      }
      thisEETX = NeighborTable[i].eetx;
      if (thisEETX >= worstEETX) {
	worstNeighborIdx = i;
	worstEETX = thisEETX;
      }
    }
    if (worstEETX >= thresholdEETX) {
      return worstNeighborIdx;
    } else {
      return INVALID_RVAL;
    }
  }

  // update the quality of the link link: self->neighbor
  // this is found in the entries in the footer of incoming message
  void updateReverseQuality(am_addr_t neighbor, uint8_t outquality) {
    uint8_t idx;
    idx = findIdx(neighbor);
    if (idx != INVALID_RVAL) {
      NeighborTable[idx].outquality = outquality;
      NeighborTable[idx].outage = MAX_AGE;
    }
  }

  // update the EETX estimator
  // called when new beacon estimate is done
  // also called when new DEETX estimate is done
  void updateEETX(neighbor_table_entry_t *ne, uint16_t newEst) {
	
	uint16_t a;
	a = ne->eetx;

    ne->eetx = (ALPHA * ne->eetx + (10 - ALPHA) * newEst + FILT_FACTOR)/10;
    
    //printf("LE  updateEEtX  ne->eetx_before   %i ne->eetx_after   %i  newEst   %i \n", a, ne->eetx, newEst);
    //printfflush();
    
  }


  // update data driven EETX
  void updateDEETX(neighbor_table_entry_t *ne) {
    uint16_t estETX;

    if (ne->data_success == 0) {
      // if there were no successful packet transmission in the
      // last window, our current estimate is the number of failed
      // transmissions
      estETX = (ne->data_total - 1)* 10;
    } else {
      estETX = (10 * ne->data_total) / ne->data_success - 10;
      ne->data_success = 0;
      ne->data_total = 0;
    }
    
    //printf("LE   data driven Etx update estEtX  %i\n ",estETX);
    //printfflush();
    
    updateEETX(ne, estETX);
  }


  // EETX (Extra Expected number of Transmission)
  // EETX = ETX - 1
  // computeEETX returns EETX*10
  uint8_t computeEETX(uint8_t q1) {
    uint16_t q;
    if (q1 > 0) {
      q =  2550 / q1 - 10;
      if (q > 255) {
	q = VERY_LARGE_EETX_VALUE;
      }
      return (uint8_t)q;
    } else {
      return VERY_LARGE_EETX_VALUE;
    }
  }

  // BidirETX = 1 / (q1*q2)
  // BidirEETX = BidirETX - 1
  // computeBidirEETX return BidirEETX*10
  uint8_t computeBidirEETX(uint8_t q1, uint8_t q2) {
    uint16_t q;
    if ((q1 > 0) && (q2 > 0)) {
      q =  65025u / q1;
      q = (10*q) / q2 - 10;
      
      //printf("LE inq  %i  otq  %i  Birdir etx  %i\n",q1,q2,q);
     // printfflush();
      
      if (q > 255) {
		q = LARGE_EETX_VALUE;
      }
      
      return (uint8_t)q;
    
    } else {
      return LARGE_EETX_VALUE;
    }
  }

  // update the inbound link quality by
  // munging receive, fail count since last update
  void updateNeighborTableEst(am_addr_t n) {
    uint8_t i, totalPkt;
    neighbor_table_entry_t *ne;
    uint8_t newEst;
    uint8_t minPkt;

    minPkt = BLQ_PKT_WINDOW;
    dbg("LI", "%s\n", __FUNCTION__);
    for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
      ne = &NeighborTable[i];
      
      if (ne->ll_addr == n) {
      	
		if (ne->flags & VALID_ENTRY) {
	  
	  		if (ne->inage > 0)
	    		ne->inage--;
	  
	  		if (ne->outage > 0)
	  		  	ne->outage--;	 
	  		  	 
	  		if ((ne->inage == 0) && (ne->outage == 0)) {		
		    	ne->flags ^= VALID_ENTRY;
		    	ne->inquality = ne->outquality = 0;		  
		  } else {
		
		    dbg("LI", "Making link: %d mature\n", i);
		    ne->flags |= MATURE_ENTRY;
		    totalPkt = ne->rcvcnt + ne->failcnt;
		    dbg("LI", "MinPkt: %d, totalPkt: %d\n", minPkt, totalPkt);

		    if (totalPkt < minPkt) {
		      totalPkt = minPkt;
		    }

		    if (totalPkt == 0) {
		      ne->inquality = (ALPHA_BEACON * ne->inquality) / 10;
		    } else {
		      newEst = (255 * ne->rcvcnt) / totalPkt;
		      dbg("LI,LITest", "  %hu: %hhu -> %hhu", ne->ll_addr, ne->inquality, (ALPHA_BEACON * ne->inquality + (10-ALPHA_BEACON) * newEst + 5)/10);
		      ne->inquality = (ALPHA_BEACON * ne->inquality + (10-ALPHA_BEACON) * newEst + 5)/10;
		    }

		    ne->rcvcnt = 0;
		    ne->failcnt = 0;

		  }
		  
		  //printf("LE   beacon driven Etx update %i \n ",ne->ll_addr);
    	  //printfflush();
		  
		  updateEETX(ne, computeBidirEETX(ne->inquality, ne->outquality));
		}
		else {
	  	dbg("LI", " - entry %i is invalid.\n", (int)i);
		}
      }
    }
  }


  // we received seq from the neighbor in idx
  // update the last seen seq, receive and fail count
  // refresh the age
  void updateNeighborEntryIdx(uint8_t idx, uint8_t seq) {
    uint8_t packetGap;

    if (NeighborTable[idx].flags & INIT_ENTRY) {
      dbg("LI", "Init entry update\n");
      NeighborTable[idx].lastseq = seq;
      NeighborTable[idx].flags &= ~INIT_ENTRY;
    }
    
    packetGap = seq - NeighborTable[idx].lastseq;
    dbg("LI", "updateNeighborEntryIdx: prevseq %d, curseq %d, gap %d\n",
	NeighborTable[idx].lastseq, seq, packetGap);
    NeighborTable[idx].lastseq = seq;
    NeighborTable[idx].rcvcnt++;
    NeighborTable[idx].inage = MAX_AGE;
    if (packetGap > 0) {
      NeighborTable[idx].failcnt += packetGap - 1;
    }
    if (packetGap > MAX_PKT_GAP) {
      NeighborTable[idx].failcnt = 0;
      NeighborTable[idx].rcvcnt = 1;
      NeighborTable[idx].outage = 0;
      NeighborTable[idx].outquality = 0;
      NeighborTable[idx].inquality = 0;
    }

    if (NeighborTable[idx].rcvcnt >= BLQ_PKT_WINDOW) {
      updateNeighborTableEst(NeighborTable[idx].ll_addr);
    }

  }



  // print the neighbor table. for debugging.
  void print_neighbor_table() {
    uint8_t i;
    uint16_t A;
    neighbor_table_entry_t *ne;
    for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
      ne = &NeighborTable[i];
      //printf("Link layer address  \n");
      
      if (ne->flags & VALID_ENTRY) {
    /* dbg("LI,LITest", "%d:%d inQ=%d, inA=%d, outQ=%d, outA=%d, rcv=%d, fail=%d, biQ=%d\n",
	    i, ne->ll_addr, ne->inquality, ne->inage, ne->outquality, ne->outage,
	    ne->rcvcnt, ne->failcnt, computeBidirEETX(ne->inquality, ne->outquality)); */
	    
	    //printf("%s%i %s%i %i %i %i %i %i %i %i \n","Entry no: ", i,"Node ", ne->ll_addr, ne->inquality, ne->inage, ne->outquality, ne->outage,
	    //ne->rcvcnt, ne->failcnt, computeBidirEETX(ne->inquality, ne->outquality));
	    //printfflush();
      }
      
    }
    //printf("\n");
    //printfflush();
  }

  // print the packet. for debugging.
  void print_packet(message_t* msg, uint8_t len) {
    uint8_t i;
    uint8_t* b;

    b = (uint8_t *)msg->data;
    for(i=0; i<len; i++)
      dbg_clear("LI", "%x ", b[i]);
    dbg_clear("LI", "\n");
  }

  // initialize the neighbor table in the very beginning
  void initNeighborTable() {
    uint8_t i;

    for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
      NeighborTable[i].flags = 0;
    }
  }

  command error_t StdControl.start() {
    dbg("LI", "Link estimator start\n");
    return SUCCESS;
  }

  command error_t StdControl.stop() {
    return SUCCESS;
  }

  // initialize the link estimator
  command error_t Init.init() {
    dbg("LI", "Link estimator init\n");
  
    retries = MAX_BEACON_RETS;
  
    modified_Beacon_Ptr = &modified_Beacon;
    call SubPacket.setPayloadLength(modified_Beacon_Ptr, MAX_MOD_BEACON_SIZE );    
	payload_mb = call SubPacket.getPayload(modified_Beacon_Ptr, MAX_MOD_BEACON_SIZE);
	payload_mb_init = payload_mb;

    modified_ReTX_Beacon_Ptr = &modified_ReTX_Beacon;
    call SubPacket.setPayloadLength(modified_ReTX_Beacon_Ptr, MAX_MOD_BEACON_SIZE );    
	payload_retx_mb = call SubPacket.getPayload(modified_ReTX_Beacon_Ptr, MAX_MOD_BEACON_SIZE);
	payload_retx_mb_init = payload_retx_mb;

    /*message_to_MAC_Ptr = &message_to_MAC;  
    call SubPacket.setPayloadLength(message_to_MAC_Ptr, MAX_MOD_BEACON_SIZE );    
	payload_msg_to_MAC = call SubPacket.getPayload(message_to_MAC_Ptr, MAX_MOD_BEACON_SIZE);
	payload_msg_to_MAC_init = payload_msg_to_MAC;*/
           
	// Now fill the modified beacon with own message at the beginning
	/** Format is 
	 * <Number_of_message>-<NodeID,Hopcount>-<NodeID,Hopcount>-<NodeID,Hopcount> .... <NodeID,Hopcount>  
	 * **/
	mod_hdr = getModHeader(modified_Beacon_Ptr);
	mod_hdr->no_of_msgs = 1;
	
	//memcpy(payload_mb, mod_hdr , sizeof(topology_info_header_t) );
	//payload_mb = payload_mb + sizeof(topology_info_header_t);
	
	//tempMsg_Ptr = NULL;
	//SubPacket.clear(tempMsg_Ptr);
		
	tuple = getTuple(modified_Beacon_Ptr,0);
	tuple->addr = TOS_NODE_ID;
	tuple->hopcount = 0;
	
	//memcpy(payload_mb, tuple , sizeof(topology_info_t) );
	//payload_mb = payload_mb + sizeof(topology_info_t);
	
	// Also fill in the message to be sent to the MAC layer by the Sink node
	/** Format is
	 * <Number_of_overall_message>-<Number_of_messages_from_1st_branch>-<Messages_from_the_1st_branch> ... 
	 *  ... <Number_of_messages_from_nth_branch>-<Messages_from_the_nth_branch>
	 * **/
	//SubPacket.clear(tempMsg_Ptr);
	
	//mod_hdr = getModHeader(message_to_MAC_Ptr);
	//mod_hdr->no_of_msgs = 0;    // change this
	
	//memcpy(payload_msg_to_MAC, mod_hdr , sizeof(topology_info_header_t) );
	//payload_msg_to_MAC = payload_msg_to_MAC + sizeof(topology_info_header_t);
				
    initNeighborTable();
    return SUCCESS;
  }

   task void re_init_msg(){	
	
  	no_of_messages = 1;       // This means that from the next time we start from begginning
  	
	mod_hdr = getModHeader(modified_Beacon_Ptr);
	mod_hdr->no_of_msgs = no_of_messages;

	tuple = getTuple(modified_Beacon_Ptr,0);
	
	if (first_mb_sent == FALSE)
		tuple->addr = TOS_NODE_ID;
	else
		tuple->addr = CONST_NODE_ID;
			
	tuple->hopcount = 0; 	
	
 	
  }

	/******************************** This is my addition*************************************/

	async event void Capisrunning.Caphasstarted(uint32_t t0 , uint32_t dt){
		atomic{
				uint8_t payloadLen;
				if (mod_beacon_to_post == TRUE && nb_sending!= TRUE){
					
					mod_hdr = getModHeader(modified_Beacon_Ptr);
					mod_hdr->no_of_msgs = no_of_messages;
					
					last_sent_beacon = TRUE;
					payloadLen = call SubPacket.payloadLength(modified_Beacon_Ptr);
					
					
					memcpy(modified_ReTX_Beacon_Ptr, modified_Beacon_Ptr , sizeof(message_t) );
					
				  	eval = call AMSend.send(current_parent, modified_ReTX_Beacon_Ptr, payloadLen);

					//printf("LE: tx'ing modified beacon to %i  with length %i result%i \n",
				  	//current_parent, no_of_messages,eval);
					//printfflush();
				  	
				  	//mod_beacon_to_post = FALSE;
				}
		}		
	}
	async event void Capisrunning.Caphasfinished(){	}
	async event void Capisrunning.MyTShasStarted(){	}

	/******************************** This is my addition*************************************/



  // return bi-directional link quality to the neighbor
  command uint16_t LinkEstimator.getLinkQuality(am_addr_t neighbor) {
    uint8_t idx;
    idx = findIdx(neighbor);
    if (idx == INVALID_RVAL) {
      return VERY_LARGE_EETX_VALUE;
    } else {
      if (NeighborTable[idx].flags & MATURE_ENTRY) {
	return NeighborTable[idx].eetx;
      } else {
	return VERY_LARGE_EETX_VALUE;
      }
    }
  }

  // return the quality of the link: neighor->self
  command uint16_t LinkEstimator.getReverseQuality(am_addr_t neighbor) {
    uint8_t idx;
    idx = findIdx(neighbor);
    if (idx == INVALID_RVAL) {
      return VERY_LARGE_EETX_VALUE;
    } else {
      if (NeighborTable[idx].flags & MATURE_ENTRY) {
	return computeEETX(NeighborTable[idx].inquality);
      } else {
	return VERY_LARGE_EETX_VALUE;
      }
    }
  }

  // return the quality of the link: self->neighbor
  command uint16_t LinkEstimator.getForwardQuality(am_addr_t neighbor) {
    uint8_t idx;
    idx = findIdx(neighbor);
    if (idx == INVALID_RVAL) {
      return VERY_LARGE_EETX_VALUE;
    } else {
      if (NeighborTable[idx].flags & MATURE_ENTRY) {
	return computeEETX(NeighborTable[idx].outquality);
      } else {
	return VERY_LARGE_EETX_VALUE;
      }
    }
  }

  // insert the neighbor at any cost (if there is a room for it)
  // even if eviction of a perfectly fine neighbor is called for
  command error_t LinkEstimator.insertNeighbor(am_addr_t neighbor) {
    uint8_t nidx;

    nidx = findIdx(neighbor);
    if (nidx != INVALID_RVAL) {
      dbg("LI", "insert: Found the entry, no need to insert\n");
      return SUCCESS;
    }

    nidx = findEmptyNeighborIdx();
    if (nidx != INVALID_RVAL) {
      dbg("LI", "insert: inserted into the empty slot\n");
      initNeighborIdx(nidx, neighbor);
      return SUCCESS;
    } else {
      nidx = findWorstNeighborIdx(BEST_EETX);
      if (nidx != INVALID_RVAL) {
	dbg("LI", "insert: inserted by replacing an entry for neighbor: %d\n",
	    NeighborTable[nidx].ll_addr);
	signal LinkEstimator.evicted(NeighborTable[nidx].ll_addr);
	initNeighborIdx(nidx, neighbor);
	return SUCCESS;
      }
    }
    return FAIL;
  }

  // pin a neighbor so that it does not get evicted
  command error_t LinkEstimator.pinNeighbor(am_addr_t neighbor) {
    uint8_t nidx = findIdx(neighbor);
    if (nidx == INVALID_RVAL) {
      return FAIL;
    }
    NeighborTable[nidx].flags |= PINNED_ENTRY;
    return SUCCESS;
  }

  // pin a neighbor so that it does not get evicted
  command error_t LinkEstimator.unpinNeighbor(am_addr_t neighbor) {
    uint8_t nidx = findIdx(neighbor);
    if (nidx == INVALID_RVAL) {
      return FAIL;
    }
    NeighborTable[nidx].flags &= ~PINNED_ENTRY;
    return SUCCESS;
  }


  // called when an acknowledgement is received; sign of a successful
  // data transmission; to update forward link quality
  command error_t LinkEstimator.txAck(am_addr_t neighbor) {
    neighbor_table_entry_t *ne;
    uint8_t nidx = findIdx(neighbor);
    if (nidx == INVALID_RVAL) {
      return FAIL;
    }
    ne = &NeighborTable[nidx];
    ne->data_success++;
    ne->data_total++;
    
    //printf("LE  ne->data_success %i   ne->data_total %i \n",ne->data_success,ne->data_total);
    //printfflush();
    
    if (ne->data_total >= DLQ_PKT_WINDOW) {
      
      //printf("LE    updateDEEtX called \n");
      //printfflush();
      
      updateDEETX(ne);
    }
    return SUCCESS;
  }

	
  // called when an acknowledgement is not received; could be due to
  // data pkt or acknowledgement loss; to update forward link quality
  command error_t LinkEstimator.txNoAck(am_addr_t neighbor) {
    neighbor_table_entry_t *ne;
    uint8_t nidx = findIdx(neighbor);
    if (nidx == INVALID_RVAL) {
      return FAIL;
    }

    ne = &NeighborTable[nidx];
    ne->data_total++;
    
    //printf("LE inside LinkEstimator.txNoAck  %i \n",ne->data_total);
    //printfflush();
    
    if (ne->data_total >= DLQ_PKT_WINDOW) {
      
      //printf("LE txNoAck DLQ_Pkt_window expired \n");
      //printfflush();
      	
      updateDEETX(ne);
    }
    return SUCCESS;
  }

  // called when the parent changes; clear state about data-driven link quality
  command error_t LinkEstimator.clearDLQ(am_addr_t neighbor) {
    neighbor_table_entry_t *ne;
    uint8_t nidx = findIdx(neighbor);
    if (nidx == INVALID_RVAL) {
      return FAIL;
    }
    ne = &NeighborTable[nidx];
    ne->data_total = 0;
    ne->data_success = 0;
    return SUCCESS;
  }

  command error_t LinkEstimator.canTxMB(am_addr_t neighbor, bool FirstMbSent, bool reTxMB){
	uint8_t payloadLen;
  	
  	if (reTxMB == FALSE){
  		
	  	//printf("LE: cantxMB \n");
	  	//printfflush();
	  	
	  	current_parent = neighbor;  // This is the current parent  	
		mod_beacon_to_post = TRUE;  // Make it false in AMSend.sendDone
		first_mb_sent = FirstMbSent;
		
	} else {

	  	//printf("LE: re-tx cantxMB \n");
	  	//printfflush();
	  	
	  	current_parent = neighbor;  // This is the current parent  	
		mod_beacon_to_post = TRUE;  // Make it false in AMSend.sendDone
		
	  	no_of_messages = 1;       // This means that from the next time we start from begginning
  	
		mod_hdr = getModHeader(modified_Beacon_Ptr);
		mod_hdr->no_of_msgs = no_of_messages;

		tuple = getTuple(modified_Beacon_Ptr,0);
		tuple->addr = TOS_NODE_ID;
		tuple->hopcount = 0; 		
		
	}
	return SUCCESS;
  }
  
  // user of link estimator calls send here
  // slap the header and footer before sending the message
  command error_t Send.send(am_addr_t addr, message_t* msg, uint8_t len) {
    uint8_t newlen;
    newlen = addLinkEstHeaderAndFooter(msg, len);
    dbg("LITest", "%s packet of length %hhu became %hhu\n", __FUNCTION__, len, newlen);
    dbg("LI", "Sending seq: %d\n", linkEstSeq);
    print_packet(msg, newlen);

    if (mod_beacon_to_post != TRUE){
    	
    	last_sent_beacon = FALSE;
    	nb_sending = TRUE;

		//printf("LE T tx'ing beacon to %i \n", addr);
		//printfflush();    	
		
    	return call AMSend.send(addr, msg, newlen);
    } else {
    	return SUCCESS;
    }

  }

  // done sending the message that originated by
  // the user of this component
  event void AMSend.sendDone(message_t* msg, error_t error ) {
  	uint8_t payloadLen;
  	
  	//printf("LE: T tx'ed beacon \n");
  	//printfflush();
  	
  	if (error == ENOACK){     // Only the case of modified beacon
  	
  	  if(retries>0){ 		  // code for retries
  	  	retries--;
  	  	
		last_sent_beacon = TRUE;		  	  	
		payloadLen = call SubPacket.payloadLength(modified_ReTX_Beacon_Ptr);
		
		if (nb_sending == FALSE)
			call AMSend.send(current_parent, modified_ReTX_Beacon_Ptr, payloadLen);
		else
  	  		mod_beacon_to_post = TRUE;		
			
		//printf("LE: T re-tx'ing modified beacon to %i  with length %i\n",current_parent, no_of_messages);
		//printfflush();
		
		
  	  } else {           // When we have tried MAX_BEACON_RETS times 
  	  	
		last_sent_beacon = FALSE; 
		mod_beacon_to_post = FALSE; 	  	  	
  		retries = MAX_BEACON_RETS;
  	  	post re_init_msg();         // Very important, re-initialization
  	  	
  	  	//printf("LE: T failed to tx modified beacon \n");
		//printfflush();
  	  
  	  }
  	  
  	} else {                  // Successful transmission
  		if (last_sent_beacon == FALSE){    // Normal beacon
  			nb_sending = FALSE;
  			return signal Send.sendDone(msg, error); 
  		} 
  		else{				 // Modified beacon              
  			last_sent_beacon = FALSE; 
  			mod_beacon_to_post = FALSE; 
  			retries = MAX_BEACON_RETS;
  			post re_init_msg();    // Very important, re-initialization
  			
  			//printf("LE: T successfully tx'ed modified beacon with length %i\n",no_of_messages);
			//printfflush();  			
  			
  		}  
  	}
  }


  // cascade the calls down
  command uint8_t Send.cancel(message_t* msg) {
    return call AMSend.cancel(msg);
  }

  command uint8_t Send.maxPayloadLength() {
    return call Packet.maxPayloadLength();
  }

  command void* Send.getPayload(message_t* msg, uint8_t len) {
    return call Packet.getPayload(msg, len);
  }
	
  // called when link estimator generator packet or
  // packets from upper layer that are wired to pass through
  // link estimator is received
  void processReceivedMessage(message_t* ONE msg, void* COUNT_NOK(len) payload, uint8_t len) {
    uint8_t nidx;
    uint8_t num_entries;
    uint8_t how_many_msgs;
    uint8_t i,jj;

    dbg("LI", "LI receiving packet, buf addr: %x\n", payload);
    print_packet(msg, len);
	
	cp_beacon_re = TRUE;
	
    if (call SubAMPacket.destination(msg) == AM_BROADCAST_ADDR) {
      linkest_header_t* hdr = getHeader(msg);
      linkest_footer_t* ONE footer;
      am_addr_t ll_addr;
	  jj =0;
	  	
      ll_addr = call SubAMPacket.source(msg);

      dbg("LI", "Got seq: %d from link: %d\n", hdr->seq, ll_addr);

      num_entries = hdr->flags & NUM_ENTRIES_FLAG;
      last_rx_beacon_MB = FALSE;
      
      /**print_neighbor_table(); **/

      // update neighbor table with this information
      // find the neighbor
      // if found
      //   update the entry
      // else
      //   find an empty entry
      //   if found
      //     initialize the entry
      //   else
      //     find a bad neighbor to be evicted
      //     if found
      //       evict the neighbor and init the entry
      //     else
      //       we can not accommodate this neighbor in the table
      nidx = findIdx(ll_addr);
      if (nidx != INVALID_RVAL) {
	dbg("LI", "Found the entry so updating\n");
	updateNeighborEntryIdx(nidx, hdr->seq);
      } else {
	nidx = findEmptyNeighborIdx();
	if (nidx != INVALID_RVAL) {
	  dbg("LI", "Found an empty entry\n");
	  initNeighborIdx(nidx, ll_addr);
	  updateNeighborEntryIdx(nidx, hdr->seq);
	} else {
	  nidx = findWorstNeighborIdx(EVICT_EETX_THRESHOLD);
	  if (nidx != INVALID_RVAL) {
	    dbg("LI", "Evicted neighbor %d at idx %d\n",
		NeighborTable[nidx].ll_addr, nidx);
	    signal LinkEstimator.evicted(NeighborTable[nidx].ll_addr);
	    initNeighborIdx(nidx, ll_addr);
	  } else {
	    dbg("LI", "No room in the table\n");
	  }
	}
      }

      /* Graphical explanation of how we get to the head of the
       * footer in the following code 
       * <---------------------- payloadLen ------------------->
       * -------------------------------------------------------
       * linkest_header_t  |  payload   | linkest_footer_t* ...|
       * -------------------------------------------------------
       * ^                              ^                      ^
       * |                              |                      |
       * subpayload                     |                      payloadEnd
       *                                |
       *                                payloadEnd - footersize*num footers
      */

      if ((nidx != INVALID_RVAL) && (num_entries > 0)) {
		
		uint8_t payloadLen = call SubPacket.payloadLength(msg);
		void* COUNT_NOK(payloadLen) subPayload = call SubPacket.getPayload(msg, payloadLen);
		void* payloadEnd = subPayload + payloadLen;
		dbg("LI", "Number of footer entries: %d\n", num_entries);
		
		footer = TCAST(linkest_footer_t* COUNT(num_entries), (payloadEnd - (num_entries*sizeof(linkest_footer_t))));
		{
		  uint8_t i;
		  am_addr_t my_ll_addr;
	          neighbor_stat_entry_t * COUNT(num_entries) neighborLists;
		  my_ll_addr = call SubAMPacket.address();
	          neighborLists = TCAST(neighbor_stat_entry_t * COUNT(num_entries), footer->neighborList);
		  
		  for (i = 0; i < num_entries; i++) {
		    if (neighborLists[i].ll_addr == my_ll_addr) {    	
		      jj = 1;
		      	
		      //printf("LE outquality   %i\n",neighborLists[i].inquality);
		     // printfflush();
		      
		      updateReverseQuality(ll_addr, neighborLists[i].inquality);
		    } 		    
		  }
		  
		}   // footer ends
		
      } // if ends
      
      if (call SubAMPacket.source(msg) == COORDINATOR_ADDRESS && jj == 0 ){
      	cp_beacon_re = FALSE;               // Coordinator could not hear us            
      }
      
    } else { // This is a modified beacon (a directed transmission / reception) 
    	
    	last_rx_beacon_MB = TRUE;
        child_addr = call SubAMPacket.source(msg);

    	mod_hdr = getModHeader(msg);
		how_many_msgs = mod_hdr->no_of_msgs;   // What is the lenght of the received vector	
		
		printf("Received MB:   %i",mod_hdr->no_of_msgs);		
		printfflush();
		
    	if(TOS_NODE_ID == COORDINATOR_ADDRESS){ 	// if this is sink
    	// fill in the code for the filling of the MAC layer message    		
    			    		
			for (i=0; i < how_many_msgs ; i++){
				tuple_rx = getTuple(msg,i);				
				if (tuple_rx->addr!=CONST_NODE_ID){			// if this node is not already heard		
					hop_nodes[i][ hop1_nodes_idx[i] ] = (uint8_t)tuple_rx->addr;    // this casting is neccesary  
					hop1_nodes_idx[i] = hop1_nodes_idx[i] + 1; 					
				}									
				printf("   %i   %i",tuple_rx->addr,tuple_rx->hopcount);		
				printfflush();				
			}  // For loop	ends
			
			printf("\n");
			printfflush();    	

    	} else {   	// else this is a device node
     				
			for (i=0; i < how_many_msgs ; i++){

				tuple_rx = getTuple(msg,i);
				tuple = getTuple(modified_Beacon_Ptr , no_of_messages);
				
				tuple->addr = tuple_rx->addr;
				tuple->hopcount = (tuple_rx->hopcount) + 1;

				printf("   %i   %i",tuple_rx->addr,tuple_rx->hopcount);		
				printfflush();
				
				no_of_messages++;               // Always increment in the end
			}  // For loop	ends
			
			printf("\n");
			printfflush();
			
    	} //If  device ends 
    } // If modified beacon end  
  } // processReceived Message end





  // new messages are received here
  // update the neighbor table with the header
  // and footer in the message
  // then signal the user of this component
  event message_t* SubReceive.receive(message_t* msg,
				      void* payload,
				      uint8_t len) {
    
    //printf("LI Received upper packet. Will signal up\n");
    //printfflush();
    
    processReceivedMessage(msg, payload, len);
    if (last_rx_beacon_MB == FALSE){ // This is not the modified beacon 
    
        //printf("LE: Rxed normal beacon \n");
    	//printfflush();
    	
    	if (cp_beacon_re == TRUE) {            // Yes, can pass beacon to RE 
	    	return signal Receive.receive(msg,
					  call Packet.getPayload(msg, call Packet.payloadLength(msg)),
					  call Packet.payloadLength(msg));
		} else{
			
			//printf("LE T Beacon from coordinator but it could not hear us \n");
			//printfflush();
			
			return msg;
		}
		
	} else {		// This is a modified beacon
			mod_hdr = getModHeader(msg);
			
			//printf("LE: T rx modified beacon from   %i with length %i\n",child_addr, mod_hdr->no_of_msgs);
			//printfflush();     
			   			
			signal LinkEstimator.receivedMB(child_addr);	
			return msg;  	
	}
  }
  
  // If this node is Sink / Coordinator, then it should pass 
  // on the information to the MAC layer. 
  
  command error_t LinkEstimator.PassMsgToMAC() {

    process_before_MAC();
    clear_schedule();    

    return SUCCESS;
  }
  
  /**///////////////////////////////// Schedule //////////////////////////////////**/
  
  void process_before_MAC(){
	uint8_t i,j;  		
	for (i=0; i < 6 ; i++){
		for (j=0; j<hop1_nodes_idx[i];j++){		
			//hop_nodes[i][ hop1_nodes_idx[i] ] = (uint8_t)tuple_rx->addr; 
			Schedule[msgToMAClen] = hop_nodes[i][j];
			msgToMAClen++;
		}									
	}  // For loop	ends
	
	call HasTimeSlot.setSchedule(Schedule, msgToMAClen);
  }
  
  void clear_schedule(){
  	uint8_t i,j;
	for (i=0; i < 32 ; i++){
		Schedule[i] =0;									
	}  // For loop	ends
  	msgToMAClen =0;
  }

  command bool LinkEstimator.canSelectThisAsParent(am_addr_t own, am_addr_t neighbor){	  	
  	uint8_t own_idx, neighbor_idx;
  	
  	own_idx = search_Schedule(own);
  	neighbor_idx = search_Schedule(neighbor);
  	
  	//printf("LE T Schedule own_idx  %i   neighbor_idx  %i \n",own_idx, neighbor_idx);
  	//printfflush();
  	
  	if (own_idx ==255 || neighbor_idx == 255) {return 0;} 
  	
  	if ( neighbor_idx < own_idx || neighbor == COORDINATOR_ADDRESS ) {return 1;}
  	else {return 0;} 
  }


  uint8_t search_Schedule(am_addr_t node){
  	uint8_t i,no_of_nodes;
  	
  	no_of_nodes = call HasTimeSlot.getSchedule(Schedule);
  	
  	//printf("LE T No. of nodes in the schdule are %i\n",no_of_nodes);
  	//printfflush();
  	
  	for ( i = 0; i < no_of_nodes; i++) {
  		
  		//printf("LE  T  Schedule[%i]  \n",i ,Schedule[i]);
  		//printfflush();
  		
       if (Schedule[i] == node){  break;  }
    }
     if (no_of_nodes == 0){return 255; }
     else {return i;} 	
  }




  /**///////////////////////////////// Schedule //////////////////////////////////**/

	
  command void Packet.clear(message_t* msg) {
    call SubPacket.clear(msg);
  }

  // subtract the space occupied by the link estimation
  // header and footer from the incoming payload size
  command uint8_t Packet.payloadLength(message_t* msg) {
    linkest_header_t *hdr;
    hdr = getHeader(msg);
    return call SubPacket.payloadLength(msg)
      - sizeof(linkest_header_t)
      - sizeof(linkest_footer_t)*(NUM_ENTRIES_FLAG & hdr->flags);
  }

  // account for the space used by header and footer
  // while setting the payload length
  command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
    linkest_header_t *hdr;
    hdr = getHeader(msg);
    call SubPacket.setPayloadLength(msg,
				    len
				    + sizeof(linkest_header_t)
				    + sizeof(linkest_footer_t)*(NUM_ENTRIES_FLAG & hdr->flags));
  }

  command uint8_t Packet.maxPayloadLength() {
    return call SubPacket.maxPayloadLength() - sizeof(linkest_header_t);
  }

  // application payload pointer is just past the link estimation header
  command void* Packet.getPayload(message_t* msg, uint8_t len) {
    void* payload = call SubPacket.getPayload(msg, len +  sizeof(linkest_header_t));
    if (payload != NULL) {
      payload += sizeof(linkest_header_t);
    }
    return payload;
  }
      
}

