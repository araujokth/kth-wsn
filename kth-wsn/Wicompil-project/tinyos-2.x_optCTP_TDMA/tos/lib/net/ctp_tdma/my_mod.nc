  command error_t Send.send[uint8_t client](message_t* msg, uint8_t len) {
  	fe_queue_entry_t *qe;
   	
   	//ECollMessage* pkt = (ECollMessage*) call Packet.getPayload(msg,sizeof(ECollMessage));
	//payloadLength = call SubPacket.payloadLength(msg);
	//payload = call SubPacket.getPayload(msg, sizeof(ctp_data_header_t));
	
    uint8_t* payload = NULL;
    uint8_t* payload_agg = NULL;
    uint8_t* payload_agg_init = NULL;
    

	uint8_t queue_idx = call SendQueue.size();
    
        
    //printf("Forwarder sending packet from client %u len %u\n", client, len);
    //printfflush();
    
    if (!hasState(ROUTING_ON)) {return EOFF;}
    if (len > call Send.maxPayloadLength[client]()) {return ESIZE;}
    
	// This is the size of aggregated msg - CTP header length
	// because, the Packet.getPayload will add the space for the header	
	
	call Packet.setPayloadLength(msg, len);
    
	hdr = getHeader(msg);
    hdr->origin = TOS_NODE_ID;
    hdr->originSeqNo  = seqno++;
    hdr->type = call CollectionId.fetch[client]();
    hdr->thl = 0;
    call CtpPacket.setNumber(msg,queue_idx);

	//printf("*******AggMsgs in Send.send() is******* %i  \n",queue_idx,call CtpPacket.getNumber(););
	//printfflush();
	
	call SubPacket.setPayloadLength(aggMsgPtr, aggDataLen );
	
	// First fill in the own data
	payload = call SubPacket.getPayload(msg, sizeof(ctp_data_header_t) + MSGLEN);
	payload_agg = call SubPacket.getPayload(aggMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
	payload_agg_init = payload_agg;
	
	
	memcpy(payload_agg, payload , sizeof(ctp_data_header_t) + MSGLEN );
	payload_agg = payload_agg + sizeof(ctp_data_header_t) + MSGLEN;
			
	// Then fill in the other data (where is the aggMsgPtr ?)	
    if (queue_idx > 0) {
    
	  while(queue_idx >0) {
					
		qe = call SendQueue.element(queue_idx-1);	
		payload = call SubPacket.getPayload(qe->msg, sizeof(ctp_data_header_t) + MSGLEN);
			
		memcpy(payload_agg, payload , sizeof(ctp_data_header_t) + MSGLEN );
		payload_agg = payload_agg + sizeof(ctp_data_header_t) + MSGLEN;
		queue_idx = queue_idx -1;
		
	  }
      
    }

    // Also de-queue the other packets here
    // (own data is not part of the queue)
	
	queue_idx = call SendQueue.size();
	
    if (queue_idx > 0) {
      while(queue_idx >0) {
		qe = call SendQueue.element(queue_idx-1);	
		call SendQueue.dequeue();	
		queue_idx = queue_idx -1;
      } 
    }else{
      	call SendQueue.dequeue();
    }

	//printf("Size of queue before aggMsg and before after de-queuing is:   %i\n",call SendQueue.size());
	//printfflush();
	
    qe = clientPtrs[client];	   	
   	qe->msg = aggMsgPtr;
   	qe->client = client;
   	qe->retries = MAX_RETRIES;
	
		    
    if (call SendQueue.enqueue(qe) == SUCCESS) {
      payload_agg = payload_agg_init;
      clientPtrs[client] = NULL;
      return SUCCESS;
    }
    else {
      payload_agg = payload_agg_init;
      printf("Forwarder: send failed as packet could not be enqueued.\n");
      printfflush();
      return FAIL;
    }       
   		payload_agg = payload_agg_init;
  }


//////////////////////////////////////////////////////////////////////////////////////

  event message_t* 
  SubReceive.receive(message_t* msg, void* payload, uint8_t len) {
  	    
    uint8_t i, thl;
    uint8_t aggMsgs = call CtpPacket.getNumber(msg);
    uint16_t gradient;
	
    uint16_t org,sqn, time_hl;
    am_addr_t dest;

	if (call RxMessageQueue.enqueue(msg) != SUCCESS){
		printf("Forwarder: could not enqueue the received msg \n");
		printfflush();
	} 

    // Loop-detection code:
    if (call CtpInfo.getEtx(&gradient) == SUCCESS) {
    	if (call CtpPacket.getEtx(msg) <= gradient) { call CtpInfo.triggerImmediateRouteUpdate(); }
    }
	
	if (call RootControl.isRoot()) {

		call SubPacket.setPayloadLength(aggtempMsgPtr, aggDataLen );
		call SubPacket.setPayloadLength(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN );
		
		memcpy(aggtempMsgPtr, msg, sizeof(message_t));			
		payload_aggtemp= call SubPacket.getPayload(aggtempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);		
		payload_aggtemp_init = payload_aggtemp;

		for (i = aggMsgs + 1; i>0; i--){
			
			payload_temp = call SubPacket.getPayload(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
			memcpy(payload_temp , payload_aggtemp , ctp_data_frame_len);
			
			hdr = getHeader(tempMsgPtr);
			org = hdr->origin;
			sqn = hdr->originSeqNo;
			time_hl = hdr->thl;
			
			dest = call UnicastNameFreeRouting.nextHop();	    
	
	        printf("%s%i  %s%i  %s%i  %s%i  T","Success_rx_from_node ", org ,"to_node ", 
	        TOS_NODE_ID,"SeqNo ", sqn, "time_to_live " , time_hl);
	  	    printfflush();    	    	
			
	 	    if (i == 1){	    // The last of the aggregated messages
	    	  
	    	  payload_aggtemp = payload_aggtemp + ctp_data_frame_len;	
		      return signal Receive.receive[collectid](tempMsgPtr, 
							       call Packet.getPayload(tempMsgPtr, call Packet.payloadLength(tempMsgPtr)), 
							       call Packet.payloadLength(tempMsgPtr));
		    } else {
				
				signal Receive.receive[collectid](tempMsgPtr, 
							       call Packet.getPayload(tempMsgPtr, call Packet.payloadLength(tempMsgPtr)), 
							       call Packet.payloadLength(tempMsgPtr));
		    }
		
		    payload_aggtemp = payload_aggtemp + ctp_data_frame_len;
							       					       
		 } // End of for loop
		 
	  } // End of If
	  
	  else if (!signal Intercept.forward[collectid](msg, 
			  call Packet.getPayload(msg, call Packet.payloadLength(msg)), 
			  call Packet.payloadLength(msg))){ 	
	    return msg;
	  }
	  
    }  // End of the Subreceive.receive function	









	
	// Setting the message length
	call SubPacket.setPayloadLength(aggtempMsgPtr, aggDataLen );
	call SubPacket.setPayloadLength(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN );
	
	// Copying the data and getting the pointer to payload region
	memcpy(aggtempMsgPtr, msg, sizeof(message_t));			
	payload_aggtemp= call SubPacket.getPayload(aggtempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
	
	payload_aggtemp_init = payload_aggtemp;

	for (i = aggMsgs + 1; i>0; i--){
		
		payload_temp = call SubPacket.getPayload(tempMsgPtr, sizeof(ctp_data_header_t) + MSGLEN);
		memcpy(payload_temp , payload_aggtemp , ctp_data_frame_len);
		
		hdr = getHeader(tempMsgPtr);
		org = hdr->origin;
		sqn = hdr->originSeqNo;
		time_hl = hdr->thl;
		
		dest = call UnicastNameFreeRouting.nextHop();	    
	    collectid = call CtpPacket.getType(tempMsgPtr);	

	    thl = call CtpPacket.getThl(tempMsgPtr);
	    thl++;		
		call CtpPacket.setThl(tempMsgPtr, thl);
	
	    if (len > call SubSend.maxPayloadLength()) {
	    	
	    	printf("In SubReceive.receive, len > call SubSend.maxPayloadLength()  \n");
	    	printfflush();
	    	
	        return tempMsgPtr;
	        
	    }
	
	    //See if we remember having seen this packet
	    //We look in the sent cache ...
	    if (call SentCache.lookup(tempMsgPtr)) {
	        call CollectionDebug.logEvent(NET_C_FE_DUPLICATE_CACHE);
	        
	        if (i == 1) {
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
	        	return tempMsgPtr;
	        } else {
	        	continue;
	        }

	    }
	
	    // I'm on the routing path and Intercept indicates that I
	    // should not forward the packet.
	    else {
		       printf("%s%i  %s%i  %s%i %s%i  T","Success_rx_from_node ", org ,"to_node ", TOS_NODE_ID, "SeqNo ", sqn, "time_to_live " , time_hl);
		       printfflush();    	
		       		       
		       if (i==1){
		       		return forward(tempMsgPtr);
			   } else {
			   		forward(tempMsgPtr);
			   }

			   
	    }  // End of the If statement
    	
    	// Move the pointer
		payload_aggtemp = payload_aggtemp + ctp_data_frame_len;
    	
    }  
       // End of the for loop        
       payload_aggtemp = payload_aggtemp_init;


///////////////////////////////////////////////////////////////////////////////


  