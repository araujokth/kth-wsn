/*
 * BaseStation154C.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.ieee802154.tkn154
 *        Created on: 2010/05/10 
 * Last modification: 
 *            Author: aitorhh
 *     
 */

#include "AM.h"
#include "Serial.h"
#include "TKN154.h"
#include "app_sensors.h"
#include "UserButton.h"
#include "math.h"

module BaseStation154P {
	uses {
		interface Boot;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;
		interface IEEE154BeaconFrame as BeaconFrame;
		interface Leds;
		interface Packet;
		//interface Timer<TMilli> as TimerTimeout;
#ifndef TKN154_BEACON_DISABLED
		interface IEEE154TxBeaconPayload;
		interface SuperframeStructure as BeaconSuperframe;
#endif

		interface SplitControl as SerialControl;
		interface AMSend as UartSend;
		interface Receive as UartReceive[am_id_t id];
		interface AMPacket as UartAMPacket;


		interface GetSet<ieee154_GTSdb_t*> as GtsCoordinatorDb;
		interface GtsUtility;

		interface Notify<bool> as IsEndSuperframe;
		interface GetNow<bool> as IsGtsOngoing;

	}
}
implementation {

	message_t m_frameToMote;
	message_t m_frameToPC;

	//am_id_t id = AM_MYMSG;
	MyMsg* m_toMote;
	EncMsgWT * m_toPC;
	RecMsgWT * m_fromPC;

	uint8_t m_payloadLenToMote = sizeof(MyMsg);
	uint8_t m_payloadLenToPC = sizeof(EncMsgWT);
	uint8_t m_payloadLenFromPC = sizeof(RecMsgWT);

	ieee154_PANDescriptor_t m_PANDescriptor;

	message_t uartQueueBufs[UART_QUEUE_LEN];
	message_t * ONE_NOK uartQueue[UART_QUEUE_LEN];
	uint8_t uartIn, uartOut;
	uint8_t bo;
	bool uartBusy, uartFull;
	
	bool ACTIVE = FALSE;
	bool INITIALIZED = FALSE;
	
	nx_float initial_value_integrator[2];
	uint16_t initial_value_sensors[2][2], initial_value_actuation[2];

	task void uartSendTask();
	void setAddressingFields(uint16_t address, message_t * frame);

	event void Boot.booted() {
		uint8_t i = 0;

		for (i = 0; i < UART_QUEUE_LEN; i++)
		uartQueue[i] = &uartQueueBufs[i];

		uartIn = uartOut = 0;
		uartBusy = FALSE;
		uartFull = TRUE;

		call SerialControl.start();

		call MLME_RESET.request(TRUE);
		
		setAddressingFields(BROADCAST_DESTINATION, &m_frameToMote);

		m_toMote = (MyMsg*)(call Packet.getPayload(&m_frameToMote,m_payloadLenToMote ));
		m_toPC = (EncMsgWT*)(call Packet.getPayload(&m_frameToPC,m_payloadLenToPC ));

		memset(m_toPC, 0, sizeof(m_toPC));
		
	}

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status != IEEE154_SUCCESS)
		return;
		call MLME_SET.phyTransmitPower(TX_POWER_COORDINATOR);
		call MLME_SET.macShortAddress(COORDINATOR_ADDRESS);
		call MLME_SET.macAssociationPermit(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);
		call MLME_SET.macGTSPermit(TRUE);

		call MLME_START.request(
				PAN_ID, // PANId
				RADIO_CHANNEL, // LogicalChannel
				0, // ChannelPage,
				0, // StartTime,
				BEACON_ORDER, // BeaconOrder
				SUPERFRAME_ORDER, // SuperframeOrder
				TRUE, // PANCoordinator
				FALSE, // BatteryLifeExtension
				FALSE, // CoordRealignment
				0, // CoordRealignSecurity,
				0 // BeaconSecurity
		);

	}

	void setAddressingFields(uint16_t address, message_t *frame)
	{
		ieee154_address_t deviceShortAddress;
		deviceShortAddress.shortAddress = address; // destination

		call Frame.setAddressingFields(
				frame,
				ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
				ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
				PAN_ID, // DstPANId,
				&deviceShortAddress, // DstAddr,
				NULL // security
		);

	}

	/************************************************************************************
	 *  DEVICE -----15.4----- BASE STATION -------EncMsg------- PC
	 ************************************************************************************/
	task void sendPacket2PC() {
			if (call MCPS_DATA.request ( &m_frameToMote, // frame,
							m_payloadLenToMote, // payloadLength,
							0, // msduHandle,
							TX_OPTIONS_ACK | TX_OPTIONS_GTS // TxOptions,
					) != IEEE154_SUCCESS) {
				call Leds.led0On(); //fail!
			}
	}
	//Store the new message in the UART input buffer
	event message_t* MCPS_DATA.indication (message_t* frame)
	{
		message_t* ret = frame;
		MyMsg *m_device;
		nx_float K[3], outf;
		
		if( ACTIVE ) {

			K[0] = -0.1160; //
			K[1] = -0.1400; //-0.1160   -0.1400   -0.0190
			K[2] = -0.0190; //
	
			// We assume that the packets with length equal to MyMsg
			// are a MyMsg
			if (call Frame.getPayloadLength(frame) == m_payloadLenToMote &&
					call Frame.getFrameType(frame) == FRAMETYPE_DATA) {
	
				m_device = (MyMsg*)(call Packet.getPayload(frame,m_payloadLenToMote));
	
				if( m_device->srcId == 1 ) {
	
					m_toPC->y11 = m_device->data[0] - initial_value_sensors[0][0];
					m_toPC->y12 = m_device->data[1] - initial_value_sensors[0][1];
					
					m_device->integrator = m_device->integrator - initial_value_integrator[0];
	
					setAddressingFields((uint16_t) 0x02, &m_frameToMote);
	
					m_toMote->other = 0x05; // Write to DAC
					m_toMote->srcId = COORDINATOR_ADDRESS;
					m_toMote->trgtId = 0x02; // DAC mote on tank system 1
	
					// 1cm in the tube is approx. 95.733 units in the ADC 
					// 1v in the pump is approx. 273 units in the DAC
					//outf = 5.4204 * 273.0 + (273.0 * (((((nx_float) m_toPC->y11)/87.36) - 10.0) * K[0] + ((((nx_float) m_toPC->y12)/87.36) - 10.0) * K[1] + (m_device->integrator + 62.0755) * K[2]));
					//outf = (273.0 * (((((nx_float) m_toPC->y11)/87.36) - 10.0) * K[0] + ((((nx_float) m_toPC->y12)/87.36) - 10.0) * K[1] + (m_device->integrator + 62.0755) * K[2]));
					
					outf = initial_value_actuation[0] + (273.0 * (((((nx_float) m_toPC->y11)/87.36)) * K[0] + ((((nx_float) m_toPC->y12)/87.36)) * K[1] + (m_device->integrator) * K[2]));
					
					if(outf < 0) outf = 0;
					m_toMote->data[0] = (uint16_t) outf;
					if(m_toMote->data[0] > 4095) m_toMote->data[0] = 4095;
					m_toPC->u1 = m_toMote->data[0];	
					m_toPC->i1 = m_device->integrator;
					
										post sendPacket2PC();
	
				} else if( m_device->srcId == 3 ) {
	
					m_toPC->y21 = m_device->data[0] - initial_value_sensors[1][0];
					m_toPC->y22 = m_device->data[1] - initial_value_sensors[1][1];
					
					m_device->integrator = m_device->integrator - initial_value_integrator[1];
	
					setAddressingFields((uint16_t) 0x04, &m_frameToMote);
	
					m_toMote->other = 0x05; // Write to DAC
					m_toMote->srcId = COORDINATOR_ADDRESS;
					m_toMote->trgtId = 0x04; // DAC mote on tank system 2
	
					// 1cm in the tube is approx. 87.36 units in the ADC 
					// 1v in the pump is approx. 273 units in the DAC
					//outf = 5.4204 * 273.0 + (273.0 * (((((nx_float) m_toPC->y21)/87.36) - 10.0) * K[0] + ((((nx_float) m_toPC->y22)/87.36) - 10.0) * K[1] + (m_device->integrator + 62.0755) * K[2]));
					//outf = (273.0 * (((((nx_float) m_toPC->y21)/87.36) - 10.0) * K[0] + ((((nx_float) m_toPC->y22)/87.36) - 10.0) * K[1] + (m_device->integrator + 62.0755) * K[2]));
					
					outf = initial_value_actuation[1] + (273.0 * (((((nx_float) m_toPC->y21)/87.36)) * K[0] + ((((nx_float) m_toPC->y22)/87.36)) * K[1] + (m_device->integrator) * K[2]));
					
					if(outf < 0) outf = 0;
					m_toMote->data[0] = (uint16_t) outf;
					if(m_toMote->data[0] > 4095) m_toMote->data[0] = 4095;
					m_toPC->u2 = m_toMote->data[0];	
					m_toPC->i2 = m_device->integrator;	
					
										post sendPacket2PC();
	
				} else if( m_device->srcId == 5 ) {
	
					m_toPC->s1 = m_device->data[0];
	
				} else if( m_device->srcId == 6 ) {
	
					m_toPC->s2 = m_device->data[0];
	
				} else if( m_device->srcId == 7 ) {
	
					m_toPC->s3 = m_device->data[0];
	
				}
	
			}
		
		} else {
		
			if (call Frame.getPayloadLength(frame) == m_payloadLenToMote &&
					call Frame.getFrameType(frame) == FRAMETYPE_DATA) {
	
				m_device = (MyMsg*)(call Packet.getPayload(frame,m_payloadLenToMote));
	
				if( m_device->srcId == 1 ) {
	
					setAddressingFields((uint16_t) 0x02, &m_frameToMote);
	
					m_toMote->other = 0x05; // Write to DAC
					m_toMote->srcId = COORDINATOR_ADDRESS;
					m_toMote->trgtId = 0x02; // DAC mote on tank system 1
	
					// 1cm in the tube is approx. 95.733 units in the ADC 
					// 1v in the pump is approx. 273 units in the DAC
					outf = (273.0 * 4.8); //4.38
					if(outf < 0) outf = 0;
					m_toMote->data[0] = (uint16_t) outf;
					if(m_toMote->data[0] > 4095) m_toMote->data[0] = 4095;
					
		initial_value_actuation[0] = m_toMote->data[0];
		
					initial_value_integrator[0] = m_device->integrator;
					initial_value_sensors[0][0] = m_device->data[0];
					initial_value_sensors[0][1] = m_device->data[1];
					
										post sendPacket2PC();
	
				} else if( m_device->srcId == 3 ) {
	
					setAddressingFields((uint16_t) 0x04, &m_frameToMote);
	
					m_toMote->other = 0x05; // Write to DAC
					m_toMote->srcId = COORDINATOR_ADDRESS;
					m_toMote->trgtId = 0x04; // DAC mote on tank system 2
	
					// 1cm in the tube is approx. 87.36 units in the ADC 
					// 1v in the pump is approx. 273 units in the DAC
					outf = (273.0 * 4.8);
					if(outf < 0) outf = 0;
					m_toMote->data[0] = (uint16_t) outf;
					if(m_toMote->data[0] > 4095) m_toMote->data[0] = 4095;
					
		initial_value_actuation[1] = m_toMote->data[0];
					
					initial_value_integrator[1] = m_device->integrator;
					initial_value_sensors[1][0] = m_device->data[0];
					initial_value_sensors[1][1] = m_device->data[1];
					
										post sendPacket2PC();
	
				}
			}
		}
		return frame;
	}

	//Send to the serial port
	task void uartSendTask() {
		message_t* msg;

		atomic {
			if (uartIn == uartOut && !uartFull) {
				uartBusy = FALSE;
				return;
			}
		}

		msg = uartQueue[uartOut];

		if (call UartSend.send(AM_BROADCAST_ADDR, uartQueue[uartOut], m_payloadLenToPC ) == SUCCESS) {
			call Leds.led1Toggle();
		} else {
			call Leds.led0Toggle();
			post uartSendTask();
		}
	}

	event void MLME_START.confirm(ieee154_status_t status) {
	ieee154_GTSdb_t* GTSdb;
		uint8_t i = 0;
		if( ! INITIALIZED ) {
		
			INITIALIZED  = TRUE;
		/***  INITIALIZE GTS SETTINGS  ***/
		
	
			/* It is important to keep this order because the MLME_SET.macSuperframeOrder
			 * is the function that set the signal to write the new configuration
			 * 
			 * 1. Coordinator database
			 * 2. Beacon order
			 * 3. Superfame order
			 */
	
			GTSdb = call GtsCoordinatorDb.get();
			
			for (i=0; i < CFP_NUMBER_SLOTS; i++)
				call GtsUtility.setNullGtsEntry(&(GTSdb->db[i]));
	
			call GtsUtility.addGtsEntry(GTSdb, 1, 9, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 2, 10, 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 3, 11, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 4, 12, 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 5, 13, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 6, 14, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 7, 15, 1, GTS_TX_ONLY_REQUEST);
	
			GTSdb->numGtsSlots = 7; //delete all the frames
	
			call MLME_SET.macBeaconOrder(BEACON_ORDER);
			call MLME_SET.macSuperframeOrder(SUPERFRAME_ORDER);
		}
	
	}

	event void SerialControl.startDone(error_t error) {

		if (error == SUCCESS) {
			uartFull = FALSE;
		}
	}

	event void SerialControl.stopDone(error_t error) {}

	event void UartSend.sendDone(message_t* msg, error_t error) {
		if (error != SUCCESS) {
			call Leds.led0Toggle();
		} else
		atomic
		if (msg == uartQueue[uartOut])
		{
			if (++uartOut >= UART_QUEUE_LEN)
			uartOut = 0;
			if (uartFull)
			uartFull = FALSE;
		}
		post uartSendTask();
	}

	/************************************************************************************
	 *  PC -----EncMsg----- BASE SATITON -------15.4------- DEVICE
	 ************************************************************************************/
	event message_t *UartReceive.receive[am_id_t id](message_t *msg,
			void *payload,
			uint8_t len) {

		/**                                                      **  
		 
		 *** DO WHATEVER WE NEED TO DO WITH THE PROTOCOL SETTINGS *** 
		 
		 **                                                      **/

		ieee154_GTSdb_t* GTSdb;
		uint8_t i = 0;
		char nodes[7], nUnused, dummyNodes[7];
		bool used[7];
		nx_int8_t *pSlots;
		m_fromPC = call Packet.getPayload(msg, m_payloadLenFromPC);
		
		if( m_fromPC->BI == 0xff ) {
			
			ACTIVE = TRUE;
			
			/***  INITIALIZE GTS SETTINGS  ***/
		
	
			/* It is important to keep this order because the MLME_SET.macSuperframeOrder
			 * is the function that set the signal to write the new configuration
			 * 
			 * 1. Coordinator database
			 * 2. Beacon order
			 * 3. Superfame order
			 */
	
			GTSdb = call GtsCoordinatorDb.get();
			
			for (i=0; i < CFP_NUMBER_SLOTS; i++)
				call GtsUtility.setNullGtsEntry(&(GTSdb->db[i]));
	
			call GtsUtility.addGtsEntry(GTSdb, 1, 9, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 2, 10, 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 3, 11, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 4, 12, 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 5, 13, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 6, 14, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 7, 15, 1, GTS_TX_ONLY_REQUEST);
	
			GTSdb->numGtsSlots = 7; //delete all the frames
	
			call MLME_SET.macBeaconOrder(BEACON_ORDER);
			call MLME_SET.macSuperframeOrder(SUPERFRAME_ORDER);
			
		} else {
		
			pSlots = (nx_int8_t *) m_fromPC;
			
			memset(used, FALSE, 7*sizeof(bool));
			
			nodes[0]=1;
			nodes[1]=2;
			nodes[2]=3;
			nodes[3]=4;
			nodes[4]=5;
			nodes[5]=6;
			nodes[6]=7;
			
			nUnused = 0;
	
			/* It is important to keep this order because the MLME_SET.macSuperframeOrder
			 * is the function that set the signal to write the new configuration
			 * 
			 * 1. Coordinator database
			 * 2. Beacon order
			 * 3. Superfame order
			 */
	
			GTSdb = call GtsCoordinatorDb.get();
			
			for (i=0; i < CFP_NUMBER_SLOTS; i++)
				call GtsUtility.setNullGtsEntry(&(GTSdb->db[i]));
			
			//GTSdb->numGtsSlots = 0; //delete all the frames
			
			
			/***                                                                        ***
			          The solution  implemented  on the  protocol layer  of assigning
			        slot 16 (non-existent  slot) to a node if we don't  want the node 
			        to  have a slot  during the  next beacon interval  is not working
			        correctly, so the following workaround is made:
			          - Set up a vector to contain the node numbers
			          - Check if the corresponding slot number is 16
			          - If it is, change the node number to 8 (non-existent node)
			          - Replace the slot numbers assigned to node 8 with unused slots 
			 ***                                                                        ***/
	
			
			
			for(i=0;i<7;i++) {
				if( pSlots[i+1] == 16 ) { nodes[i] = 8; dummyNodes[nUnused++] = i+1; }
				else { used[pSlots[i+1]-9] = TRUE; }
			}
			for(i=0;i<7;i++) {
				if(nUnused == 0) break;
				if(!used[i]) {
					nUnused--;
					pSlots[dummyNodes[nUnused]] = i+9;
				}
			}
			
			/*call GtsUtility.addGtsEntry(GTSdb, 1, m_fromPC->T1, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 2, m_fromPC->T2, 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 3, m_fromPC->T3, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 4, m_fromPC->T4, 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 5, m_fromPC->T5, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 6, m_fromPC->T6, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, 7, m_fromPC->T7, 1, GTS_TX_ONLY_REQUEST);*/
			
			call GtsUtility.addGtsEntry(GTSdb, nodes[0], m_fromPC->T1, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, nodes[1], m_fromPC->T2, 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, nodes[2], m_fromPC->T3, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, nodes[3], m_fromPC->T4, 1, GTS_RX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, nodes[4], m_fromPC->T5, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, nodes[5], m_fromPC->T6, 1, GTS_TX_ONLY_REQUEST);
			call GtsUtility.addGtsEntry(GTSdb, nodes[6], m_fromPC->T7, 1, GTS_TX_ONLY_REQUEST);
	
			GTSdb->numGtsSlots = 7; //delete all the frames
	
	
			call MLME_SET.macBeaconOrder(m_fromPC->BI);
			call MLME_SET.macSuperframeOrder(SUPERFRAME_ORDER);
		
		}
	//printf("ch\n");

		return msg;

	}

	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	) {}
	/************************************************************************************
	 *  CFP EVENTS/NOTIFIES FUNCTIONS
	 ************************************************************************************/
	event void IsEndSuperframe.notify( bool val ) {
		
		
			//send packet with the time to the computer
			m_toPC->time = call BeaconSuperframe.sfStartTime();
			
			m_toPC->y11_initial = initial_value_sensors[0][0];
			m_toPC->y12_initial = initial_value_sensors[0][1];
			m_toPC->y21_initial = initial_value_sensors[1][0];
			m_toPC->y22_initial = initial_value_sensors[1][1];
			m_toPC->i1_initial = initial_value_integrator[0];
			m_toPC->i2_initial = initial_value_integrator[1];
			m_toPC->u1_initial = initial_value_actuation[0];
			m_toPC->u2_initial = initial_value_actuation[1];
			
			if ( (m_toPC->time * 0.000015259) > 80 ) {
		
				atomic {
					if (!uartFull)
					{
						memcpy(uartQueue[uartIn], &m_frameToPC, sizeof(message_t));
						uartIn = (uartIn + 1) % UART_QUEUE_LEN;
		
						if (!uartBusy)
						{
							post uartSendTask();
							uartBusy = TRUE;
						}
					}
					else {
						call Leds.led0Toggle();
					}
				}
			
			}
		
		//if (beaconCounts % 1) {
		//code below for debug purposes
		/*if (1) {
			call MLME_SET.macBeaconOrder(SUPERFRAME_ORDER + call Random.rand16() % 3);
			call MLME_SET.macSuperframeOrder(SUPERFRAME_ORDER);
		}*/
			//printf("end sf\n");
		
		return;
	}
	/************************************************************************************
	 *  BEACON FUNCTIONS
	 ************************************************************************************/
#ifndef TKN154_BEACON_DISABLED
	event void IEEE154TxBeaconPayload.aboutToTransmit() {
		//printf("bf tx\n");
	}

	event void IEEE154TxBeaconPayload.setBeaconPayloadDone(void *beaconPayload, uint8_t length) {}

	event void IEEE154TxBeaconPayload.modifyBeaconPayloadDone(uint8_t offset, void *buffer, uint8_t bufferLength) {}

	event void IEEE154TxBeaconPayload.beaconTransmitted()
	{
		ieee154_macBSN_t beaconSequenceNumber = call MLME_GET.macBSN();
		

		//call TimerTimeout.startOneShot(482); // Send packet through the serial port when CFP is ended,
		// so that all packets from motes are received.
		// Assumming SFO = 5, 469/1000ms -> 482/1024ms with some extra space.

		if (beaconSequenceNumber & 1) {
			call Leds.led2On();
		}
		else {
			call Leds.led2Off();

		}
		bo=call MLME_GET.macBeaconOrder();
		if(bo != m_fromPC->BI);
	//printf("fudeo\n");

	}
#endif


	
}
