#include "LATIN.h"
#include "CC2420.h"
#include "IEEE802154E.h"

#include "printf.h"

module LatinMacP {
	//admin
	uses interface Boot;
	provides interface Init as SoftwareInit;
	//time
	uses interface Alarm<T32khz,uint32_t> as FastAlarm; //private
	uses interface Alarm<T32khz,uint32_t> as SlotAlarm; //private
	provides interface GlobalTime;
	provides interface GlobalSync;

	//down the stack
	provides interface OpenSend as OpenSendFromUpper;
	uses interface OpenQueue;
	uses interface RadioControl;
	uses interface RadioSend;
	uses interface RadioReceive;
	//up the stack
	uses interface OpenReceive as OpenReceiveToUpper;
	//debug
	uses interface Leds;
	uses interface PinDebug;
	//misc
	uses interface PacketFunctions;
	uses interface NeighborStats;
	uses interface Malloc;
	uses interface IDManager;
	uses interface Random;
	uses interface LatinMatrix;

	provides interface MAC;

}
implementation {
	/*------------------------------ variables -------------------------------------------*/
	timervalue_t fastAlarmStartSlotTimestamp;
	timervalue_t slotAlarmStartSlotTimestamp;
	uint8_t state;
	asn_t asn;
	OpenQueueEntry_t* dataFrameToSend; //NULL at beginning and end
	OpenQueueEntry_t* packetACK; //NULL at beginning and end, free at end of slot
	OpenQueueEntry_t* frameReceived; //NULL at beginning and end
	// Added for the TSCH Test Experiment
	OpenQueueEntry_t* pktToTx;
	open_addr_t temp_nextHop;
	///////////////////////////////////////
	bool isSync;
	uint8_t dsn;
	uint8_t frequencyChannel;
	OpenQueueEntry_t* sendDoneMessage;
	error_t sendDoneError;

	// TSCH Critical Time variables
	uint8_t init;
	uint8_t numReceiveNothing;
	uint8_t numADV;
	uint8_t numPrepareReceiveFailed;
	uint8_t numSlotsInSynchStatus;
	uint32_t timestampStart;
	uint32_t criticalTime;
	bool beacon_channel;
	bool change;
	bool change_IsDAG;

	// Code Scheduling
	norace uint8_t slotType;
	norace uint8_t networkState;
	/*------------------------------ prototypes ------------------------------------------*/
#include "IEEE802154_common.c"
	void change_state(uint8_t newstate);
	void resynchronize(bool resyncType, open_addr_t* node_id, timervalue_t dataGlobalSlotOffset, int16_t timeCorrection);
	void endSlot();
	//the two following tasks are used to break the asynchronicity: everything in MAC and below is async, all the above not
	task void taskReceive();
	void postTaskSendDone(OpenQueueEntry_t* param_sendDoneMessage, error_t param_sendDoneError);
	task void taskSendDone();
	/*------------------------------ start/stop sequence ---------------------------------*/
	event void Boot.booted() {
		atomic init=0;
		atomic numReceiveNothing=0;
		atomic numADV=0;
		atomic isSync=FALSE;
		atomic beacon_channel=TRUE;
		atomic change=FALSE;
		numPrepareReceiveFailed=0;
		numSlotsInSynchStatus=0;
		call Leds.led2Off();
		call RadioControl.start();

		// Code Scheduling
		slotType = UPLINK;
		networkState = BUILDING;
	}
	async event void RadioControl.startDone(error_t error) {
		call SlotAlarm.startAt((call SlotAlarm.getNow()),SLOT_TIME);
	}
	async event void RadioControl.stopDone(error_t error) {
		return;//radio turned off not implemented
	}
	/*------------------------------ recording a new packet to send ----------------------*/
	//OpenSendFromUpper
	command error_t OpenSendFromUpper.send(OpenQueueEntry_t* msg) {
		msg->owner = COMPONENT_MAC;
		if ((call PacketFunctions.isBroadcastMulticast(&(msg->l2_nextORpreviousHop))==TRUE)) {
			msg->l2_retriesLeft = 1;
		} else {
			msg->l2_retriesLeft = TXRETRIES;
		}
		msg->l1_txPower = TX_POWER;
		prependIEEE802154header(msg,
				msg->l2_frameType,
				IEEE154_SEC_NO_SECURITY,
				dsn++,
				&(msg->l2_nextORpreviousHop)
		);
		return SUCCESS;
	}
	/*------------------------------ new slot (TX or RX) ---------------------------------*/
	async event void SlotAlarm.fired() {
		asn_t temp_asn;
		uint8_t temp_state;
		bool temp_isSync;
		OpenQueueEntry_t* temp_dataFrameToSend;
		error_t temp_error;
		ieee802154_header_iht transmitted_ieee154_header;
		uint8_t* latinChannelOffset;

		// call PinDebug.ADC0toggle();


		fastAlarmStartSlotTimestamp = call FastAlarm.getNow();
		slotAlarmStartSlotTimestamp = call SlotAlarm.getNow();
		atomic asn++;
		if (slotType == UPLINK) {
			slotType = DOWNLINK;
		} else {
			slotType = UPLINK;
		}

		call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME);

		// reset WDT
		atomic WDTCTL = WDTPW + WDTCNTCL + WDTSSEL;

		atomic dataFrameToSend = NULL;

		atomic {
			temp_asn = asn;
			temp_state = state;
			temp_isSync = isSync;
			temp_dataFrameToSend = dataFrameToSend;
		}

		if ((call IDManager.getIsDAGroot())==TRUE) {
			// If I'm DAGroot, I'm synchronized
			atomic isSync=TRUE;
			call Leds.led2On();
			atomic {
				if (state==S_SYNCHRONIZING) {//happens right after node becomes LBR
					endSlot();
					return;
				}
			}
		} else if (temp_isSync==FALSE) {
			// If I'm not in sync, enter/stay in S_SYNCHRONIZING
			if (temp_state!=S_SYNCHRONIZING) {
				change_state(S_SYNCHRONIZING);
				if ((call RadioControl.prepareReceive(11))!=SUCCESS) {
					endSlot();
				}
			}
			return;
		}

		if (temp_state!=S_SLEEP) {
			endSlot();
			return;
		}

		switch (networkState) {
			case BUILDING:
			frequencyChannel = 11;

			if (((TOS_NODE_ID == (asn/2)%MAXNUMNEIGHBORS)&&(slotType == UPLINK))||((TOS_NODE_ID != (asn/2)%MAXNUMNEIGHBORS)&&(slotType == DOWNLINK))) {
				if (slotType == UPLINK) {

					dataFrameToSend = call OpenQueue.inQueueBySeed(DEFAULT_SEED);

				} else {
					dataFrameToSend = call OpenQueue.inQueueBySeed((asn/2)%MAXNUMNEIGHBORS);
				}
				if (dataFrameToSend!=NULL) {
					// Start the TX sequence

					// Set Frequency Channel
					atomic dataFrameToSend->l1_channel = frequencyChannel;

					transmitted_ieee154_header = retrieveIEEE802154header(dataFrameToSend);

					if (dataFrameToSend->l2_frameType == IEEE154_TYPE_CMD) {
						if (((CMD_MSG_t*)((dataFrameToSend->payload)+transmitted_ieee154_header.headerLength))->commandFrameId == SCAN_MSG) {
							//I will be sending an SCAN msg

							((CMD_MSG_t*)((dataFrameToSend->payload)+transmitted_ieee154_header.headerLength))->timingInformation=(call GlobalTime.getASN());
							//					((CMD_MSG_t*)((dataFrameToSend->payload)+transmitted_ieee154_header.headerLength+10))->timingInformation=slotType;
						}
					}
					change_state(S_TX_TXDATAPREPARE);
					atomic temp_error = call RadioSend.prepareSend(dataFrameToSend);
					if (temp_error!=SUCCESS) {
						// Retry sending the packet later
						temp_dataFrameToSend->l2_retriesLeft--;
						if (temp_dataFrameToSend->l2_retriesLeft==0) {
							postTaskSendDone(temp_dataFrameToSend,FAIL);
						}
						endSlot();
					};
					atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsTxOffset);
				}
				else {
					endSlot();
				}
			} else if (((TOS_NODE_ID == (asn/2)%MAXNUMNEIGHBORS)&&(slotType == DOWNLINK))||((TOS_NODE_ID != (asn/2)%MAXNUMNEIGHBORS)&&(slotType == UPLINK))) {
				// Start the RX sequence
				change_state(S_RX_RXDATAPREPARE);
				atomic temp_error = call RadioControl.prepareReceive(frequencyChannel);
				if (temp_error!=SUCCESS) {
					// Abort
					endSlot();
				};
				atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsRxOffset);
			}
			break;

			case SCHEDULING:
			if (slotType == UPLINK) {
				// Any child can transmit?
				if (call LatinMatrix.childCanTransmit (((asn/2))%(MAXNUMNEIGHBORS), latinChannelOffset)) {
					// Start the RX sequence
					printf("asn: %u\n",asn);
					printf("UPLINK\n");
					printf("C\n");
					printfflush();
					// Set Frequency Channel
					frequencyChannel = (*latinChannelOffset)+12;

					change_state(S_RX_RXDATAPREPARE);
					atomic temp_error = call RadioControl.prepareReceive(frequencyChannel);
					if (temp_error!=SUCCESS) {
						// Abort
						endSlot();
					};
					atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsRxOffset);
				} else {


					dataFrameToSend = call OpenQueue.inQueueBySeed(call LatinMatrix.getParentSeed());

					if (dataFrameToSend!=NULL) {

						// Any sibling node has a higher priority?
						if (call LatinMatrix.siblingCanTransmit(((asn/2))%(MAXNUMNEIGHBORS), latinChannelOffset)) {
							printf("asn: %u\n",asn);
							printf("UPLINK\n");
							printf("S\n");
							printfflush();
							endSlot();

						} else {
							printf("asn: %u\n",asn);
							printf("UPLINK\n");
							printf("MP\n");
							printfflush();
							// Start the TX sequence
;
							// Set Frequency Channel
							frequencyChannel = (*latinChannelOffset)+12;
							atomic dataFrameToSend->l1_channel = frequencyChannel;

							change_state(S_TX_TXDATAPREPARE);
//							atomic temp_error = call RadioSend.prepareSend(dataFrameToSend);
//							if (temp_error!=SUCCESS) {
//								// Retry sending the packet later
//								temp_dataFrameToSend->l2_retriesLeft--;
//								if (temp_dataFrameToSend->l2_retriesLeft==0) {
//									postTaskSendDone(temp_dataFrameToSend,FAIL);
//								}
//								endSlot();
//							};
							atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsTxOffset);

						}

					} else {
						endSlot();
					}

				}

			} else {
				// My parent can transmit?
				if (call LatinMatrix.parentCanTransmit(((asn/2))%(MAXNUMNEIGHBORS), latinChannelOffset)) {
					// Start the RX sequence
//					printf("asn: %u\n",asn);
//					printf("DOWNLINK\n");
//					printf("P\n");
//					printfflush();
					// Set Frequency Channel
					frequencyChannel = (*latinChannelOffset)+12;

					change_state(S_RX_RXDATAPREPARE);
					atomic temp_error = call RadioControl.prepareReceive(frequencyChannel);
					if (temp_error!=SUCCESS) {
						// Abort
						endSlot();
					};
					atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsRxOffset);
				} else {
					dataFrameToSend = call OpenQueue.inQueueToChild(call LatinMatrix.getParentSeed());
					if (dataFrameToSend!=NULL) {

						if (call LatinMatrix.canTransmit(((asn/2))%(MAXNUMNEIGHBORS),latinChannelOffset)) {
//							printf("asn: %u\n",asn);
//							printf("DOWNLINK\n");
//							printf("MC\n");
//							printfflush();
							// Start the TX sequence
							//							printf("MP\n");
							// Set Frequency Channel
							frequencyChannel = (*latinChannelOffset)+12;
							atomic dataFrameToSend->l1_channel = frequencyChannel;

							change_state(S_TX_TXDATAPREPARE);
//							atomic temp_error = call RadioSend.prepareSend(dataFrameToSend);
//							if (temp_error!=SUCCESS) {
//								// Retry sending the packet later
//								temp_dataFrameToSend->l2_retriesLeft--;
//								if (temp_dataFrameToSend->l2_retriesLeft==0) {
//									postTaskSendDone(temp_dataFrameToSend,FAIL);
//								}
//								endSlot();
//							};
							atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsTxOffset);

						} else {
//							printf("asn: %u\n",asn);
//							printf("DOWNLINK\n");
//							printf("MNC\n");
//							printfflush();
							endSlot();
						}
					} else {
						endSlot();
					}

				}

			}
			break;
			default:
		}
	}
	//	switch (call CellUsageGet.getType(temp_asn%LENGTHCELLFRAME)) {
	//		case CELLTYPE_TXRX:
	//		//get a packet out of the buffer (if any)
	//		if ( call CellUsageGet.isTX(temp_asn%LENGTHCELLFRAME) || call CellUsageGet.isSH_TX(temp_asn%LENGTHCELLFRAME)) {
	//			atomic {
	//				dataFrameToSend = call OpenQueue.inQueue(call CellUsageGet.isADV(temp_asn%LENGTHCELLFRAME));
	//				temp_dataFrameToSend = dataFrameToSend;
	//			}
	//		}
	//		temp_channelOffset = call CellUsageGet.getChannelOffset(temp_asn%LENGTHCELLFRAME);
	//		if (HOPPING_ENABLED) {
	//			atomic frequencyChannel = ((temp_asn+temp_channelOffset)%8)+16;
	//
	//		} else {
	//			atomic frequencyChannel =((temp_channelOffset)%15)+11;
	//		}
	//
	//		if (temp_dataFrameToSend!=NULL) { //start the TX sequence
	//			dataFrameToSend->owner = COMPONENT_MAC;
	//
	//			if(dataFrameToSend->creator==COMPONENT_RES) {
	//				atomic dataFrameToSend->l1_channel = 16;
	//			}
	//			else {
	//				atomic dataFrameToSend->l1_channel = frequencyChannel;
	//			}
	//
	//			transmitted_ieee154_header = retrieveIEEE802154header(temp_dataFrameToSend);
	//			if (call CellUsageGet.isADV(temp_asn%LENGTHCELLFRAME)) {
	//				//I will be sending an ADV frame
	//
	//				((IEEE802154E_ADV_t*)((dataFrameToSend->payload)+transmitted_ieee154_header.headerLength))->timingInformation=(call GlobalTime.getASN());
	//			}
	//			change_state(S_TX_TXDATAPREPARE);
	//			atomic temp_error = call RadioSend.prepareSend(dataFrameToSend);
	//			if (temp_error!=SUCCESS) {
	//				//retry sending the packet later
	//				temp_dataFrameToSend->l2_retriesLeft--;
	//				if (temp_dataFrameToSend->l2_retriesLeft==0) {
	//					postTaskSendDone(temp_dataFrameToSend,FAIL);
	//				}
	//				endSlot();
	//			};
	//			atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsTxOffset);
	//		} else {
	//			if ((call IDManager.getIsDAGroot())==FALSE) {
	//				if (beacon_channel) {
	//					atomic frequencyChannel =16;
	//				}
	//			}
	//			if (call CellUsageGet.isRX(temp_asn%LENGTHCELLFRAME)) { //start the RX sequence
	//				change_state(S_RX_RXDATAPREPARE);
	//				atomic temp_error = call RadioControl.prepareReceive(frequencyChannel);
	//				if (temp_error!=SUCCESS) {
	//					//abort
	//					endSlot();
	//				};
	//				atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsRxOffset);
	//			} else { //nothing to do, abort
	//				endSlot();
	//				return;
	//			}
	//		}
	//		return;
	//		break;
	//
	//		case CELLTYPE_RXSERIAL:
	//		endSlot();
	//		return;
	//
	//		case CELLTYPE_OFF:
	//		endSlot();
	//		return;
	//
	//		default:
	//		endSlot();
	//		return;
	//	}
	//}

	//prepareSendDone
	async event void RadioSend.prepareSendDone(error_t error) {
		asn_t temp_asn;
		uint8_t temp_state;
		atomic {
			temp_asn = asn;
			temp_state = state;
		}
		switch (temp_state) {
			case S_TX_TXDATAPREPARE:
			if (error==SUCCESS) {
				change_state(S_TX_TXDATAREADY);
			} else {
				atomic {
					dataFrameToSend->l2_retriesLeft--;
					if (dataFrameToSend->l2_retriesLeft==0) {
						postTaskSendDone(dataFrameToSend,FAIL);
					}
				}
				endSlot();
			}
			break;
			case S_RX_TXACKPREPARE:
			if (error==SUCCESS) {
				change_state(S_RX_TXACKREADY);
			} else {
				//abort
				endSlot();
			}
			break;
			default:
			endSlot();
			return;
			break;
		}
	}

	//prepareReceiveDone
	async event void RadioControl.prepareReceiveDone(error_t error) {
		asn_t temp_asn;
		uint8_t temp_state;
		atomic {
			temp_asn = asn;
			temp_state = state;
		}
		if (error!=SUCCESS) {
			// Abort
			endSlot();
		}
		switch (temp_state) {
			case S_TX_RXACKPREPARE:
			change_state(S_TX_RXACKREADY);
			break;
			case S_RX_RXDATAPREPARE:
			change_state(S_RX_RXDATAREADY);
			break;
			case S_SYNCHRONIZING:
			if ( (call RadioControl.receiveNow(TRUE,3200))!=SUCCESS ) {
				// Abort
				endSlot();
			}
			break;
			default:
			endSlot();
			return;
			break;
		}
	}

	//FastAlarm.fired
	async event void FastAlarm.fired() {
		asn_t temp_asn;
		uint8_t temp_state;
		OpenQueueEntry_t* temp_dataFrameToSend;
		atomic {
			temp_asn = asn;
			temp_state = state;
			temp_dataFrameToSend = dataFrameToSend;
		}
		switch (temp_state) {
			/*------------------- TX sequence ------------------------*/
			case S_TX_TXDATAPREPARE: //[timer fired] transmitter (ERROR state)
			//I'm a transmitter, didn't have time to prepare for TX
			postTaskSendDone(temp_dataFrameToSend,FAIL);
			endSlot();
			break;
			case S_TX_TXDATAREADY: //[timer fired] transmitter
			//I'm a transmitter, Tx data now
			change_state(S_TX_TXDATA);
			if ((call RadioSend.sendNow())!=SUCCESS) {
				//retry later
				temp_dataFrameToSend->l2_retriesLeft--;
				if (temp_dataFrameToSend->l2_retriesLeft==0) {
					postTaskSendDone(temp_dataFrameToSend,FAIL);
				}
				endSlot();
			}
			break;
			case S_TX_RXACKREADY: //[timer fired] transmitter
			//I'm a transmitter, switch on RX for ACK now
			//this is done automatically after Tx is finished, rx is already on.
			//I'm calling receiveNow anyways because it reports receivedNothing
			change_state(S_TX_RXACK);
			if ( (call RadioControl.receiveNow(TIME_LIMITED_RX,TsRxWaitTime))!=SUCCESS ) {
				//abort
				endSlot();
			};
			break;

			/*------------------- RX sequence -----------------------*/
			case S_RX_RXDATAPREPARE: //[timer fired] receiver (ERROR state)
			//I'm a receiver, didn't have time to prepare for RX

			endSlot();
			break;
			case S_RX_RXDATAREADY: //[timer fired] receiver
			//I'm a receiver, switch RX radio on for data now
			change_state(S_RX_RXDATA);
			if ( (call RadioControl.receiveNow(TIME_LIMITED_RX,TsRxWaitTime))!=SUCCESS ) {
				//abort
				endSlot();
			};
			break;
			case S_RX_TXACKPREPARE: //[timer fired] receiver (ERROR state)
			//I'm a receiver, didn't have time to prepare ACK
			endSlot();
			break;
			case S_RX_TXACKREADY: //[timer fired] receiver
			//I'm a receiver, TX ACK now
			change_state(S_RX_TXACK);
			if ((call RadioSend.sendNow())!=SUCCESS) {
				//abort
				endSlot();
			}
			break;
			default:
			endSlot();
			break;
		}
	}
	//receivedNothing
	async event void RadioControl.receivedNothing() {
		asn_t temp_asn;
		uint8_t temp_state;
		OpenQueueEntry_t* temp_dataFrameToSend;

		uint8_t temp_init;

		atomic {
			temp_asn = asn;
			temp_state = state;
			temp_dataFrameToSend = dataFrameToSend;
			temp_init = init;
		}
		switch(temp_state) {
			case S_RX_RXDATA: //[receivedNothing] receiver (WARNING state)
			//I'm a receiver, didn't receive data
			endSlot();
			break;
			case S_TX_RXACK: //[receivedNothing] transmitter (WARNING state)
			//I'm a transmitter, didn't receive ACK (end of TX sequence)

			temp_dataFrameToSend->l2_retriesLeft--;
			if (temp_dataFrameToSend->l2_retriesLeft==0) {
				postTaskSendDone(temp_dataFrameToSend,FAIL);
			}
			endSlot();
			break;
			case S_SYNCHRONIZING: //[receivedNothing] synchronizer
			//it's OK not to receive anything after TsRxWaitTime when trying to synchronize
			endSlot();
			break;
			default:
			endSlot();
			break;
		}
	}

	//sendNowDone
	async event void RadioSend.sendNowDone(error_t error) {
		asn_t temp_asn;
		uint8_t temp_state;
		OpenQueueEntry_t* temp_dataFrameToSend;
		atomic {
			temp_asn = asn;
			temp_state = state;
			temp_dataFrameToSend = dataFrameToSend;
		}
		switch (temp_state) {
			case S_TX_TXDATA: //[sendNowDone] transmitter
			//I'm a transmitter, finished sending data
			if (error!=SUCCESS) {
				//retry later
				temp_dataFrameToSend->l2_retriesLeft--;
				if (temp_dataFrameToSend->l2_retriesLeft==0) {
					postTaskSendDone(temp_dataFrameToSend,FAIL);
				}
				endSlot();
				return;
			}

			if ((call PacketFunctions.isBroadcastMulticast(&(temp_dataFrameToSend->l2_nextORpreviousHop))==TRUE)) {

				postTaskSendDone(temp_dataFrameToSend,SUCCESS);
				endSlot();
			} else {
				call FastAlarm.start(TsRxAckDelay);
				change_state(S_TX_RXACKREADY);
			}
			break;
			case S_RX_TXACK: //[sendNowDone] receiver
			//I'm a receiver, finished sending ACK (end of RX sequence)
			if (error!=SUCCESS) {
				//don't do anything if error==FAIL

			}
			/* //sync off of DATA I received before I sent ACK
			 * poipoi for simplicity, only resync from ADV
			 resynchronize(FRAME_BASED_RESYNC,
			 &(frameReceived->l2_nextORpreviousHop),
			 frameReceived->l1_rxTimestamp,
			 (int16_t)((int32_t)(frameReceived->l1_rxTimestamp)-(int32_t)radio_delay)-(int32_t)TsTxOffset);*/
			post taskReceive();
			endSlot();
			break;
			default:
			endSlot();
			break;
		}
	}

	void postTaskSendDone(OpenQueueEntry_t* param_sendDoneMessage,
			error_t param_sendDoneError) {
		atomic {

			sendDoneMessage = param_sendDoneMessage;
			sendDoneError = param_sendDoneError;
		}
		// Commented for the TSCH Test Experiment
		post taskSendDone();
		////////////////////////////////////////
	}

	task void taskSendDone() {
		OpenQueueEntry_t* temp_sendDoneMessage;
		error_t temp_sendDoneError;
		atomic {
			temp_sendDoneMessage = sendDoneMessage;
			temp_sendDoneError = sendDoneError;
		}

		signal OpenSendFromUpper.sendDone(temp_sendDoneMessage,temp_sendDoneError);
		atomic sendDoneMessage = NULL;
	}

	void endSlot() {
		asn_t temp_asn;
		uint8_t temp_state;
		atomic {
			temp_asn = asn;
			temp_state = state;
			if (packetACK!=NULL) {
				call Malloc.freePacketBuffer(packetACK);
				packetACK=NULL;
			}
		}
		if (call RadioControl.rfOff()!=SUCCESS) {
			// Abort
		}
		change_state(S_SLEEP);
	}

	/*------------------------------ reception -----------------------------------------*/

	//RadioReceive
	async event void RadioReceive.receive(OpenQueueEntry_t* msg) {
		asn_t temp_asn;
		uint8_t temp_state;
		bool temp_isSync;
		OpenQueueEntry_t* temp_dataFrameToSend;
		ieee802154_header_iht received_ieee154_header;
		ieee802154_header_iht transmitted_ieee154_header;

		uint8_t temp_init;
		uint8_t temp_numADV;

		atomic {
			temp_asn = asn;
			temp_isSync = isSync;
			temp_state = state;
			temp_dataFrameToSend = dataFrameToSend;
			temp_init=init;
			temp_numADV=numADV;
		}
		msg->owner = COMPONENT_MAC;

		received_ieee154_header = retrieveIEEE802154header(msg);
		call PacketFunctions.tossHeader(msg,received_ieee154_header.headerLength);
		call PacketFunctions.tossFooter(msg,2);

		msg->l2_frameType = received_ieee154_header.frameType;
		memcpy(&(msg->l2_nextORpreviousHop),&(received_ieee154_header.src),sizeof(open_addr_t));

		if (received_ieee154_header.frameType==IEEE154_TYPE_DATA &&
				!(call IDManager.isMyAddress(&received_ieee154_header.panid))) {
			call Malloc.freePacketBuffer(msg);
			return;
		}

		switch (temp_state) {

			/*------------------- TX sequence ------------------------*/
			case S_TX_RXACK: //[receive] transmitter
			//I'm a transmitter, just received ACK (end of TX sequence)
			transmitted_ieee154_header = retrieveIEEE802154header(temp_dataFrameToSend);
			if (received_ieee154_header.dsn == transmitted_ieee154_header.dsn) {
				//I'm a transmitter, sync off of ACK message
				/* poipoi for simplicity, only resync from ADV
				 resynchronize(ACK_BASED_RESYNC,
				 &received_ieee154_header.src,
				 TsTxOffset,
				 (((IEEE802154E_ACK_ht*)(msg->payload))->timeCorrection));//poipoi /32  poipoipoipoi*/


				postTaskSendDone(temp_dataFrameToSend,SUCCESS);
			} else {


				temp_dataFrameToSend->l2_retriesLeft--;
				if (temp_dataFrameToSend->l2_retriesLeft==0) {
					postTaskSendDone(temp_dataFrameToSend,FAIL);
				}
			}
			call Malloc.freePacketBuffer(msg);//free ACK
			endSlot();
			break;

			/*------------------- RX sequence ------------------------*/
			case S_SYNCHRONIZING:
			case S_RX_RXDATA: //[receive] receiver
			//I'm a receiver, just received data
			if (call IDManager.isMyAddress(&(received_ieee154_header.dest)) && received_ieee154_header.ackRequested) {
				//				//ACK requested
				//
				//				if (call RadioControl.rfOff()!=SUCCESS) {
				//					//do nothing about it
				//				}
				//				call FastAlarm.start(TsTxAckDelay);
				//				change_state(S_RX_TXACKPREPARE);
				//				atomic {
				//					packetACK = call Malloc.getFreePacketBuffer();
				//					temp_packetACK = packetACK;
				//				}
				//				if (temp_packetACK==NULL) {
				//					call Malloc.freePacketBuffer(msg);
				//					endSlot();
				//					return;
				//				}
				//				temp_packetACK->creator = COMPONENT_MAC;
				//				temp_packetACK->owner = COMPONENT_MAC;
				//				//ACK payload
				//				call PacketFunctions.reserveHeaderSize(temp_packetACK,sizeof(IEEE802154E_ACK_ht));
				//				((IEEE802154E_ACK_ht*)(temp_packetACK->payload))->dhrAckNack = IEEE154E_ACK_dhrAckNack_DEFAULT;
				//				((IEEE802154E_ACK_ht*)(temp_packetACK->payload))->timeCorrection =
				//				(int16_t)((int32_t)(TsTxOffset+radio_delay)-(int32_t)(msg->l1_rxTimestamp));//poipoi *32
				//				//154 header
				//				prependIEEE802154header(temp_packetACK,
				//						IEEE154_TYPE_ACK,
				//						IEEE154_SEC_NO_SECURITY,
				//						received_ieee154_header.dsn,
				//						NULL
				//				);
				//				//l2 metadata
				//				temp_packetACK->l2_retriesLeft = 1;
				//				//l1_metadata
				//				temp_packetACK->l1_txPower = TX_POWER;
				//				atomic temp_packetACK->l1_channel = frequencyChannel;
				//				atomic temp_error = call RadioSend.prepareSend(temp_packetACK);
				//				if (temp_error!=SUCCESS) {
				//					//abort
				//					endSlot();
				//				};

				atomic frameReceived = msg;
				post taskReceive();
				endSlot();

			} else if (call PacketFunctions.isBroadcastMulticast(&(received_ieee154_header.dest))) {
				if (received_ieee154_header.frameType==IEEE154_TYPE_CMD &&
						((CMD_MSG_t*)(msg->payload))->commandFrameId==SCAN_MSG) {
					if (temp_isSync==FALSE) {
						atomic asn = ((CMD_MSG_t*)(msg->payload))->timingInformation;
						slotType = UPLINK;
					}
					resynchronize(FRAME_BASED_RESYNC,
							&received_ieee154_header.src,
							msg->l1_rxTimestamp,
							(int16_t)((int32_t)(msg->l1_rxTimestamp)-(int32_t)radio_delay)-(int32_t)TsTxOffset);
				}
				atomic frameReceived = msg;
				post taskReceive();
				endSlot();

			} else {
				// This should not happen with Hardware Address recognition
				call Malloc.freePacketBuffer(msg);
				endSlot();
			}
			break;
			default:
			call Malloc.freePacketBuffer(msg);
			endSlot();
			break;
		}
	}

	task void taskReceive() {
		asn_t temp_asn;
		OpenQueueEntry_t* temp_frameReceived;
		atomic {
			temp_asn = asn;
			temp_frameReceived = frameReceived;
		}
		call OpenReceiveToUpper.receive(temp_frameReceived);
	}

	/*------------------------------ resynchronization ---------------------------------*/
	void resynchronize(bool resyncType, open_addr_t* node_id,
			timervalue_t dataGlobalSlotOffset, int16_t timeCorrection) {

		bool temp_isSync;

		bool iShouldSynchronize;

		atomic {
			temp_isSync = isSync;
		}

		if ((call IDManager.getIsDAGroot())==FALSE) { //I resync only if I'm not a DAGroot
			//---checking whether I should synchronize

			iShouldSynchronize = FALSE;
			if (temp_isSync == FALSE) {

				if (resyncType == FRAME_BASED_RESYNC) {
					iShouldSynchronize = TRUE;
					atomic isSync = TRUE;
					//					call OpenSerial.printError(COMPONENT_MAC,ERR_ACQUIRED_SYNC,0,0);//not an error!
					call Leds.led2On();
				}
				//			}

				//				else { //I'm already synchronized
				//				call NeighborGet.getPreferredParent(&timeParent,node_id->type);
				//				if (timeParent.type!=ADDR_NONE) { //I have a timeparent, I sync off of any packet from it
				//					if (call PacketFunctions.sameAddress(&timeParent,node_id)) {
				//						iShouldSynchronize=TRUE;
				//					}
				//				} else { //I don't have a timeparent, I sync off of any packet
				//					iShouldSynchronize=TRUE;
				//				}
				//			}
				//---synchronize iif I need to
				if (iShouldSynchronize==TRUE) {
					atomic {
						if (resyncType==FRAME_BASED_RESYNC) {
							if (dataGlobalSlotOffset!=INVALID_TIMESTAMP) {
								if (slotAlarmStartSlotTimestamp+dataGlobalSlotOffset<call SlotAlarm.getNow()) {
									call SlotAlarm.startAt((uint32_t)((int32_t)slotAlarmStartSlotTimestamp+(int32_t)timeCorrection),SLOT_TIME);
								} else {
									atomic isSync=FALSE;

									//								atomic call OpenSerial.printError(COMPONENT_MAC,ERR_SYNC_RACE_CONDITION,
									//										(errorparameter_t)asn%LENGTHCELLFRAME,
									//										(errorparameter_t)dataGlobalSlotOffset);

									call Leds.led2Off();
									call SlotAlarm.startAt((call SlotAlarm.getNow())-(SLOT_TIME/2),SLOT_TIME);
									endSlot();

								}
							}
						} else {
							call SlotAlarm.startAt((uint32_t)((int32_t)slotAlarmStartSlotTimestamp+(int32_t)timeCorrection),SLOT_TIME);
						}
					}
					//				 call SlotAlarm.startAt((uint32_t)((int32_t)slotAlarmStartSlotTimestamp+(int32_t)timeCorrection),SLOT_TIME);

					// Commented for the TSCH Test Experiment
					//				post taskResetLosingLostTimers();
					/////////////////////////////////////////
				}
			}
		}
	}

	/*------------------------------ misc ----------------------------------------------*/

	//SoftwareInit
	command error_t SoftwareInit.init() {
		change_state(S_SLEEP);
		atomic dataFrameToSend = NULL;
		atomic asn = 0;
		//WDT configuration
		WDTCTL = WDTPW + WDTHOLD;
		WDTCTL = WDTPW + WDTCNTCL + WDTSSEL;//run from ACLK, ~1s
		return SUCCESS;
	}

	//GlobalTime
	async command timervalue_t GlobalTime.getGlobalSlotOffset() {
		//SlotAlarm.getNow() is epoch of now
		//(SlotAlarm.getAlarm()-SLOT_TIME) is epoch of the start of the slot
		//(call SlotAlarm.getNow())-(SlotAlarm.getAlarm()-SLOT_TIME) is the time since start of cell
		return ((call SlotAlarm.getNow())-((call SlotAlarm.getAlarm())-(uint32_t)SLOT_TIME));
	}
	async command timervalue_t GlobalTime.getLocalTime() {
		atomic return (call SlotAlarm.getNow());
	}
	async command asn_t GlobalTime.getASN() {
		atomic return asn;
	}

	async command bool GlobalSync.getIsSync() {
		atomic return isSync;
	}

	void change_state(uint8_t newstate) {
		atomic state = newstate;
		switch (newstate) {
			case S_SYNCHRONIZING:
			case S_TX_TXDATA:
			case S_TX_RXACK:
			case S_RX_RXDATA:
			case S_RX_TXACK:
			break;
			case S_TX_TXDATAPREPARE:
			case S_TX_TXDATAREADY:
			case S_TX_RXACKPREPARE:
			case S_TX_RXACKREADY:
			case S_RX_RXDATAPREPARE:
			case S_RX_RXDATAREADY:
			case S_RX_TXACKPREPARE:
			case S_RX_TXACKREADY:
			case S_SLEEP:
			break;
		}
	}
	async command void MAC.changeNetworkState(uint8_t tmp_networkState) {
		atomic networkState = tmp_networkState;
		call Leds.led0On();
	}
}
