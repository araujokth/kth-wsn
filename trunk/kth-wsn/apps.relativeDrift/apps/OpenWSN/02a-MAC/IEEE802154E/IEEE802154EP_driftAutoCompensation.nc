#include "OpenWSN.h"
#include "CC2420.h"
#include "IEEE802154E.h"
#include "printf.h"
module IEEE802154EP {
	//admin
	uses interface Boot;
	provides interface Init as SoftwareInit;
	//time

	uses interface Alarm<T32khz,uint32_t> as FastAlarm; //private
	uses interface Alarm<T32khz,uint32_t> as SlotAlarm; //private
	uses interface Timer<TMilli> as LosingSyncTimer; //private
	uses interface Timer<TMilli> as LostSyncTimer; //private
	provides interface GlobalTime;
	provides interface GlobalSync;
	provides interface ControlSync;
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
	uses interface CellStats;
	uses interface PacketFunctions;
	uses interface NeighborStats;
	uses interface Malloc;
	uses interface CellUsageGet;
	uses interface IDManager;
	uses interface NeighborGet;
	uses interface Random;

	// TSCH Critical Time interfaces
	uses interface CriticalTime;
	uses interface Timer<TMilli> as CriticalTimeTimer;

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

	// Relative Drift Compensation
	uint32_t timestampY1;
	uint32_t timestampY2;
	uint32_t timestampX1;
	uint32_t timestampX2;
	float relativeDrift_a;
	float tmpNumSlotsToAdjust;
	uint32_t numSlotsToAdjust;
	uint32_t counterSlotsToAdjust;
	uint32_t numSlotsToReAdjust;
	uint32_t counterSlotsToReAdjust;

	bool beacon_channel;
	bool change;
	bool change_IsDAG;
	bool compensateDrift;
	/*------------------------------ prototypes ------------------------------------------*/
#include "IEEE802154_common.c"
	void change_state(uint8_t newstate);
	void resynchronize(bool resyncType, open_addr_t* node_id, timervalue_t dataGlobalSlotOffset, int16_t timeCorrection);
	void endSlot();
	task void taskResetLosingLostTimers();
	//the two following tasks are used to break the asynchronicity: everything in MAC and below is async, all the above not
	task void taskReceive();
	void postTaskSendDone(OpenQueueEntry_t* param_sendDoneMessage, error_t param_sendDoneError);
	task void taskSendDone();
	void printfFloat(float toBePrinted);
	uint32_t round(float number);
	/*------------------------------ start/stop sequence ---------------------------------*/
	event void Boot.booted() {
		atomic init=0;
		atomic numReceiveNothing=0;
		atomic numADV=0;
		atomic isSync=FALSE;
		atomic beacon_channel=TRUE;
		atomic change=FALSE;
		atomic compensateDrift = FALSE;
		numPrepareReceiveFailed=0;
		numSlotsInSynchStatus=0;
		call Leds.led2Off();
		call RadioControl.start();
		//		// Added for the TSCH Test Experiment
		//		temp_nextHop.type = ADDR_64B;
		//		temp_nextHop.addr_64b[0] = 0x00;
		//		temp_nextHop.addr_64b[1] = 0x00;
		//		temp_nextHop.addr_64b[2] = 0x00;
		//		temp_nextHop.addr_64b[3] = 0x00;
		//		temp_nextHop.addr_64b[4] = 0x00;
		//		temp_nextHop.addr_64b[5] = 0x00;
		//		temp_nextHop.addr_64b[6] = 0x00;
		//		temp_nextHop.addr_64b[7] = 0x00;
		//		pktToTx = call Malloc.getFreePacketBuffer();
		//		pktToTx->creator=COMPONENT_MAC;
		//		pktToTx->owner=COMPONENT_MAC;
		//		pktToTx->l2_frameType = IEEE154_TYPE_DATA;
		//		call PacketFunctions.reserveHeaderSize(pktToTx,27);
		//		memcpy(&(pktToTx->l2_nextORpreviousHop),&temp_nextHop,sizeof(open_addr_t));
		//		if (call PacketFunctions.isBroadcastMulticast(&(pktToTx->l2_nextORpreviousHop))==TRUE) {
		//			pktToTx->l2_retriesLeft = 1;
		//		} else {
		//			pktToTx->l2_retriesLeft = TXRETRIES;
		//		}
		//		pktToTx->l1_txPower = TX_POWER;
		//		prependIEEE802154header(pktToTx,
		//			pktToTx->l2_frameType,
		//			IEEE154_SEC_NO_SECURITY,
		//			dsn++,
		//			&(pktToTx->l2_nextORpreviousHop)
		//		);
		//////////////////////////////////////
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
		//open_addr_t             temp_addr_16b;
		//open_addr_t             temp_addr_64b;
		ieee802154_header_iht transmitted_ieee154_header;
		uint8_t temp_channelOffset;
		uint32_t tmpDrift=0;
		call PinDebug.ADC0toggle();


		fastAlarmStartSlotTimestamp = call FastAlarm.getNow();
		slotAlarmStartSlotTimestamp = call SlotAlarm.getNow();
		atomic asn++;

		if (compensateDrift) {
			counterSlotsToAdjust=counterSlotsToAdjust-1;
			counterSlotsToReAdjust=counterSlotsToReAdjust-1;
			if ((counterSlotsToAdjust==0)&&(counterSlotsToReAdjust==0)) {
				counterSlotsToAdjust=numSlotsToAdjust;
				counterSlotsToReAdjust=numSlotsToReAdjust;
				if (((float)(numSlotsToAdjust))<=tmpNumSlotsToAdjust) {
					if (relativeDrift_a>1) {
						call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME);
					} else {
						call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME);
					}
				} else if (((float)(numSlotsToAdjust))>tmpNumSlotsToAdjust) {
					if (relativeDrift_a>1) {
						call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME+2);
					} else {
						call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME-2);
					}
				}


			} else if ((counterSlotsToAdjust==0)&&(counterSlotsToReAdjust!=0)) {
				counterSlotsToAdjust=numSlotsToAdjust;
				if (relativeDrift_a>1) {
					call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME+1);
				} else {
					call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME-1);
				}

			} else if ((counterSlotsToReAdjust==0)&&(counterSlotsToAdjust!=0)) {
				counterSlotsToReAdjust=numSlotsToReAdjust;
				if (((float)(numSlotsToAdjust))<=tmpNumSlotsToAdjust) {
					if (relativeDrift_a>1) {
						call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME-1);
					} else {
						call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME+1);
					}
				} else if (((float)(numSlotsToAdjust))>tmpNumSlotsToAdjust) {
					if (relativeDrift_a>1) {
						call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME+1);
					} else {
						call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME-1);
					}
				}

			} else {
				call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME);

			}
		} else {
			call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME);
		}
//		if (compensateDrift) {
//			counterSlotsToAdjust=counterSlotsToAdjust-1;
//
//			if (counterSlotsToAdjust==0) {
//				counterSlotsToAdjust=numSlotsToAdjust;
//				call Leds.led0Toggle();
//				if (relativeDrift_a>1) {
//					call SlotAlarm.startAt((call SlotAlarm.getAlarm()),(SLOT_TIME+2));
//				} else {
//					call SlotAlarm.startAt((call SlotAlarm.getAlarm()),(SLOT_TIME-2));
//				}
//
//
//
//			} else {
//				call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME);
//
//			}
//		} else {
//			call SlotAlarm.startAt((call SlotAlarm.getAlarm()),SLOT_TIME);
//		}

		//reset WDT
		atomic WDTCTL = WDTPW + WDTCNTCL + WDTSSEL;

		atomic dataFrameToSend = NULL;

		atomic {
			if (change) {
				change=FALSE;
				call IDManager.setDAGRoot (change_IsDAG);
				isSync=FALSE;
				if ((call IDManager.getIsDAGroot())==FALSE) {
					call Leds.led2Off();
				}
				numADV=0;
				init=0;
				numReceiveNothing=0;
				beacon_channel=TRUE;
				numSlotsInSynchStatus=0;
				numPrepareReceiveFailed=0;
				compensateDrift=FALSE;
				numSlotsToAdjust=0;
				counterSlotsToAdjust=0;
			}
		}

		atomic {
			temp_asn = asn;
			temp_state = state;
			temp_isSync = isSync;
			temp_dataFrameToSend = dataFrameToSend;
		}

		//----- switch to/from S_SYNCHRONIZING
		if ((call IDManager.getIsDAGroot())==TRUE) {
			//if I'm DAGroot, I'm synchronized
			//			post taskResetLosingLostTimers();
			atomic isSync=TRUE;
			call Leds.led2On();
			atomic {
				if (state==S_SYNCHRONIZING) {//happens right after node becomes LBR
					endSlot();
					return;
				}
			}
		} else if (temp_isSync==FALSE) {
			//If I'm not in sync, enter/stay in S_SYNCHRONIZING


			if (temp_state!=S_SYNCHRONIZING) {
				numSlotsInSynchStatus=0;

				change_state(S_SYNCHRONIZING);

				if ((call RadioControl.prepareReceive(16))!=SUCCESS) {

					numPrepareReceiveFailed=numPrepareReceiveFailed+1;
					if (numPrepareReceiveFailed>=10)
					{
						numPrepareReceiveFailed=0;
						//						call RadioControl.start();
					}
					call Leds.led0On();
					call Leds.led1On();
					call Leds.led2On();
					endSlot();
				}
				else
				{
					numPrepareReceiveFailed=0;
				}
			}
			else {
				numSlotsInSynchStatus=numSlotsInSynchStatus+1;
				if (numSlotsInSynchStatus>=20)
				{
					numSlotsInSynchStatus=0;
					endSlot();
					//					call RadioControl.start();
				}
			}
			return;
		}

		//----- state error
		if (temp_state!=S_SLEEP) {
			endSlot();
			return;
		}

		switch (call CellUsageGet.getType(temp_asn%LENGTHCELLFRAME)) {
			case CELLTYPE_TXRX:
			//get a packet out of the buffer (if any)
			if ( call CellUsageGet.isTX(temp_asn%LENGTHCELLFRAME) || call CellUsageGet.isSH_TX(temp_asn%LENGTHCELLFRAME)) {
				atomic {
					dataFrameToSend = call OpenQueue.inQueue(call CellUsageGet.isADV(temp_asn%LENGTHCELLFRAME));
					temp_dataFrameToSend = dataFrameToSend;
				}
			}
			temp_channelOffset = call CellUsageGet.getChannelOffset(temp_asn%LENGTHCELLFRAME);
			if (HOPPING_ENABLED) {
				atomic frequencyChannel = ((temp_asn+temp_channelOffset)%8)+16;

			} else {
				atomic frequencyChannel =16;
			}

			if (temp_dataFrameToSend!=NULL) { //start the TX sequence
				dataFrameToSend->owner = COMPONENT_MAC;

				if(dataFrameToSend->creator==COMPONENT_RES) {
					atomic dataFrameToSend->l1_channel = 16;
				}
				else {
					atomic dataFrameToSend->l1_channel = frequencyChannel;
				}

				transmitted_ieee154_header = retrieveIEEE802154header(temp_dataFrameToSend);
				if (call CellUsageGet.isADV(temp_asn%LENGTHCELLFRAME)) {
					//I will be sending an ADV frame

					((IEEE802154E_ADV_t*)((dataFrameToSend->payload)+transmitted_ieee154_header.headerLength))->timingInformation=(call GlobalTime.getASN());
					((IEEE802154E_ADV_t*)((dataFrameToSend->payload)+transmitted_ieee154_header.headerLength))->timestampInformation=(call SlotAlarm.getNow());
				}
				change_state(S_TX_TXDATAPREPARE);
				atomic temp_error = call RadioSend.prepareSend(dataFrameToSend);
				if (temp_error!=SUCCESS) {
					//retry sending the packet later
					temp_dataFrameToSend->l2_retriesLeft--;
					if (temp_dataFrameToSend->l2_retriesLeft==0) {
						postTaskSendDone(temp_dataFrameToSend,FAIL);
					}
					endSlot();
				};
				atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsTxOffset);
			} else {
				if ((call IDManager.getIsDAGroot())==FALSE) {
					if (beacon_channel) {
						atomic frequencyChannel =16;
					}
				}
				if (call CellUsageGet.isRX(temp_asn%LENGTHCELLFRAME)) { //start the RX sequence
					change_state(S_RX_RXDATAPREPARE);
					atomic temp_error = call RadioControl.prepareReceive(frequencyChannel);
					if (temp_error!=SUCCESS) {
						//abort
						endSlot();
					};
					atomic call FastAlarm.startAt(fastAlarmStartSlotTimestamp,TsRxOffset);
				} else { //nothing to do, abort
					endSlot();
					return;
				}
			}
			return;
			break;

			case CELLTYPE_RXSERIAL:
			endSlot();
			return;

			case CELLTYPE_OFF:
			endSlot();
			return;

			default:
			endSlot();
			return;
		}
	}

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
			//abort
			//			call PinDebug.ADC2toggle();
			endSlot();
			return;
		}
		//		call PinDebug.ADC0toggle();
		switch (temp_state) {
			case S_TX_RXACKPREPARE:
			change_state(S_TX_RXACKREADY);
			break;
			case S_RX_RXDATAPREPARE:
			change_state(S_RX_RXDATAREADY);
			break;
			case S_SYNCHRONIZING:
			//			call PinDebug.ADC1toggle();
			if ( (call RadioControl.receiveNow(TRUE,3200))!=SUCCESS ) {
				//abort
				//				call PinDebug.ADC2toggle();
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
			if (temp_init==1) {
				atomic numReceiveNothing=numReceiveNothing+1;
				if (numReceiveNothing==3) {
					atomic criticalTime=((call CriticalTimeTimer.getNow())-timestampStart);
					call CriticalTime.sendCriticalTime(criticalTime);
					atomic init=2;
				}
			}
			endSlot();
			break;
			case S_TX_RXACK: //[receivedNothing] transmitter (WARNING state)
			//I'm a transmitter, didn't receive ACK (end of TX sequence)
			call CellStats.indicateUse(temp_asn%LENGTHCELLFRAME,WAS_NOT_ACKED);
			call NeighborStats.indicateTx(&(temp_dataFrameToSend->l2_nextORpreviousHop),WAS_NOT_ACKED);
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
			if (call CellUsageGet.isADV(temp_asn%LENGTHCELLFRAME)==TRUE) {
				//ADV slot, don't have to listen for ACK
				call NeighborStats.indicateTx(&(temp_dataFrameToSend->l2_nextORpreviousHop),WAS_NOT_ACKED);
				call CellStats.indicateUse(temp_asn%LENGTHCELLFRAME,WAS_NOT_ACKED);
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
			//abort

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
		OpenQueueEntry_t* temp_packetACK;
		error_t temp_error;
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
				call CellStats.indicateUse(temp_asn%LENGTHCELLFRAME,WAS_ACKED);
				call NeighborStats.indicateTx(&(temp_dataFrameToSend->l2_nextORpreviousHop),WAS_ACKED);
				postTaskSendDone(temp_dataFrameToSend,SUCCESS);
			} else {
				call CellStats.indicateUse(temp_asn%LENGTHCELLFRAME,WAS_NOT_ACKED);
				call NeighborStats.indicateTx(&(temp_dataFrameToSend->l2_nextORpreviousHop),WAS_NOT_ACKED);
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
				//I'm a receiver, sync off of DATA I received iif ADV (I will not send an ACK)
				//				call Leds.led1Toggle();
				if (received_ieee154_header.frameType==IEEE154_TYPE_CMD &&
						((IEEE802154E_ADV_t*)(msg->payload))->commandFrameId==IEEE154E_ADV) {

					atomic numADV=numADV+1;
					if (numADV==1) {
						atomic timestampX1 = ((IEEE802154E_ADV_t*)(msg->payload))->timestampInformation;
						atomic timestampY1 = call SlotAlarm.getNow();
					}
					if (numADV==240) {
						atomic timestampX2 = ((IEEE802154E_ADV_t*)(msg->payload))->timestampInformation;
						atomic timestampY2 = call SlotAlarm.getNow();
						atomic relativeDrift_a = ((float)(timestampY2 - timestampY1))/((float)(timestampX2 - timestampX1));
						atomic tmpNumSlotsToAdjust = 1/(((float)(SLOT_TIME))*(1-relativeDrift_a));
						if (tmpNumSlotsToAdjust<0) {
							tmpNumSlotsToAdjust=tmpNumSlotsToAdjust*(-1);
						}
						numSlotsToAdjust=round(tmpNumSlotsToAdjust);
						counterSlotsToAdjust=numSlotsToAdjust;
						if (((float)numSlotsToAdjust)>=tmpNumSlotsToAdjust) {
							numSlotsToReAdjust=round(((float)numSlotsToAdjust)/((((float)numSlotsToAdjust))-tmpNumSlotsToAdjust));
						} else {
							numSlotsToReAdjust=round(((float)numSlotsToAdjust)/(tmpNumSlotsToAdjust-((float)numSlotsToAdjust)));
						}
						numSlotsToReAdjust=numSlotsToReAdjust*numSlotsToAdjust;
						counterSlotsToReAdjust=numSlotsToReAdjust;
																		printf("%lu ",timestampY2);
																		printf("%lu\n",timestampY1);
																		printf("%lu ",timestampX2);
																		printf("%lu\n",timestampX1);
						//						printfFloat(relativeDrift_a);
						//						printf("%lu ",numSlotsToAdjust);
						//						printf("%lu",numSlotsToReAdjust);
						//						printf("\n");
												printfflush();
						atomic compensateDrift = TRUE;
						call CriticalTime.syncDone();
						call Leds.led0Toggle();
					}
					atomic asn = ((IEEE802154E_ADV_t*)(msg->payload))->timingInformation;
					resynchronize(FRAME_BASED_RESYNC,
							&received_ieee154_header.src,
							msg->l1_rxTimestamp,
							(int16_t)((int32_t)(msg->l1_rxTimestamp)-(int32_t)radio_delay)-(int32_t)TsTxOffset);
				}
				else if (received_ieee154_header.frameType==IEEE154_TYPE_DATA)
				{
					atomic
					{
						if (numADV >=240 )
						{
							if (init==0)
							{
								init=1;
								beacon_channel = FALSE;
								resynchronize(FRAME_BASED_RESYNC,
										&received_ieee154_header.src,
										msg->l1_rxTimestamp,
										(int16_t)((int32_t)(msg->l1_rxTimestamp)-(int32_t)radio_delay)-(int32_t)TsTxOffset);

								timestampStart=call CriticalTimeTimer.getNow();
							}
							numReceiveNothing=0;
						}
					}
				}
				atomic frameReceived = msg;
				post taskReceive();
				endSlot();

			} else {
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
		call CellStats.indicateUse(temp_asn%LENGTHCELLFRAME,FALSE);
		call NeighborStats.indicateRx(&(temp_frameReceived->l2_nextORpreviousHop),temp_frameReceived->l1_rssi);
		call OpenReceiveToUpper.receive(temp_frameReceived);
	}

	/*------------------------------ resynchronization ---------------------------------*/
	void resynchronize(bool resyncType, open_addr_t* node_id,
			timervalue_t dataGlobalSlotOffset, int16_t timeCorrection) {

		bool temp_isSync;
		open_addr_t timeParent;
		bool iShouldSynchronize;

		atomic {
			temp_isSync = isSync;
		}

		if ((call IDManager.getIsDAGroot())==FALSE) { //I resync only if I'm not a DAGroot
			//---checking whether I should synchronize

			iShouldSynchronize = FALSE;
			//			if (temp_isSync==FALSE) { //I'm not synchronized, I sync off of all ADV packets

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

	/*------------------------------ misc ----------------------------------------------*/

	task void taskResetLosingLostTimers() {
		call LosingSyncTimer.startOneShot(DELAY_LOSING_NEIGHBOR_1KHZ);
		call LostSyncTimer.startOneShot(DELAY_LOST_NEIGHBOR_1KHZ);
	}

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

	//LosingSyncTimer
	event void LosingSyncTimer.fired() {
		signal GlobalSync.losingSync();
		//		call OpenSerial.printError(COMPONENT_MAC,ERR_LOSING_SYNC,0,0);
	}

	//LostSyncTimer
	event void LostSyncTimer.fired() {
		//		call OpenSerial.printError(COMPONENT_MAC,ERR_LOST_SYNC,0,0);
		atomic isSync=FALSE;
		call Leds.led2Off();
		signal GlobalSync.lostSync();
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

	command void ControlSync.setIsSync(bool temp_IsDAG )
	{
		atomic change_IsDAG=temp_IsDAG;
		atomic change=TRUE;

		//		atomic isSync=temp_IsSync;
		//		if ((call IDManager.getIsDAGroot())==FALSE) {
		//		call Leds.led2Off();
		//		}
		//		atomic{
		//			numADV=0;
		//			init=0;
		//			numReceiveNothing=0;
		//			beacon_channel=TRUE;
		//		}

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
	event void CriticalTimeTimer.fired() {
	}
	void printfFloat(float toBePrinted) {
		uint32_t fi, f0, f1, f2;
		char c;
		float f = toBePrinted;

		if (f < 0) {
			c = '-';
			f = -f;
		} else {
			c = ' ';
		}

		// integer portion.
		fi = (uint32_t) f;

		// decimal portion...get index for up to 3 decimal places.
		f = f - ((float) fi);
		f0 = f * 10;
		f0 %= 10;
		f1 = f * 100;
		f1 %= 10;
		f2 = f * 1000;
		f2 %= 10;
		printf("%c%ld.%d%d%d", c, fi, (uint8_t) f0, (uint8_t) f1, (uint8_t) f2);
	}
	uint32_t round(float number) {
		return (number >= 0) ? (uint32_t)(number + 0.5) : (uint32_t)(number - 0.5);
	}
}
