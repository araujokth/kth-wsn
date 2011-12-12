#include "OpenWSN.h"
#include "printf.h"

module appDATADevP {

	provides interface Init as SoftwareInit;
	provides interface OpenReceive as OpenReceiveFromLower;

	// TSCH Critical Time interfaces
	provides interface CriticalTime;

	uses interface OpenSend as OpenSendToLower;
	uses interface OpenSendADV as OpenSendToLowerADV;
	uses interface Malloc;

	uses interface Leds;

	uses interface NeighborGet;

	uses interface PacketFunctions;

	uses interface IDManager;

	uses interface SplitControl as SerialControl;
	uses interface AMSend as UartSend[am_id_t id];
	uses interface Receive as UartReceive;
	uses interface AMPacket as UartAMPacket;
	uses interface Packet;

	// TSCH Critical Time interfaces
	uses interface ControlSync;
	uses interface ControlADV;
	uses interface Read<uint16_t> as ReadTemp;
	uses interface Timer<TMilli> as timerCriticalTime;
}

implementation {
	/*-------------------------------- variables -----------------------------------------*/
	uint32_t ackSUC=0;
	uint32_t numPackets=0;
	uint8_t ackFAIL=0;
	uint8_t init=0;
	open_addr_t temp_Hop;
	uint32_t timestampStart;
	uint8_t numADV = 0;
	/*-------------------------------- prototypes ----------------------------------------*/
	void taskBuildAndSendPkt();
	/*-------------------------------- http receive sequence -----------------------------*/

	/*-------------------------------- interfaces ----------------------------------------*/
	message_t m_frameToken;
	uint8_t m_payloadLenTranfer;
	TokenRequestMsg* tokenRequest;
	TokenTransferredMsg* tokenReceived;
	message_t m_ackFrame;
	uint8_t m_payloadLenReceived;
	TokenTransferredMsg* tokenAckSent;
	task void transferTokenTo();
	task void sendAckBackTo();
	uint16_t tokenId;

	//SoftwareInit
	command error_t SoftwareInit.init() {
		m_payloadLenTranfer = sizeof(TokenRequestMsg);
		tokenRequest = (TokenRequestMsg* ) call Packet.getPayload(&m_frameToken, m_payloadLenTranfer);
		tokenRequest->tokenId = TOS_NODE_ID;
		call UartAMPacket.setSource(&m_frameToken, TOS_NODE_ID);

		m_payloadLenReceived = sizeof(TokenTransferredMsg);
		tokenAckSent = (TokenTransferredMsg* ) call Packet.getPayload(&m_ackFrame, m_payloadLenReceived);
		call UartAMPacket.setSource(&m_ackFrame, TOS_NODE_ID);

		if (call SerialControl.start() != SUCCESS ) {
			//call Leds.led0On(); // error
		}

		return SUCCESS;
	}

	event void SerialControl.startDone(error_t error) {
		if (error != SUCCESS) {
			//call Leds.led0On(); // error
		} else {
			//call MLME_RESET.request(TRUE);
		}

	}
	event void SerialControl.stopDone(error_t error) {}
	command void OpenReceiveFromLower.receive(OpenQueueEntry_t* pktRx) {
		call Malloc.freePacketBuffer(pktRx);
	}

	event void OpenSendToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {
		msg->owner = COMPONENT_APPDATA;
		call Malloc.freePacketBuffer(msg);
		atomic P3OUT &= ~0x10;
		if (tokenId==TOS_NODE_ID)
		{
			taskBuildAndSendPkt();
		}

	}

	event void ReadTemp.readDone(error_t result, uint16_t data)
	{
//		if (result == SUCCESS) {
//			tokenAckSent->temperature=data;
//			post sendAckBackTo();
//		}else{
//			call ReadTemp.read();
//		}
		tokenAckSent->temperature=data;
		post sendAckBackTo();

	}

	command void CriticalTime.sendCriticalTime (uint32_t criticalTime)
	{
		tokenRequest->criticalTime=criticalTime;
		//		post transferTokenTo();
		call timerCriticalTime.startPeriodic(3200);

	}

	command void CriticalTime.syncDone ()
	{
		// Pseudo code
		tokenAckSent->cmd = ACK_COMMAND;
		tokenAckSent->destinationId = TOS_NODE_ID;
		tokenAckSent->tokenId = SYNC_DONE;
		// Read Temperature
		call ReadTemp.read();
		//post sendAckBackTo();

	}

	void taskBuildAndSendPkt() {
		OpenQueueEntry_t* pktToTx;
		open_addr_t temp_nextHop;
		// Extended Address Mode
		//		temp_nextHop.type = ADDR_64B;
		//		temp_nextHop.addr_64b[0] = 0x00;
		//		temp_nextHop.addr_64b[1] = 0x00;
		//		temp_nextHop.addr_64b[2] = 0x00;
		//		temp_nextHop.addr_64b[3] = 0x00;
		//		temp_nextHop.addr_64b[4] = 0x00;
		//		temp_nextHop.addr_64b[5] = 0x00;
		//		temp_nextHop.addr_64b[6] = 0x00;
		//		temp_nextHop.addr_64b[7] = tokenReceived->destinationId;
		// Short Address Mode
		temp_nextHop.type = ADDR_16B;
		temp_nextHop.addr_16b[0] = 0xFF;
		temp_nextHop.addr_16b[1] = 0xFF;

		//			  call NeighborGet.getPreferredParent(&temp_nextHop,ADDR_64B);


		pktToTx = call Malloc.getFreePacketBuffer();
		if (pktToTx==NULL) {

			return;
		}

		pktToTx->creator=COMPONENT_APPDATA;
		pktToTx->owner = COMPONENT_APPDATA;

		call PacketFunctions.reserveHeaderSize(pktToTx,39);

		//l2 header
		memcpy(&(pktToTx->l2_nextORpreviousHop),&temp_nextHop,sizeof(open_addr_t));
		//			  pktToTx->l2_frameType=IEEE154_TYPE_DATA;
		//send
		if ((call OpenSendToLower.send(pktToTx))==FAIL) {
			call Malloc.freePacketBuffer(pktToTx);
		}
	}
	/****************************************************************************
	 * S E R I A L   T O   M A N A G E   T H E   T O K E N
	 */
	// Send the new sensor values to the PC
	task void transferTokenTo() {
		if (call UartSend.send[AM_TOKENREQUESTMSG](AM_BROADCAST_ADDR, &m_frameToken, m_payloadLenTranfer ) == SUCCESS) {
			//call Leds.led1Toggle();
		} else {
		}
	}

	task void sendAckBackTo() {
		if (call UartSend.send[AM_TOKENTRANSFERREDMSG](AM_BROADCAST_ADDR, &m_ackFrame, m_payloadLenReceived ) == SUCCESS) {
			//call Leds.led1Toggle();

		} else {
			//call Leds.led0Toggle();
		}
	}

	event message_t *UartReceive.receive(message_t *msg,
			void *payload,
			uint8_t len) {

		atomic {

			if (call UartAMPacket.type(msg) == AM_TOKENTRANSFERREDMSG) {
				uint8_t cmd;

				tokenReceived = (TokenTransferredMsg*) call Packet.getPayload(msg, m_payloadLenReceived);
				cmd = tokenReceived->cmd;

				atomic tokenRequest->criticalTime = 0;
				//printf("Token received [%u]: src=%u dst=%u \n", cmd, tokenReceived->tokenId, tokenReceived->destinationId);printfflush();

				switch(cmd) {

					case CHANGE_COMMAND:
					call Leds.led0Off();
					call Leds.led1Off();
					call Leds.led2Off();

					tokenId = tokenReceived->tokenId;
					//				call ControlSync.setIsSync (FALSE);


					if (tokenId == TOS_NODE_ID)
					{
						//					call Leds.led0On();
						call ControlSync.setIsSync (TRUE);
					} else {
						//					call Leds.led1On();
						call ControlSync.setIsSync (FALSE);
					}
					tokenAckSent->cmd = ACK_COMMAND;
					tokenAckSent->destinationId = TOS_NODE_ID;
					tokenAckSent->tokenId = TOS_NODE_ID;
					post sendAckBackTo();

					break;

					case START_ADV:

					tokenAckSent->cmd = ACK_COMMAND;
					tokenAckSent->destinationId = TOS_NODE_ID;
					tokenAckSent->tokenId = TOS_NODE_ID;
					post sendAckBackTo();
					call OpenSendToLowerADV.sendADV(tokenId);
					break;

					case START_DATA:

					call ControlADV.stopADV();
					tokenAckSent->cmd = ACK_COMMAND;
					tokenAckSent->destinationId = TOS_NODE_ID;
					tokenAckSent->tokenId = TOS_NODE_ID;
					post sendAckBackTo();
					taskBuildAndSendPkt();
					break;
					default:
					tokenAckSent->cmd = ACK_COMMAND;
					tokenAckSent->destinationId = TOS_NODE_ID;
					tokenAckSent->tokenId = TOS_NODE_ID;
					post sendAckBackTo();
					break;
				}

			}
		}
		return msg;

	}
	event void UartSend.sendDone[am_id_t id](message_t* msg, error_t error) {
		//if (error != SUCCESS) call Leds.led0Toggle();
	}
	event void timerCriticalTime.fired() {
		call timerCriticalTime.stop();
		post transferTokenTo();
	}
}
