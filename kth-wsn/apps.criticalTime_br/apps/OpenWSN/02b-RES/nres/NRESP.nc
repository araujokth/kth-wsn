#include "OpenWSN.h"
#include "IEEE802154E.h"

module NRESP {
	provides interface Init as SoftwareInit;
	//down the stack
	provides interface OpenSend as OpenSendFromUpper;
	provides interface OpenSendADV as OpenSendFromUpperADV;
	//   provides interface OpenSend as OpenSendFromBridge;
	provides interface OpenSend as OpenSendKAFromNeighbors;
	uses interface OpenSend as OpenSendToLower;
	//up the stack
	provides interface OpenReceive as OpenReceiveFromLower;
	uses interface OpenReceive as OpenReceiveADVToNeighbor;
	uses interface OpenReceive as OpenReceiveToUpper;
	//   uses     interface OpenReceive as OpenReceiveToBridge;
	//misc
	uses interface NeighborGet; //for getMyDAGrank
	uses interface Malloc;
	//   uses interface OpenSerial;
	uses interface IDManager;
	provides interface DebugPrint;
	//private
	uses interface Timer<TMilli> as timerADV;
	uses interface PacketFunctions;
	uses interface Random;

	//Debug
	uses interface Leds;
	uses interface OpenQueue;

	// TSCH Critical Time interfaces
	provides interface ControlADV;
}
implementation {
	/*-------------------------------- variables -----------------------------------------*/

	bool busyADV;
	//timervalue_t      periodADV = (SLOT_TIME/32)*LENGTHCELLFRAME*2;
	timervalue_t periodADV = 250;//poipoi
	uint8_t tokenId;



	/*-------------------------------- prototypes ----------------------------------------*/

	task void taskPrintDebug();
	task void taskBuildAndSendAdv();
	error_t sendToLower(OpenQueueEntry_t *msg);

	/*-------------------------------- helper functions ----------------------------------*/

	task void taskPrintDebug() {
		uint16_t output=0;
		output = call NeighborGet.getMyDAGrank();
		//      call OpenSerial.printStatus(STATUS_ADVERTISEP_DAGRANK,(uint8_t*)&output,1);
	}

	task void taskBuildAndSendAdv() {
		OpenQueueEntry_t* pkt;

		open_addr_t temp_nextHop;
		uint8_t temp_NumNeighbors;
		// Extended Address Mode
//		temp_nextHop.type = ADDR_64B;
//		temp_nextHop.addr_64b[0] = 0xFF;
//		temp_nextHop.addr_64b[1] = 0xFF;
//		temp_nextHop.addr_64b[2] = 0xFF;
//		temp_nextHop.addr_64b[3] = 0xFF;
//		temp_nextHop.addr_64b[4] = 0xFF;
//		temp_nextHop.addr_64b[5] = 0xFF;
//		temp_nextHop.addr_64b[6] = 0xFF;
//		temp_nextHop.addr_64b[7] = 0xFF;
		// Short Address Mode
		temp_nextHop.type = ADDR_16B;
		temp_nextHop.addr_16b[0] = 0xFF;
		temp_nextHop.addr_16b[1] = 0xFF;
		pkt = call Malloc.getFreePacketBuffer();
		if (pkt==NULL) {

			//         call OpenSerial.printError(COMPONENT_RES,ERR_NO_FREE_PACKET_BUFFER,0,0);
			call Leds.led2Toggle();
			return;
		}
		pkt->creator = COMPONENT_RES;
		pkt->owner = COMPONENT_RES;

		//ADV header
		call PacketFunctions.reserveHeaderSize(pkt,sizeof(IEEE802154E_ADV_t));
		((IEEE802154E_ADV_t*)(pkt->payload))->commandFrameId = IEEE154E_ADV;
		//timing information is filled out in IEEE802154E
		((IEEE802154E_ADV_t*)(pkt->payload))->securityControlField = IEEE154E_ADV_SEC_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->joinControl = IEEE154E_ADV_JOINCONTROL_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->timeslotHopping = IEEE154E_ADV_HOPPING_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->channelPage = IEEE154E_ADV_CHANNELPAGE_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->channelMap = IEEE154E_ADV_CHANNELMAP_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->numberSlotFrames = IEEE154E_ADV_NUMSLOTFRAMES_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->slotFrameID = IEEE154E_ADV_SLOTFRAMEID_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->slotFrameSize = IEEE154E_ADV_SLOTFRAMESIZE_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->numberLinks = IEEE154E_ADV_NUMLINKS_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->linkInfo1 = IEEE154E_ADV_LINKINFO1_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->linkInfo2 = IEEE154E_ADV_LINKINFO2_DEFAULT;
		((IEEE802154E_ADV_t*)(pkt->payload))->DAGrank = call NeighborGet.getMyDAGrank();
		//l2 header
		memcpy(&(pkt->l2_nextORpreviousHop),&temp_nextHop,sizeof(open_addr_t));
		pkt->l2_frameType = IEEE154_TYPE_CMD;
		//send


		if ((call OpenSendToLower.send(pkt))==FAIL) {
			call Malloc.freePacketBuffer(pkt);
			atomic busyADV=FALSE;
		}
		call Leds.led0Toggle();
		//set a new random periodADV
		temp_NumNeighbors = call NeighborGet.getNumNeighbors();
		if (temp_NumNeighbors>1) {
			//periodADV  = (SLOT_TIME/32)*LENGTHCELLFRAME*temp_NumNeighbors;poipoi
		}
		//      call timerADV.startPeriodic(192000);
	}

	/*-------------------------------- interfaces ----------------------------------------*/

	//SoftwareInit
	command error_t SoftwareInit.init() {
		atomic busyADV = FALSE;
		//      call timerADV.startPeriodic(periodADV);

		return SUCCESS;
	}

	//OpenSendFromUpper
	command error_t OpenSendFromUpper.send(OpenQueueEntry_t *msg) {
		msg->owner = COMPONENT_RES;
//		if (call IDManager.getIsBridge()==TRUE) {
//			//         call OpenSerial.printError(COMPONENT_RES,ERR_BRIDGE_MISMATCH,0,0);
//			return FAIL;
//		}
		call Leds.led0Toggle();
		return sendToLower(msg);
	}

	//OpenSendFromBridge
	//   command error_t OpenSendFromBridge.send(OpenQueueEntry_t *msg) {
	//      msg->owner = COMPONENT_RES;
	//      if (call IDManager.getIsBridge()==FALSE) {
	////         call OpenSerial.printError(COMPONENT_RES,ERR_BRIDGE_MISMATCH,1,0);
	//         return FAIL;
	//      }
	//      return sendToLower(msg);
	//   }

	//OpenSendKAFromNeighbors
	command error_t OpenSendKAFromNeighbors.send(OpenQueueEntry_t *msg) {
		msg->owner = COMPONENT_RES;
		return sendToLower(msg);
	}

	error_t sendToLower(OpenQueueEntry_t *msg) {
		msg->l2_frameType = IEEE154_TYPE_DATA;
		return call OpenSendToLower.send(msg);
	}

	//OpenSendToLower
	event void OpenSendToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {
		bool temp_busyADV;
		atomic {
			temp_busyADV = busyADV;
		}
		msg->owner = COMPONENT_RES;
		switch (msg->creator) {
			case COMPONENT_RES:
			//this is my ADV packet
			if (temp_busyADV==FALSE) {
				//               call OpenSerial.printError(COMPONENT_RES,ERR_SENDDONE_WHILE_NOT_BUSY,0,0);
			}

			call Malloc.freePacketBuffer(msg);

			atomic busyADV = FALSE;
			break;
			case COMPONENT_NEIGHBORS:
			//this is a KA packet
			signal OpenSendKAFromNeighbors.sendDone(msg,error);
			break;
			default:
			//this is a packet from upper layers
			if (call IDManager.getIsBridge()==TRUE) {
				//               signal OpenSendFromBridge.sendDone(msg,error);
			} else {
				signal OpenSendFromUpper.sendDone(msg,error);
			}
			break;
		}
	}

	//OpenReceiveFromLower
	command void OpenReceiveFromLower.receive(OpenQueueEntry_t* msg) {
		msg->owner = COMPONENT_RES;
		call Leds.led1Toggle();
		switch (msg->l2_frameType) {
			case IEEE154_TYPE_DATA:


			if (call IDManager.getIsBridge()==TRUE) {
				//               call OpenReceiveToBridge.receive(msg);     //out to the OpenLBR stack
			} else {
				// Commented for the TSCH Test Experiment
				call Malloc.freePacketBuffer(msg);
//				call OpenReceiveToUpper.receive(msg); //up the internal stack
				////////////////////////////////////////
			}
			break;
			case IEEE154_TYPE_CMD:
			if (((IEEE802154E_ADV_t*)(msg->payload))->commandFrameId==IEEE154E_ADV) {
//				call OpenReceiveADVToNeighbor.receive(msg); //to NeighborsC

				call OpenReceiveToUpper.receive(msg);
				return;
			} else {
				call Malloc.freePacketBuffer(msg);
				//               call OpenSerial.printError(COMPONENT_RES,ERR_MSG_UNKNOWN_TYPE,msg->l2_frameType,0);
			}
			break;
			default:
			call Malloc.freePacketBuffer(msg);
			//            call OpenSerial.printError(COMPONENT_RES,ERR_MSG_UNKNOWN_TYPE,msg->l2_frameType,0);
			break;
		}
	}

	//DebugPrint
	command void DebugPrint.print() {
		post taskPrintDebug();
	}

	//timerADV
	event void timerADV.fired() {
		bool temp_busyADV;

		atomic {
			temp_busyADV = busyADV;
		}
		if (temp_busyADV==TRUE) {
			return;
		}
		atomic busyADV = TRUE;
		if ((call IDManager.getIsDAGroot())==TRUE) {
			post taskBuildAndSendAdv();
		}
	}

	command void OpenSendFromUpperADV.sendADV(uint8_t temp_tokenId) {
		atomic tokenId = temp_tokenId;
		atomic busyADV = FALSE;
		call timerADV.startPeriodic(periodADV);
	}
	command void ControlADV.stopADV() {
		call timerADV.stop();
	}
}
