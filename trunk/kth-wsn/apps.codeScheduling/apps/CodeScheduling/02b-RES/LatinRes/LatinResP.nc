#include "LATIN.h"
#include "IEEE802154E.h"
#include "IEEE802154.h"
#include "Neighbors.h"
#include "UserButton.h"

#include "printf.h"

module LatinResP {
	provides interface Init as SoftwareInit;
	//down the stack
	provides interface OpenSend as OpenSendFromUpper;
	uses interface OpenSend as OpenSendToLower;
	//up the stack
	provides interface OpenReceive as OpenReceiveFromLower;
	uses interface OpenReceive as OpenReceiveToUpper;
	//misc
	uses interface Malloc;
	uses interface IDManager;
	uses interface Notify<button_state_t> as UserButton;
	uses interface OpenQueue;
	uses interface PacketFunctions;

	// Code Scheduling
	provides interface LatinMatrix;
	uses interface MAC;

	// Timers
	uses interface Timer<TMilli> as timerAdvWait;

	//Debug
	uses interface Leds;

}
implementation {
	/*-------------------------------- variables -----------------------------------------*/

	uint8_t state;
	neighborList_t neighborList;

	/*-------------------------------- prototypes ----------------------------------------*/

	task void taskBuildAndSendScanMsg();
	task void taskBuildAndSendAdvMsg();
	task void taskBuildAndSendReqMsg();
	task void taskBuildAndSendSeedMsgToChildren();
	task void taskBuildAndSendDataToChildren();
	void taskBuildAndSendConfMsg(OpenQueueEntry_t *msg);
	void updateNeighborList(OpenQueueEntry_t *msg);
	void registerNewNeighbor(OpenQueueEntry_t *msg);
	void computeBestParent();
	void updateChildrenList(OpenQueueEntry_t *msg);
	void resetNeighborList();
	//	void printfNeighbors(); // Added for debugging
	bool isThisRowMatching(open_addr_t* address, uint8_t rowNumber);
	error_t sendToLower(OpenQueueEntry_t *msg);

	/*-------------------------------- helper functions ----------------------------------*/

	task void taskBuildAndSendScanMsg() {
		OpenQueueEntry_t* pkt;
		open_addr_t temp_nextHop;

		// Short Address Mode
		temp_nextHop.type = ADDR_16B;
		temp_nextHop.addr_16b[0] = 0xFF;
		temp_nextHop.addr_16b[1] = 0xFF;

		pkt = call Malloc.getFreePacketBuffer();
		if (pkt==NULL) {
			return;
		}

		pkt->creator = COMPONENT_RES;
		pkt->owner = COMPONENT_RES;

		// Scan Message header
		call PacketFunctions.reserveHeaderSize(pkt,sizeof(CMD_MSG_t));
		((CMD_MSG_t*)(pkt->payload))->commandFrameId = SCAN_MSG;
		((CMD_MSG_t*)(pkt->payload))->seedNumber = TOS_NODE_ID;

		// l2 header
		memcpy(&(pkt->l2_nextORpreviousHop),&temp_nextHop,sizeof(open_addr_t));
		pkt->l2_seedNumber = 255;
		pkt->l2_frameType = IEEE154_TYPE_CMD;
		pkt->l2_commandFrameId = SCAN_MSG;

		// Send
		if ((call OpenSendToLower.send(pkt))==FAIL) {
			call Malloc.freePacketBuffer(pkt);
		}

	}

	task void taskBuildAndSendAdvMsg() {
		OpenQueueEntry_t* pkt;
		open_addr_t temp_nextHop;

		// Short Address Mode
		temp_nextHop.type = ADDR_16B;
		temp_nextHop.addr_16b[0] = 0xFF;
		temp_nextHop.addr_16b[1] = 0xFF;
		pkt = call Malloc.getFreePacketBuffer();
		if (pkt==NULL) {
			return;
		}

		pkt->creator = COMPONENT_RES;
		pkt->owner = COMPONENT_RES;

		// Advertisement Message header
		call PacketFunctions.reserveHeaderSize(pkt,sizeof(CMD_MSG_t));
		((CMD_MSG_t*)(pkt->payload))->commandFrameId = ADV_MSG;
		((CMD_MSG_t*)(pkt->payload))->seedNumber = TOS_NODE_ID;
		((CMD_MSG_t*)(pkt->payload))->numHops = neighborList.minHops+1;

		// l2 header
		memcpy(&(pkt->l2_nextORpreviousHop),&temp_nextHop,sizeof(open_addr_t));
		pkt->l2_seedNumber = 255;
		pkt->l2_frameType = IEEE154_TYPE_CMD;
		pkt->l2_commandFrameId = ADV_MSG;

		// Send
		if ((call OpenSendToLower.send(pkt))==FAIL) {
			call Malloc.freePacketBuffer(pkt);
		}
	}

	task void taskBuildAndSendReqMsg() {
		OpenQueueEntry_t* pkt;

		pkt = call Malloc.getFreePacketBuffer();
		if (pkt==NULL) {
			return;
		}

		pkt->creator = COMPONENT_RES;
		pkt->owner = COMPONENT_RES;

		// Request Message header
		call PacketFunctions.reserveHeaderSize(pkt,sizeof(CMD_MSG_t));
		((CMD_MSG_t*)(pkt->payload))->commandFrameId = REQ_MSG;
		((CMD_MSG_t*)(pkt->payload))->seedNumber = TOS_NODE_ID;

		// l2 header
		memcpy(&(pkt->l2_nextORpreviousHop),&(neighborList.neighbors[neighborList.parent].addr_16b),sizeof(open_addr_t));
		pkt->l2_seedNumber = neighborList.neighbors[neighborList.parent].seedNumber;
		pkt->l2_seedNumber = 255;
		pkt->l2_frameType = IEEE154_TYPE_CMD;
		pkt->l2_commandFrameId = REQ_MSG;

		// Send
		if ((call OpenSendToLower.send(pkt))==FAIL) {
			call Malloc.freePacketBuffer(pkt);
		}
	}

	void taskBuildAndSendConfMsg(OpenQueueEntry_t *msg) {
		OpenQueueEntry_t* pkt;

		pkt = call Malloc.getFreePacketBuffer();
		if (pkt==NULL) {
			return;
		}

		pkt->creator = COMPONENT_RES;
		pkt->owner = COMPONENT_RES;

		// Request Message header
		call PacketFunctions.reserveHeaderSize(pkt,sizeof(CMD_MSG_t));
		((CMD_MSG_t*)(pkt->payload))->commandFrameId = CONF_MSG;
		((CMD_MSG_t*)(pkt->payload))->seedNumber = TOS_NODE_ID;

		// l2 header
		memcpy(&(pkt->l2_nextORpreviousHop),&(msg->l2_nextORpreviousHop),sizeof(open_addr_t));
		pkt->l2_seedNumber =((CMD_MSG_t*)(msg->payload))->seedNumber;
		pkt->l2_frameType = IEEE154_TYPE_CMD;
		pkt->l2_commandFrameId = CONF_MSG;

		// Send
		if ((call OpenSendToLower.send(pkt))==FAIL) {
			call Malloc.freePacketBuffer(pkt);
		}
	}

	task void taskBuildAndSendSeedMsgToChildren() {
		uint8_t i;
		OpenQueueEntry_t* pkt;

		for (i=0;i<neighborList.numChildren;i++) {

			pkt = call Malloc.getFreePacketBuffer();
			if (pkt==NULL) {
				return;
			}

			pkt->creator = COMPONENT_RES;
			pkt->owner = COMPONENT_RES;

			// Request Message header
			call PacketFunctions.reserveHeaderSize(pkt,sizeof(CMD_MSG_t));
			((CMD_MSG_t*)(pkt->payload))->commandFrameId = SEED_MSG;
			((CMD_MSG_t*)(pkt->payload))->seedNumber = TOS_NODE_ID;
			((CMD_MSG_t*)(pkt->payload))->numChildren = neighborList.numChildren;
			memcpy(((CMD_MSG_t*)(pkt->payload))->seedList,neighborList.seedList,((sizeof(uint8_t))*(neighborList.numChildren)));

			// l2 header
			memcpy(&(pkt->l2_nextORpreviousHop),&(neighborList.neighbors[neighborList.childrenIndex[i]].addr_16b),sizeof(open_addr_t));
			pkt->l2_seedNumber = neighborList.neighbors[neighborList.childrenIndex[i]].seedNumber;

			pkt->l2_frameType = IEEE154_TYPE_CMD;
			pkt->l2_commandFrameId = SEED_MSG;

			// Send
			if ((call OpenSendToLower.send(pkt))==FAIL) {
				call Malloc.freePacketBuffer(pkt);
			}

		}
	}

	task void taskBuildAndSendDataToChildren() {
		uint8_t i;
		OpenQueueEntry_t* pkt;

		for (i=0;i<neighborList.numChildren;i++) {

			pkt = call Malloc.getFreePacketBuffer();
			if (pkt==NULL) {
				return;
			}

			pkt->creator = COMPONENT_RES;
			pkt->owner = COMPONENT_RES;

			// Request Message header
			call PacketFunctions.reserveHeaderSize(pkt,sizeof(CMD_MSG_t));

			// l2 header
			memcpy(&(pkt->l2_nextORpreviousHop),&(neighborList.neighbors[neighborList.childrenIndex[i]].addr_16b),sizeof(open_addr_t));
			pkt->l2_seedNumber = neighborList.neighbors[neighborList.childrenIndex[i]].seedNumber;
			pkt->l2_frameType = IEEE154_TYPE_DATA;

			// Send
			if ((call OpenSendToLower.send(pkt))==FAIL) {
				call Malloc.freePacketBuffer(pkt);
			}

		}
		if (neighborList.parent!=254) {
			pkt = call Malloc.getFreePacketBuffer();
			if (pkt==NULL) {
				return;
			}

			pkt->creator = COMPONENT_RES;
			pkt->owner = COMPONENT_RES;

			// Request Message header
			call PacketFunctions.reserveHeaderSize(pkt,sizeof(CMD_MSG_t));

			// l2 header
			memcpy(&(pkt->l2_nextORpreviousHop),&(neighborList.neighbors[neighborList.parent].addr_16b),sizeof(open_addr_t));
			pkt->l2_seedNumber = neighborList.neighbors[neighborList.parent].seedNumber;
			pkt->l2_frameType = IEEE154_TYPE_DATA;

			// Send
			if ((call OpenSendToLower.send(pkt))==FAIL) {
				call Malloc.freePacketBuffer(pkt);
			}
		}

	}

	void updateNeighborList(OpenQueueEntry_t *msg) {
		uint8_t i;

		if (((CMD_MSG_t*)(msg->payload))->commandFrameId == SEED_MSG) {
			neighborList.numSibling = ((CMD_MSG_t*)(msg->payload))->numChildren;
			memcpy(neighborList.siblingList,((CMD_MSG_t*)(msg->payload))->seedList,((sizeof(uint8_t))*(neighborList.numSibling)));
		} else {
			for (i=0;i<MAXNUMNEIGHBORS;i++)
			{
				atomic {
					if (isThisRowMatching(&(msg->l2_nextORpreviousHop),i)) {

						neighborList.neighbors[i].rssi = msg->l1_rssi;
						if (((CMD_MSG_t*)(msg->payload))->commandFrameId == ADV_MSG) {
							neighborList.neighbors[i].numHops = ((CMD_MSG_t*)(msg->payload))->numHops;
							if (neighborList.neighbors[i].numHops==neighborList.minHops) {
								if (neighborList.neighbors[i].rssi>neighborList.maxRssi) {
									neighborList.maxRssi=neighborList.neighbors[i].rssi;
									neighborList.parent=i;
								}
							} else if (neighborList.neighbors[i].numHops<neighborList.minHops) {
								neighborList.maxRssi=neighborList.neighbors[i].rssi;
								neighborList.minHops=neighborList.neighbors[i].numHops;
								neighborList.parent=i;
							}
						} else if (((CMD_MSG_t*)(msg->payload))->commandFrameId == REQ_MSG) {
							if (neighborList.neighbors[i].isChild == FALSE) {
								neighborList.neighbors[i].isChild = TRUE;
								neighborList.seedList[neighborList.numChildren] = ((CMD_MSG_t*)(msg->payload))->seedNumber;
								neighborList.childrenIndex[neighborList.numChildren] = i;
								neighborList.numChildren=neighborList.numChildren+1;
							}
						}
						return;
					}
				}
			}
			registerNewNeighbor(msg);
		}
	}

	bool isThisRowMatching(open_addr_t* address, uint8_t rowNumber) {
		switch (address->type) {
			case ADDR_16B:
			atomic return neighborList.neighbors[rowNumber].used &&
			call PacketFunctions.sameAddress(address,&neighborList.neighbors[rowNumber].addr_16b);
			case ADDR_64B:
			atomic return neighborList.neighbors[rowNumber].used &&
			call PacketFunctions.sameAddress(address,&neighborList.neighbors[rowNumber].addr_64b);
			case ADDR_128B:
			atomic return neighborList.neighbors[rowNumber].used &&
			call PacketFunctions.sameAddress(address,&neighborList.neighbors[rowNumber].addr_128b);
			break;
			default:
			return FALSE;
			break;
		}
	}

	void registerNewNeighbor(OpenQueueEntry_t *msg) {
		open_addr_t temp_prefix;
		open_addr_t temp_addr16b;
		open_addr_t temp_addr64b;
		open_addr_t temp_addr128b;

		uint8_t i=0;
		while(i<MAXNUMNEIGHBORS) {
			atomic {
				if (neighborList.neighbors[i].used==FALSE) {
					neighborList.neighbors[i].used = TRUE;
					neighborList.numNeighbors=neighborList.numNeighbors+1;
					neighborList.neighbors[i].seedNumber = ((CMD_MSG_t*)(msg->payload))->seedNumber;
					neighborList.neighbors[i].addr_16b.type = ADDR_NONE;
					neighborList.neighbors[i].addr_64b.type = ADDR_NONE;
					neighborList.neighbors[i].addr_128b.type = ADDR_NONE;
					switch (msg->l2_nextORpreviousHop.type) {
						case ADDR_16B:
						call PacketFunctions.mac16bToMac64b(&(msg->l2_nextORpreviousHop),&temp_addr64b);
						call PacketFunctions.mac64bToIp128b(
								call IDManager.getMyID(ADDR_PREFIX),
								&temp_addr64b,
								&temp_addr128b);
						memcpy(&neighborList.neighbors[i].addr_16b, &(msg->l2_nextORpreviousHop), sizeof(open_addr_t));
						memcpy(&neighborList.neighbors[i].addr_64b, &temp_addr64b, sizeof(open_addr_t));
						memcpy(&neighborList.neighbors[i].addr_128b, &temp_addr128b, sizeof(open_addr_t));
						break;
						case ADDR_64B:
						call PacketFunctions.mac64bToMac16b(&(msg->l2_nextORpreviousHop),&temp_addr16b);
						call PacketFunctions.mac64bToIp128b(
								call IDManager.getMyID(ADDR_PREFIX),
								&(msg->l2_nextORpreviousHop),
								&temp_addr128b);
						memcpy(&neighborList.neighbors[i].addr_16b, &temp_addr16b, sizeof(open_addr_t));
						memcpy(&neighborList.neighbors[i].addr_64b, &(msg->l2_nextORpreviousHop), sizeof(open_addr_t));
						memcpy(&neighborList.neighbors[i].addr_128b, &temp_addr128b, sizeof(open_addr_t));
						break;
						case ADDR_128B:
						call PacketFunctions.ip128bToMac64b(
								&(msg->l2_nextORpreviousHop),
								&temp_prefix,
								&temp_addr64b);
						call PacketFunctions.mac64bToMac16b(&temp_addr64b,&temp_addr16b);
						memcpy(&neighborList.neighbors[i].addr_16b, &temp_addr16b, sizeof(open_addr_t));
						memcpy(&neighborList.neighbors[i].addr_64b, &temp_addr64b, sizeof(open_addr_t));
						memcpy(&neighborList.neighbors[i].addr_128b, &(msg->l2_nextORpreviousHop), sizeof(open_addr_t));
						break;
					}
					neighborList.neighbors[i].rssi = msg->l1_rssi;

					if (((CMD_MSG_t*)(msg->payload))->commandFrameId == ADV_MSG) {
						neighborList.neighbors[i].numHops = ((CMD_MSG_t*)(msg->payload))->numHops;
						if (neighborList.neighbors[i].numHops==neighborList.minHops) {
							if (neighborList.neighbors[i].rssi>neighborList.maxRssi) {
								neighborList.maxRssi=neighborList.neighbors[i].rssi;
								neighborList.parent=i;
							}
						} else if (neighborList.neighbors[i].numHops<neighborList.minHops) {
							neighborList.maxRssi=neighborList.neighbors[i].rssi;
							neighborList.minHops=neighborList.neighbors[i].numHops;
							neighborList.parent=i;
						}
					} else if (((CMD_MSG_t*)(msg->payload))->commandFrameId == REQ_MSG) {
						neighborList.neighbors[i].isChild = TRUE;
						neighborList.seedList[neighborList.numChildren] = ((CMD_MSG_t*)(msg->payload))->seedNumber;
						neighborList.childrenIndex[neighborList.numChildren] = i;
						neighborList.numChildren=neighborList.numChildren+1;
					}
					return;
				}
			}
			i++;
		}
	}

	void resetNeighborList() {
		uint8_t i;
		neighborList.numNeighbors=0;
		neighborList.old_numNeighbors=0;
		neighborList.counterNoNewNeighbors=0;
		neighborList.numChildren=0;
		neighborList.numSibling=0;
		neighborList.old_numChildren=0;
		neighborList.counterNoNewChildren=0;
		neighborList.counterSeedMsgSent=0;
		neighborList.maxRssi=-64;
		neighborList.minHops=255;
		neighborList.parent=254;

		for (i=0;i<MAXNUMNEIGHBORS;i++) {
			neighborList.neighbors[i].used=FALSE;
			neighborList.neighbors[i].isChild=FALSE;
			neighborList.neighbors[i].seedNumber=254;
			neighborList.neighbors[i].rssi=-64;
			neighborList.neighbors[i].numHops=255;
		}
	}

	// Added for debugging
	//	void printfNeighbors(){
	//		uint8_t i;
	//		printf("maxRssi: %i\n",(int32_t)(neighborList.maxRssi));
	//		for(i=0;i<neighborList.numNeighbors;i++){
	//			printf("Neighbor %u %i\n",i,(int32_t)(neighborList.neighbors[i].rssi));
	//		}
	//
	//	}

	/*-------------------------------- interfaces ----------------------------------------*/

	//SoftwareInit

	command error_t SoftwareInit.init() {
		call UserButton.enable();
		resetNeighborList();
		state = ScanWait;
		return SUCCESS;
	}

	//OpenSendFromUpper

	command error_t OpenSendFromUpper.send(OpenQueueEntry_t *msg) {
		msg->owner = COMPONENT_RES;
		return sendToLower(msg);
	}

	error_t sendToLower(OpenQueueEntry_t *msg) {
		msg->l2_frameType = IEEE154_TYPE_DATA;
		return call OpenSendToLower.send(msg);
	}

	// OpenSendToLower

	event void OpenSendToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {
		msg->owner = COMPONENT_RES;

		switch (msg->creator) {
			case COMPONENT_RES:

			if (msg->l2_commandFrameId == SCAN_MSG) {

				if (neighborList.numNeighbors==neighborList.old_numNeighbors) {
					atomic neighborList.counterNoNewNeighbors=neighborList.counterNoNewNeighbors+1;
					if (neighborList.counterNoNewNeighbors>=MAXSCANSAMPLE) {
						if ((call IDManager.getIsDAGroot())==TRUE) {
							state = AdvSend;
							//							printfNeighbors();
							post taskBuildAndSendAdvMsg();
						} else {
							state = AdvWait;
						}
					} else {
						post taskBuildAndSendScanMsg();
					}
				} else {
					atomic neighborList.old_numNeighbors=neighborList.numNeighbors;
					post taskBuildAndSendScanMsg();
				}
			} else if (msg->l2_commandFrameId == ADV_MSG) {
				if (neighborList.numChildren==neighborList.old_numChildren) {
					atomic neighborList.counterNoNewChildren=neighborList.counterNoNewChildren+1;
					if (neighborList.counterNoNewChildren>=MAXADVSAMPLE) {
						if (neighborList.numChildren>0) {
							state = Build;
							post taskBuildAndSendSeedMsgToChildren();
						} else {
							state = BuildDone;
							call MAC.changeNetworkState (SCHEDULING);
							post taskBuildAndSendDataToChildren(); // Added for debugging
						}

					} else {
						post taskBuildAndSendAdvMsg();
					}
				} else {
					atomic neighborList.old_numChildren=neighborList.numChildren;
					post taskBuildAndSendAdvMsg();
				}
			} else if (msg->l2_commandFrameId == SEED_MSG) {
				neighborList.counterSeedMsgSent=neighborList.counterSeedMsgSent+1;
				if (neighborList.counterSeedMsgSent>=neighborList.numChildren) {
					state = BuildDone;
					call MAC.changeNetworkState (SCHEDULING);
					post taskBuildAndSendDataToChildren(); // Added for debugging
				}
			} else if (msg->l2_frameType == IEEE154_TYPE_DATA) {
				//				post taskBuildAndSendDataToChildren(); // Added for debugging
			}
			call Malloc.freePacketBuffer(msg);
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

		switch (msg->l2_frameType) {
			case IEEE154_TYPE_CMD:
			if (((CMD_MSG_t*)(msg->payload))->commandFrameId == SCAN_MSG) {
				atomic {
					if (state == ScanWait || state == ScanSend) {
						updateNeighborList(msg);
						if (state == ScanWait) {
							state = ScanSend;
							post taskBuildAndSendScanMsg();
						}
					}
				}
			} else if (((CMD_MSG_t*)(msg->payload))->commandFrameId == ADV_MSG) {
				atomic {
					if (state == AdvWait) {
						if ((call timerAdvWait.isRunning() == FALSE) && (call IDManager.getIsDAGroot()==FALSE) ) {
							call timerAdvWait.startOneShot(5000);
						}
						updateNeighborList(msg);
					}
				}
			} else if (((CMD_MSG_t*)(msg->payload))->commandFrameId == REQ_MSG) {
				atomic {
					if (state == AdvSend) {
						updateNeighborList(msg);
						taskBuildAndSendConfMsg(msg);
					}
				}
			} else if (((CMD_MSG_t*)(msg->payload))->commandFrameId == CONF_MSG) {
				atomic {
					if (state == ReqSend) {
						state = AdvSend;
						post taskBuildAndSendAdvMsg();
					}
				}
			} else if (((CMD_MSG_t*)(msg->payload))->commandFrameId == SEED_MSG) {
				atomic {
					if (state == AdvSend || state == Build) {
						updateNeighborList(msg);
					}
				}
			}
			call Malloc.freePacketBuffer(msg);
			break;

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

			default:
			call Malloc.freePacketBuffer(msg);
			//            call OpenSerial.printError(COMPONENT_RES,ERR_MSG_UNKNOWN_TYPE,msg->l2_frameType,0);
			break;
		}
	}

	async command bool LatinMatrix.childCanTransmit (uint8_t minLatinWindow, uint8_t* latinChannelOffset) {
		uint8_t i, j;
		for (i=0; i<NUMCHANNELS; i=i+1) {
			for (j=0; j<neighborList.numChildren; j=j+1) {
				if ((minLatinWindow+i)%MAXNUMNEIGHBORS==neighborList.seedList[j]) {
					*latinChannelOffset=i;
					return TRUE;
				}
			}
		}
		return FALSE;
	}

	async command bool LatinMatrix.siblingCanTransmit (uint8_t minLatinWindow, uint8_t* latinChannelOffset) {
		uint8_t i, j;
		for (i=0; i<NUMCHANNELS; i=i+1) {
			for (j=0; j<neighborList.numSibling; j=j+1) {
				if ((minLatinWindow+i)%MAXNUMNEIGHBORS==neighborList.siblingList[j]) {
					if (neighborList.siblingList[j]==TOS_NODE_ID) {
						*latinChannelOffset=i;
						return FALSE;
					} else {
						*latinChannelOffset=i;
						return TRUE;
					}
				}
			}
		}
		return TRUE;
	}

	async command bool LatinMatrix.parentCanTransmit (uint8_t minLatinWindow, uint8_t* latinChannelOffset) {
		uint8_t i;
		for (i=0; i<NUMCHANNELS; i=i+1) {
			if ((minLatinWindow+i)%MAXNUMNEIGHBORS==neighborList.neighbors[neighborList.parent].seedNumber) {
				*latinChannelOffset=i;
				return TRUE;
			}
		}
		return FALSE;
	}

	async command bool LatinMatrix.canTransmit (uint8_t minLatinWindow, uint8_t* latinChannelOffset) {
		uint8_t i;
		for (i=0; i<NUMCHANNELS; i=i+1) {
			if ((minLatinWindow+i)%MAXNUMNEIGHBORS==TOS_NODE_ID) {
				*latinChannelOffset=i;
				return TRUE;
			}
		}
		return FALSE;
	}

	async command uint8_t LatinMatrix.getParentSeed() {
		return neighborList.neighbors[neighborList.parent].seedNumber;
	}

	// Timers

	event void timerAdvWait.fired() {
		atomic state = ReqSend;
		post taskBuildAndSendReqMsg();
	}

	// Misc

	event void UserButton.notify( button_state_t button ) {
		if ((call IDManager.getIsDAGroot())==TRUE && state==ScanWait) {
			atomic state = ScanSend;
			post taskBuildAndSendScanMsg();
		}
	}
}
