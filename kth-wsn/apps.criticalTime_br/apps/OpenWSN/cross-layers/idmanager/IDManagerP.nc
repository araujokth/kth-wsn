#include "OpenWSN.h"
#include "IDManager.h"

module IDManagerP {
	provides interface Init as SoftwareInit;
	provides interface DebugPrint;
	provides interface IDManager;
	//   uses     interface OpenSerial;
	uses interface PacketFunctions;
	provides interface Trigger as TriggerIDManagerAboutBridge;
	provides interface Trigger as TriggerIDManagerAboutRoot;
}
implementation {
	/*-------------------------------- variables -----------------------------------------*/

	bool isDAGroot;
	bool isBridge;
	open_addr_t my16bID;
	open_addr_t my64bID;
	open_addr_t myPANID;
	open_addr_t myPrefix;

	/*-------------------------------- prototypes ----------------------------------------*/

	task void taskPrint();
	task void taskHandleBridge();
	task void taskHandleRoot();

	/*-------------------------------- interfaces ----------------------------------------*/

	command void TriggerIDManagerAboutBridge.trigger() {
		post taskHandleBridge();
	}
	task void taskHandleBridge() {
		uint8_t number_bytes_from_input_buffer;
		uint8_t input_buffer[9];
		//get command from OpenSerial (1B command, 8B prefix)
		//      number_bytes_from_input_buffer = call OpenSerial.getInputBuffer(&input_buffer[0],sizeof(input_buffer));
		if (number_bytes_from_input_buffer!=sizeof(input_buffer)) {
			//         call OpenSerial.printError(COMPONENT_IDMANAGER,ERR_INPUTBUFFER_LENGTH,
			//               (errorparameter_t)number_bytes_from_input_buffer,
			//               (errorparameter_t)0);
			return;
		};
		//handle command
		switch (input_buffer[0]) {
			case 'Y':
			call IDManager.setIsBridge(TRUE);
			memcpy(&(myPrefix.prefix),&(input_buffer[1]),8);
			break;
			case 'N':
			call IDManager.setIsBridge(FALSE);
			break;
			case 'T':
			if (call IDManager.getIsBridge()) {
				call IDManager.setIsBridge(FALSE);
			} else {
				call IDManager.setIsBridge(TRUE);
				memcpy(&(myPrefix.prefix),&(input_buffer[1]),8);
			}
			break;
		}
		return;
	}

	command void TriggerIDManagerAboutRoot.trigger() {
		post taskHandleRoot();
	}
	task void taskHandleRoot() {
		uint8_t number_bytes_from_input_buffer;
		uint8_t input_buffer;
		//get command from OpenSerial (16B IPv6 destination address, 2B destination port)
		//      number_bytes_from_input_buffer = call OpenSerial.getInputBuffer(&input_buffer,sizeof(input_buffer));
		if (number_bytes_from_input_buffer!=sizeof(input_buffer)) {
			//         call OpenSerial.printError(COMPONENT_IDMANAGER,ERR_INPUTBUFFER_LENGTH,
			//               (errorparameter_t)number_bytes_from_input_buffer,
			//               (errorparameter_t)0);
			return;
		};
		//handle command
		switch (input_buffer) {
			case 'Y':
			call IDManager.setIsDAGroot(TRUE);
			break;
			case 'N':
			call IDManager.setIsDAGroot(FALSE);
			break;
			case 'T':
			if (call IDManager.getIsDAGroot()) {
				call IDManager.setIsDAGroot(FALSE);
			} else {
				call IDManager.setIsDAGroot(TRUE);
			}
			break;
		}
		return;
	}

	command error_t SoftwareInit.init() {
		isDAGroot = FALSE;
		isBridge = FALSE;
		myPANID.type = ADDR_PANID;
		myPANID.panid[0] = 0xAA;
		myPANID.panid[1] = 0xAA;
		myPrefix.type = ADDR_PREFIX;
		/*myPrefix.prefix[0]   = 0x20;
		 myPrefix.prefix[1]   = 0x01;
		 myPrefix.prefix[2]   = 0x04;
		 myPrefix.prefix[3]   = 0x70;
		 myPrefix.prefix[4]   = 0x1f;
		 myPrefix.prefix[5]   = 0x05;
		 myPrefix.prefix[6]   = 0x09;
		 myPrefix.prefix[7]   = 0x8e;*/
		myPrefix.prefix[0] = 0x00;
		myPrefix.prefix[1] = 0x00;
		myPrefix.prefix[2] = 0x00;
		myPrefix.prefix[3] = 0x00;
		myPrefix.prefix[4] = 0x00;
		myPrefix.prefix[5] = 0x00;
		myPrefix.prefix[6] = 0x00;
		myPrefix.prefix[7] = 0x00;
		my64bID.type = ADDR_64B;
		my16bID.addr_64b[0] = 0x00;
		my64bID.addr_64b[1] = 0x00;
		my64bID.addr_64b[2] = 0x00;
		my64bID.addr_64b[3] = 0x00;
		my64bID.addr_64b[4] = 0x00;
		my64bID.addr_64b[5] = 0x00;
		my64bID.addr_64b[6] = 0x00;
		my64bID.addr_64b[7] = TOS_NODE_ID;
		//uncomment to have a static DAGroot node
		/*if (TOS_NODE_ID==18) {
		 call IDManager.setIsDAGroot(TRUE);
		 }*/
		call PacketFunctions.mac64bToMac16b(&my64bID,&my16bID);
		return SUCCESS;
	}

	async command bool IDManager.getIsDAGroot() {
		atomic return isDAGroot;
	}
	async command error_t IDManager.setIsDAGroot(bool newRole) {
		atomic isDAGroot = newRole;
		return SUCCESS;
	}

	async command bool IDManager.getIsBridge() {
		atomic return isBridge;
	}
	async command error_t IDManager.setIsBridge(bool newRole) {
		atomic isBridge = newRole;
		return SUCCESS;
	}

	async command open_addr_t* IDManager.getMyID(uint8_t type) {
		switch (type) {
			case ADDR_16B:
			return &my16bID;
			case ADDR_64B:
			return &my64bID;
			case ADDR_PANID:
			return &myPANID;
			case ADDR_PREFIX:
			return &myPrefix;
			case ADDR_128B:
			//you don't ask for my full address, rather for prefix, then 64b
			default:
			//            call OpenSerial.printError(COMPONENT_IDMANAGER,ERR_WRONG_ADDR_TYPE,
			//                  (errorparameter_t)type,
			//                  (errorparameter_t)0);
			return NULL;
		}
	}
	async command error_t IDManager.setMyID(open_addr_t* newID) {
		switch (newID->type) {
			case ADDR_16B:
			atomic memcpy(&my16bID,newID,sizeof(open_addr_t));
			break;
			case ADDR_64B:
			atomic memcpy(&my64bID,newID,sizeof(open_addr_t));
			break;
			case ADDR_PANID:
			atomic memcpy(&myPANID,newID,sizeof(open_addr_t));
			break;
			case ADDR_PREFIX:
			atomic memcpy(&myPrefix,newID,sizeof(open_addr_t));
			break;
			case ADDR_128B:
			//don't set 128b, but rather prefix and 64b
			default:
			//            call OpenSerial.printError(COMPONENT_IDMANAGER,ERR_WRONG_ADDR_TYPE,
			//                  (errorparameter_t)newID->type,
			//                  (errorparameter_t)1);
			return FAIL;
		}
		return SUCCESS;
	}

	async command bool IDManager.isMyAddress(open_addr_t* addr) {
		open_addr_t temp_my128bID;
		switch (addr->type) {
			case ADDR_16B:
			return call PacketFunctions.sameAddress(addr,&my16bID);
			case ADDR_64B:
			return call PacketFunctions.sameAddress(addr,&my64bID);
			case ADDR_128B:
			//build temporary my128bID
			temp_my128bID.type = ADDR_128B;
			memcpy(&temp_my128bID.addr_128b[0],&myPrefix.prefix,8);
			memcpy(&temp_my128bID.addr_128b[8],&my64bID.addr_64b,8);
			return call PacketFunctions.sameAddress(addr,&temp_my128bID);
			case ADDR_PANID:
			return call PacketFunctions.sameAddress(addr,&myPANID);
			case ADDR_PREFIX:
			return call PacketFunctions.sameAddress(addr,&myPrefix);
			default:
			//            call OpenSerial.printError(COMPONENT_IDMANAGER,ERR_WRONG_ADDR_TYPE,
			//                  (errorparameter_t)addr->type,
			//                  (errorparameter_t)2);
			return FALSE;
		}
	}

	command void DebugPrint.print() {
		post taskPrint();
	}
	command void IDManager.setDAGRoot(bool setIsDAGRoot)
	{
		atomic isDAGroot = setIsDAGRoot;
	}
	/*-------------------------------- helper functions ----------------------------------*/

	task void taskPrint() {
		debugIDManagerEntry_t output;
		atomic {
			output.isDAGroot = isDAGroot;
			output.isBridge = isBridge;
			output.my16bID = my16bID;
			output.my64bID = my64bID;
			output.myPANID = myPANID;
			output.myPrefix = myPrefix;
		}
		//      call OpenSerial.printStatus(STATUS_IDMANAGER_ID,(uint8_t*)&output,sizeof(debugIDManagerEntry_t));
	}
}
