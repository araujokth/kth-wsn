#include "LATIN.h"
#include "printf.h"

module appDATADevP {

	provides interface Init as SoftwareInit;
	provides interface OpenReceive as OpenReceiveFromLower;

	uses interface OpenSend as OpenSendToLower;

	uses interface Malloc;

	uses interface Leds;


	uses interface PacketFunctions;

	uses interface IDManager;

}

implementation {
	/*-------------------------------- variables -----------------------------------------*/

	/*-------------------------------- prototypes ----------------------------------------*/
	void taskBuildAndSendPkt();
	/*-------------------------------- http receive sequence -----------------------------*/

	/*-------------------------------- interfaces ----------------------------------------*/

	//SoftwareInit
	command error_t SoftwareInit.init() {

		return SUCCESS;
	}

	event void OpenSendToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {

	}

	command void OpenReceiveFromLower.receive(OpenQueueEntry_t* msg) {

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
}
