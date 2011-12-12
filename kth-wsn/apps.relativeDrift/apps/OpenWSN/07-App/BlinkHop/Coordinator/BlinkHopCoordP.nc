#include "OpenWSN.h"
#include "CC2420.h"

module BlinkHopCoordP {

	uses interface Boot;
	uses interface OpenSend as OpenSendToLower;
	uses interface Malloc;
	uses interface IDManager;
	uses interface PacketFunctions;

	uses interface Leds;
	uses interface Alarm<T32khz,uint32_t> as AlarmBlinkP;


}

implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- prototypes ----------------------------------------*/
	void startApp();
	task void taskSend();
   /*-------------------------------- http receive sequence -----------------------------*/

   /*-------------------------------- interfaces ----------------------------------------*/

	event void Boot.booted() {
		call IDManager.setIsDAGroot(TRUE);
		startApp();
	}

	void startApp()
	{
		atomic call AlarmBlinkP.startAt(call AlarmBlinkP.getNow(),32000);
	}
	async event void AlarmBlinkP.fired() {
			post taskSend();

	}
	task void taskSend() {
					OpenQueueEntry_t* pkt;
					pkt = call Malloc.getFreePacketBuffer();
					pkt->creator                      = COMPONENT_BLINKHOP;
					pkt->owner                        = COMPONENT_BLINKHOP;
					call PacketFunctions.reserveHeaderSize(pkt,2);
					((uint8_t*)pkt->payload)[0] = 'h';
					((uint8_t*)pkt->payload)[1] = 'o';
					((uint8_t*)pkt->payload)[2] = 'p';
					((uint8_t*)pkt->payload)[3] = 'p';
					((uint8_t*)pkt->payload)[4] = 'i';
					((uint8_t*)pkt->payload)[5] = 'n';
					((uint8_t*)pkt->payload)[5] = 'g';
					if ((call OpenSendToLower.send(pkt))==FAIL) {
					         call Malloc.freePacketBuffer(pkt);
					}
					call AlarmBlinkP.start(32000);
	 }
	 event void OpenSendToLower.sendDone(OpenQueueEntry_t* pkt, error_t error) {
	      call Malloc.freePacketBuffer(pkt);
	}
}
