#include "LATIN.h"
#include "CC2420.h"

module BlinkHopDevP {

	uses interface Boot;
	provides interface OpenReceive as OpenReceiveFromLower;
	uses interface Malloc;
	uses interface IDManager;
	uses interface Leds;



}

implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- prototypes ----------------------------------------*/

   /*-------------------------------- http receive sequence -----------------------------*/

   /*-------------------------------- interfaces ----------------------------------------*/

	event void Boot.booted() {
		call IDManager.setIsDAGroot(FALSE);
	}
	command void OpenReceiveFromLower.receive(OpenQueueEntry_t* pkt) {
		  call Leds.led0Toggle();
		  call Malloc.freePacketBuffer(pkt);
	 }
}

