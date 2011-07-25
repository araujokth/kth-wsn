/*
 * Copyright (c) 2011, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice, this list
 * 	  of conditions and the following disclaimer.
 *
 * 	- Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *	  materials provided with the distribution.
 *
 * 	- Neither the name of the KTH Royal Institute of Technology nor the names of its
 *    contributors may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
/**
 * @author Aziz Khakulov <khakulov@kth.se> 
 * 
 * @version  $Revision: 1.0 Date: 2011/07/21 $ 
 * @modified 2011/06/07 
 */
/*
* This is an example code showing how the serial communication
* between the mote and the PC works. It has a timer which fires every second,
* increments a counter by 1 and sends the counter value to the serial port.
*/

#include <Timer.h>
#include "app_profile.h"


module TestSerialCommC {
	uses {
		interface Boot;
		interface Leds;
		interface SplitControl as SerialControl;
		interface AMSend as UartSend[am_id_t id];		
		interface Packet;
		interface AMPacket;
		interface Timer<TMilli> as MilliTimer; 
	}

}
implementation {

	uint16_t counter;
	message_t pkt;
	bool busy = FALSE;	
	uint16_t counter = 0; 
	
	event void Boot.booted() {		
		call AMPacket.setSource(&pkt, TOD_NODE_ID);
		call SerialControl.start();
	}

	event void SerialControl.startDone(error_t err) {
		if (err == SUCCESS) {
		MilliTimer.startPeriodic(1000);
		}
		else {
			call SerialControl.start();
		}
	}
	event void MilliTimer.fired() {
		if(busy == FALSE){
			counter++;
			TestSerialCommMsg* rcm = (TestSerialCommMsg*)call Packet.getPayload(&packet, sizeof(TestSerialCommMsg));
			rcm->counter = counter;
			if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(test_serial_msg_t)) == SUCCESS) {	
				call Leds.led0Toggle();
			}
			busy = TRUE;
		}
	}
	
	event void SerialControl.stopDone(error_t err) {}

	event void UartSend.sendDone[am_id_t id](message_t* msg, error_t err) {
		if (&pkt == msg) {
			busy = FALSE;
		}
	}
}
