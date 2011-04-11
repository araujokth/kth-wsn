/*
 * Copyright (c) 2010, KTH Royal Institute of Technology
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
 * @author Aitor Hernandez <aitorhh@kth.se>
 * 
 * @version  $Revision: 1.0 Date: 2010/11/03 $ 
 * @modified 2011/02/01 
 */
#include <Timer.h>
#include "InterferenceRate.h"
#include "UserButton.h"

module InterferenceRateC {
	uses interface Boot;
	uses interface Leds;
	uses interface Timer<TMilli> as SendTimer;
	uses interface Packet;
	uses interface AMPacket;
	uses interface AMSend;
	uses interface Receive;
	uses interface SplitControl as AMControl;

	//To change the rate
	uses interface Notify<button_state_t>;
	uses interface Get<button_state_t>;
}
implementation {

	message_t pkt;
	bool busy = FALSE;
	uint8_t payloadLen = sizeof(InterferenceRateMsg)+20;
	uint16_t rate = DEFAULT_RATE;

	event void Boot.booted() {
		call AMControl.start();
		call Notify.enable();
	}

	event void AMControl.startDone(error_t err) {
		if (err == SUCCESS) {
			call SendTimer.startPeriodic(rate);
		}
		else {
			call AMControl.start();
		}
	}

	event void AMControl.stopDone(error_t err) {}

	event void SendTimer.fired() {
		if (call AMSend.send(AM_BROADCAST_ADDR,
						&pkt, payloadLen) == SUCCESS) {
			call Leds.led1Toggle();
			busy = TRUE;
		} else
		call Leds.led0Toggle();

	}

	event void AMSend.sendDone(message_t* msg, error_t err) {
		if (&pkt == msg) {
			busy = FALSE;
		}
	}

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
		return msg;
	}

	/*
	 * To change the sample rate
	 */
	event void Notify.notify( button_state_t state ) {
		if ( state == BUTTON_PRESSED ) {
			atomic {rate /= 5;}
			call Leds.led2On();
		} else if ( state == BUTTON_RELEASED ) {
			call Leds.led2Off();
		}
		call SendTimer.stop();

		call SendTimer.startPeriodic(rate);

	}
}