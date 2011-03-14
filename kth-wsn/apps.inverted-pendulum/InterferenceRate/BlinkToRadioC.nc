// $Id: BlinkToRadioC.nc,v 1.5 2007/09/13 23:10:23 scipio Exp $

/*
 * "Copyright (c) 2000-2006 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 */

/**
 * Implementation of the BlinkToRadio application.  A counter is
 * incremented and a radio message is sent whenever a timer fires.
 * Whenever a radio message is received, the three least significant
 * bits of the counter in the message payload are displayed on the
 * LEDs.  Program two motes with this application.  As long as they
 * are both within range of each other, the LEDs on both will keep
 * changing.  If the LEDs on one (or both) of the nodes stops changing
 * and hold steady, then that node is no longer receiving any messages
 * from the other node.
 *
 * @author Prabal Dutta
 * @date   Feb 1, 2006
 */
#include <Timer.h>
#include "BlinkToRadio.h"
#include "UserButton.h"

module BlinkToRadioC {
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
	uint8_t payloadLen = sizeof(BlinkToRadioMsg)+20;
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