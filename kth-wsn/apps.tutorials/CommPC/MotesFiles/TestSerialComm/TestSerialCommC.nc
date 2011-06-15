// $Id: TestSerialCommC.nc,v 1.6 2010-06-29 22:07:40 scipio Exp $

/*
 * Copyright (c) 2000-2006 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * Implementation of the TestSerialComm application.  A counter is
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
#include "app_profile.h"


module TestSerialCommC {
	uses {
		interface Boot;

		interface Leds;

		interface SplitControl as SerialControl;
		interface AMSend as UartSend[am_id_t id];
		interface Receive as UartReceive[am_id_t id];
		interface Packet;
		interface AMPacket;


	}

}
implementation {

	uint16_t counter;
	message_t pkt;
	bool busy = FALSE;

	void setLeds(uint16_t val) {
		if (val & 0x01)
		call Leds.led0On();
		else
		call Leds.led0Off();
		if (val & 0x02)
		call Leds.led1On();
		else
		call Leds.led1Off();
		if (val & 0x04)
		call Leds.led2On();
		else
		call Leds.led2Off();
	}

	event void Boot.booted() {
		call AMPacket.setSource(&pkt, TOS_NODE_ID);

		call SerialControl.start();
	}

	event void SerialControl.startDone(error_t err) {
		if (err == SUCCESS) {
		}
		else {
			call SerialControl.start();
		}
	}

	event void SerialControl.stopDone(error_t err) {}

	event void UartSend.sendDone[am_id_t id](message_t* msg, error_t err) {
		if (&pkt == msg) {
			busy = FALSE;
		}
	}

	event message_t* UartReceive.receive[am_id_t id](message_t* msg, void* payload, uint8_t len) {
		if (len == sizeof(TestSerialCommMsg)) {
			TestSerialCommMsg* btrpkt = (TestSerialCommMsg*)payload;
			setLeds(btrpkt->counter);
			counter = btrpkt->counter + 1;
			
			if (!busy) {
				btrpkt = (TestSerialCommMsg*)(call Packet.getPayload(&pkt, sizeof(TestSerialCommMsg)));

				if (btrpkt == NULL) return;

				btrpkt->counter = counter;

				if (call UartSend.send[AM_TESTSERIALCOMMMSG](AM_BROADCAST_ADDR,
								&pkt, sizeof(TestSerialCommMsg)) == SUCCESS) {
					busy = TRUE;
				}
			}

		}
		return msg;
	}

}
