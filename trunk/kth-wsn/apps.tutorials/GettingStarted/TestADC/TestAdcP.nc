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
 * @author Aziz Khakulov <khakulov@kth.se> * 
 * 
 * @version  $Revision: 1.0 Date: 2011/07/257 $ 
 * @modified 2011/07/25 
 */
#include <Timer.h>
#include "app_profile.h"
#include "Serial.h"

module TestAdcP {
	uses {
		interface Boot;
		interface Leds;		
		interface Timer<TMilli> as MilliTimer;
		interface Resource;
		interface Msp430Adc12MultiChannel as MultiChannel;
		interface SplitControl as SerialControl;
		interface AMSend as UartSend[am_id_t id];
		interface AMPacket;
		interface Packet;
	}
	provides interface AdcConfigure<const msp430adc12_channel_config_t*>;
}
implementation {
	message_t pkt;
	bool busy = FALSE;
	uint16_t buffer[4];
	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_1_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};
	
	event void Boot.booted() {
		call SerialControl.start();
		call AMPacket.setSource(&pkt, TOS_NODE_ID);		
		call Resource.request();	
	}
	
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}
	
	event void MilliTimer.fired() {
		call Leds.led2Toggle();
		call MultiChannel.getData();		
	}
	
	event void Resource.granted()
	{
		atomic {
			adc12memctl_t memctl[] = { {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss},{INPUT_CHANNEL_A2, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A3, REFERENCE_VREFplus_AVss}};

			if (call MultiChannel.configure(&config, memctl, 3, buffer, 4, 0) != SUCCESS) {
				call Leds.led0Toggle();
			}
		}
		call MilliTimer.startPeriodic(1000);
	}
	
	task void sendData(){
		if (busy==FALSE){
			atomic{	
				TestAdcMsg* btrpkt = (TestAdcMsg*)(call Packet.getPayload(&pkt, sizeof(TestAdcMsg)));
				btrpkt->Val1 = buffer[0];
				btrpkt->Val2 = buffer[1];
				btrpkt->Val3 = buffer[2];
				btrpkt->Val4 = buffer[3];
				call Leds.led1Toggle();
				if (call UartSend.send[AM_TESTSERIALCOMMMSG](AM_BROADCAST_ADDR, &pkt, sizeof(TestAdcMsg)) == SUCCESS) {	
					call Leds.led0Toggle();
					busy = TRUE;
				}
			}
		}
	}
	
	async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{	
		buffer[0] = buf [0];
		buffer[1] = buf [1];
		buffer[2] = buf [2];
		buffer[3] = buf [3];
		post sendData();
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
			atomic busy = FALSE;
		}
	}
}