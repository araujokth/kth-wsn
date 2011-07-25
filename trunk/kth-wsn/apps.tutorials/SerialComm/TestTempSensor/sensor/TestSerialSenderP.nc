/*
 * Copyright (c) 2011, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of the KTH Royal Institute of Technology nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @version $Revision: 1.0 $ $Date: 2011/01/25 $
 */
#include "printf.h"
#include "app_profile.h"

module TestSerialSenderP @ safe() {

uses {
	interface Boot;
	interface Leds;

	interface Timer<TMilli> as TimerSamples;
	interface UartStream;
	interface Resource as UartResource;

	interface Random;

}
provides interface Msp430UartConfigure;
}
implementation {
	
	uint8_t v[] = {48, 55, 49, 48, 32, 32, 50, 54, 46, 56, 32, 52, 51, 46, 49, 13, 10};
	
	msp430_uart_union_config_t msp430_uart_config = {
		{
			utxe : 1,
			urxe : 1,
			ubr : BAUD_RATE_UBR,
			umctl : BAUD_RATE_UMCTL,
			ssel : 0x02,
			pena : 0,
			pev : 0,
			spb : 0,
			clen : 1,
			listen : 0,
			mm : 0,
			ckpl : 0,
			urxse : 0,
			urxeie : 1,
			urxwie : 0
		}
	};

	async command msp430_uart_union_config_t* Msp430UartConfigure.getConfig() {
		return &msp430_uart_config;
	}

	event void Boot.booted() {

		call UartStream.enableReceiveInterrupt();
		call UartResource.request();
	}

	event void TimerSamples.fired() {
		call UartStream.send(v,  17*sizeof(uint8_t) );
	}

	async event void UartStream.sendDone( uint8_t* buf, uint16_t len, error_t error ) {

		if (error != SUCCESS)
		call Leds.led0Toggle();
		else
		call Leds.led2Toggle();

	}

	async event void UartStream.receiveDone( uint8_t* buf, uint16_t len, error_t error ) {}
	async event void UartStream.receivedByte( uint8_t byte ) {}


	event void UartResource.granted() {
		call TimerSamples.startPeriodic(2000);
	}
}
