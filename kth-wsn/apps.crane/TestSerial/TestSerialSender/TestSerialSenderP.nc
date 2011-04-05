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
#include "app_crane.h"

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

	float floatValue[NUM_CONTROL];
	uint8_t idx;
	
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

		call TimerSamples.startPeriodic(2000);

		call UartStream.enableReceiveInterrupt();
		atomic {
			for (idx= 0; idx < ( NUM_CONTROL ); idx++) {
				*(floatValue + idx) = 0.00;
			}
//			floatValue[0] = 0.00;
//			floatValue[1] = 1.00;
//			floatValue[2] = 2.00;
//			floatValue[3] = 3.00;
//			floatValue[4] = 4.00;
//			floatValue[5] = 5.00;
//			floatValue[6] = 6.00;
//			floatValue[7] = 7.00;
//			floatValue[8] = 8.00;
//			floatValue[9] = 9.00;
//			floatValue[10] = 10.00;
//			floatValue[11] = 11.00;
		}
	}

	event void TimerSamples.fired() {
		call UartResource.request();
	}

	async event void UartStream.sendDone( uint8_t* buf, uint16_t len, error_t error ) {

		atomic {
			// we have sent 4*12 = 48 bytes
			// change the value
			for (idx = 0; idx < ( NUM_CONTROL ); idx++) {
					*(floatValue + idx) = *(floatValue + idx) + 0.25;
			}idx = 0;

		}

		call UartResource.release();

		if (error != SUCCESS)
		call Leds.led0Toggle();
		else
		call Leds.led2Toggle();

	}

	async event void UartStream.receiveDone( uint8_t* buf, uint16_t len, error_t error ) {}
	async event void UartStream.receivedByte( uint8_t byte ) {}

	void printfFloat(float toBePrinted) {
		uint32_t fi, f0, f1, f2;
		char c;
		float f = toBePrinted;

		if (f<0) {
			c = '-'; f = -f;
		} else {
			c = ' ';
		}

		// integer portion.
		fi = (uint32_t) f;

		// decimal portion...get index for up to 3 decimal places.
		f = f - ((float) fi);
		f0 = f*10; f0 %= 10;
		f1 = f*100; f1 %= 10;
		f2 = f*1000; f2 %= 10;
		printf("%c%ld.%d%d%d", c, fi, (uint8_t) f0, (uint8_t) f1,
				(uint8_t) f2);
	}

	event void UartResource.granted() {
		call UartStream.send((uint8_t*)floatValue, ( NUM_CONTROL )*sizeof(float) );
	}
}
