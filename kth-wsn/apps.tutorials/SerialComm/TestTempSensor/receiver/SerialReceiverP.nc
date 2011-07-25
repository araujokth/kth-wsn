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

module SerialReceiverP @ safe() {

uses {
	interface Boot;
	interface Leds;

	interface UartStream;
	interface Resource as UartResource;
}
provides interface Msp430UartConfigure;
}
implementation {
	typedef nx_struct SensorsValues {
		nx_uint16_t co2;
		nx_float temp;
		nx_float humid;
	}SensorsValues;

	SensorsValues m_sensor;

	uint8_t buffer[17];
	uint16_t idx;

	void printfFloat(float toBePrinted);
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
		idx = 0;
		call UartResource.request();
	}
	/**
	 * S E R I A L   R E C E I V E R
	 */
	event void UartResource.granted() {call UartStream.enableReceiveInterrupt();}
	async event void UartStream.sendDone( uint8_t* buf, uint16_t len, error_t error ) {}
	async event void UartStream.receiveDone( uint8_t* buf, uint16_t len, error_t error ) {}

	async event void UartStream.receivedByte( uint8_t byte ) {
		buffer[idx] = byte;
		if (idx >= 0 && idx <= 16)// printf("%u ",buffer[idx]); printfflush();
		idx++;

		if (byte == LF) {
			call Leds.led1Toggle();
			
			m_sensor.co2=((buffer[0]-48)*1000+(buffer[1]-48)*100+(buffer[2]-48)*10+(buffer[3]-48));
			m_sensor.temp=(float)((buffer[6]-48)*10+(buffer[7]-48)+((float)(buffer[9]-48)/10));
			m_sensor.humid=(float)((buffer[11]-48)*10+(buffer[12]-48)+((float)(buffer[14]-48)/10));

			printf("CO2: %u ",m_sensor.co2); printfflush();

			printf(" Temperature: ");
			printfFloat(m_sensor.temp);

			printf(" Humidity: ");
			printfFloat(m_sensor.humid);

			printf("\n");
			printfflush();
			idx = 0;
		}
	}

	void printfFloat(float toBePrinted) {
		uint32_t fi, f0, f1, f2;
		char c;
		float f = toBePrinted;

		if (f < 0) {
			c = '-';
			f = -f;
		} else {
			c = ' ';
		}

		// integer portion.
		fi = (uint32_t) f;

		// decimal portion...get index for up to 3 decimal places.
		f = f - ((float) fi);
		f0 = f * 10;
		f0 %= 10;
		f1 = f * 100;
		f1 %= 10;
		f2 = f * 1000;
		f2 %= 10;
		printf("%c%ld.%d%d%d", c, fi, (uint8_t) f0, (uint8_t) f1, (uint8_t) f2);
	}
}
