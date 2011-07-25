/*
 * Copyright (c) 2010, KTH Royal Institute of Technology
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
 * @version $Revision: 1.0 $ $Date: 2010/01/25 $
 */
#ifndef app_profile_H
#define app_profile_H

//  UBR_1MHZ_1200=0x0369,   UMCTL_1MHZ_1200=0x7B,
//  UBR_1MHZ_1800=0x0246,   UMCTL_1MHZ_1800=0x55,
//  UBR_1MHZ_2400=0x01B4,   UMCTL_1MHZ_2400=0xDF,
//  UBR_1MHZ_4800=0x00DA,   UMCTL_1MHZ_4800=0xAA,
//  UBR_1MHZ_9600=0x006D,   UMCTL_1MHZ_9600=0x44,
//  UBR_1MHZ_19200=0x0036,  UMCTL_1MHZ_19200=0xB5,
//  UBR_1MHZ_38400=0x001B,  UMCTL_1MHZ_38400=0x94,
//  UBR_1MHZ_57600=0x0012,  UMCTL_1MHZ_57600=0x84,
//  UBR_1MHZ_76800=0x000D,  UMCTL_1MHZ_76800=0x6D,
//  UBR_1MHZ_115200=0x0009, UMCTL_1MHZ_115200=0x10,
//  UBR_1MHZ_230400=0x0004, UMCTL_1MHZ_230400=0x55,
enum {
	// serial comm
	BLANK = 0x20,
	CR = 0x0D,
	LF = 0x0A,

	BAUD_RATE_UBR = 0x0009, // maximum for 12 floats 115200
	BAUD_RATE_UMCTL = 0x10,
// maximum for 12 floats 115200
};

#endif
