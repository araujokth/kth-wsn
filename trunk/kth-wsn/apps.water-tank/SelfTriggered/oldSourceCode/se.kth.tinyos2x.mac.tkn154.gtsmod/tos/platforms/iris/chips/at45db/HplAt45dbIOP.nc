// $Id: HplAt45dbIOP.nc,v 1.2 2008/06/11 00:46:26 razvanm Exp $

/*
 * "Copyright (c) 2000-2003 The Regents of the University  of California.
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
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA,
 * 94704.  Attention:  Intel License Inquiry.
 */
/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 */

/**
 * Low level hardware access to the onboard AT45DB flash chip.
 * <p>
 * Note: This component includes optimised bit-banging SPI code with the
 * pins hardwired.  Don't copy it to some other platform without
 * understanding it (see txByte).
 *
 * @author Jason Hill
 * @author David Gay
 * @author Philip Levis
 * @author Janos Sallai <janos.sallai@vanderbilt.edu>
 */

#include "Timer.h"

module HplAt45dbIOP {
  provides {
    interface Init;
    interface SpiByte as FlashSpi;
    interface HplAt45dbByte;
  }
  uses {
    interface GeneralIO as Select;
    interface GeneralIO as Clk;
    interface GeneralIO as Out;
    interface GeneralIO as In;
    interface BusyWait<TMicro, uint16_t>;
  }
}
implementation
{
  // We use SPI mode 0 (clock low at select time)

  command error_t Init.init() {
    call Select.makeOutput();
    call Select.set();
    call Clk.clr();
    call Clk.makeOutput();
    call Out.set();
    call Out.makeOutput();
    call In.clr();
    call In.makeInput();
    return SUCCESS;
  }

  command void HplAt45dbByte.select() {
    call Clk.clr(); // ensure SPI mode 0
    call Select.clr();
  }

  command void HplAt45dbByte.deselect() {
    call Select.set();
  }

#define BITINIT \
  uint8_t clrClkAndData = PORTD & ~0x28

#define BIT(n) \
	PORTD = clrClkAndData; \
	asm __volatile__ \
        (  "sbrc %2," #n "\n" \
	 "\tsbi 11,3\n" \
	 "\tsbi 11,5\n" \
	 "\tsbic 9,2\n" \
	 "\tori %0,1<<" #n "\n" \
	 : "=d" (spiIn) : "0" (spiIn), "r" (spiOut))

  async command uint8_t FlashSpi.write(uint8_t spiOut) {
    uint8_t spiIn = 0;

    // This atomic ensures integrity at the hardware level...
    atomic
      {
	BITINIT;

	BIT(7);
	BIT(6);
	BIT(5);
	BIT(4);
	BIT(3);
	BIT(2);
	BIT(1);
	BIT(0);
      }

    return spiIn;
  }

  task void avail() {
    signal HplAt45dbByte.idle();
  }

  command void HplAt45dbByte.waitIdle() {
    // at45db041 rev d fix by handsomezhu hongsong at ios.cn
    // http://mail.millennium.berkeley.edu/pipermail/tinyos-help/2008-January/030255.html
    int i;
    call Clk.clr();
    call BusyWait.wait(2);
    while( ! call In.get() ) {
      for( i=0; i < 8; i ++ ) {
        call Clk.set();
        call Clk.clr();
        call BusyWait.wait(2);
      }
    }
    post avail();
  }

  command bool HplAt45dbByte.getCompareStatus() {
    call Clk.set();
    call Clk.clr();
    // Wait for compare value to propagate
    asm volatile("nop");
    asm volatile("nop");
    return !call In.get();
  }
}
