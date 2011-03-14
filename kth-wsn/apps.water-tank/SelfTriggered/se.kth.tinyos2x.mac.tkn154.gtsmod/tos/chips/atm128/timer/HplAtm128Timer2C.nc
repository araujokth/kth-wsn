/// $Id: HplAtm128Timer2C.nc,v 1.4 2006/12/12 18:23:04 vlahan Exp $

/*
 * Copyright (c) 2004-2005 Crossbow Technology, Inc.  All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL CROSSBOW TECHNOLOGY OR ANY OF ITS LICENSORS BE LIABLE TO 
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
 * IF CROSSBOW OR ITS LICENSOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH 
 * DAMAGE. 
 *
 * CROSSBOW TECHNOLOGY AND ITS LICENSORS SPECIFICALLY DISCLAIM ALL WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS 
 * ON AN "AS IS" BASIS, AND NEITHER CROSSBOW NOR ANY LICENSOR HAS ANY 
 * OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR 
 * MODIFICATIONS.
 */

/**
 * HPL interface to Atmega128 timer 2.
 *
 * @author Martin Turon <mturon@xbow.com>
 */

#include <Atm128Timer.h>

module HplAtm128Timer2C
{
  provides {
    interface HplAtm128Timer<uint8_t>   as Timer;
    interface HplAtm128TimerCtrl8       as TimerCtrl;
    interface HplAtm128Compare<uint8_t> as Compare;
  }
}
implementation
{
  //=== Read the current timer value. ===================================
  async command uint8_t  Timer.get() { return TCNT2; }

  //=== Set/clear the current timer value. ==============================
  async command void Timer.set(uint8_t t)  { TCNT2 = t; }

  //=== Read the current timer scale. ===================================
  async command uint8_t Timer.getScale() { return TCCR2 & 0x7; }

  //=== Turn off the timers. ============================================
  async command void Timer.off() { call Timer.setScale(AVR_CLOCK_OFF); }

  //=== Write a new timer scale. ========================================
  async command void Timer.setScale(uint8_t s)  { 
    Atm128TimerControl_t x = call TimerCtrl.getControl();
    x.bits.cs = s;
    call TimerCtrl.setControl(x);  
  }

  //=== Read the control registers. =====================================
  async command Atm128TimerControl_t TimerCtrl.getControl() { 
    return *(Atm128TimerControl_t*)&TCCR2; 
  }

  //=== Control registers utilities. ==================================
  DEFINE_UNION_CAST(TimerCtrlCompareint, Atm128TimerCtrlCompare_t, uint16_t);
  DEFINE_UNION_CAST(TimerCtrlCapture2int, Atm128TimerCtrlCapture_t, uint16_t);
  DEFINE_UNION_CAST(TimerCtrlClock2int, Atm128TimerCtrlClock_t, uint16_t);

  //=== Write the control registers. ====================================
  async command void TimerCtrl.setControl( Atm128TimerControl_t x ) { 
    TCCR2 = x.flat; 
  }

  //=== Read the interrupt mask. =====================================
  async command Atm128_TIMSK_t TimerCtrl.getInterruptMask() { 
    return *(Atm128_TIMSK_t*)&TIMSK; 
  }

  //=== Write the interrupt mask. ====================================
  DEFINE_UNION_CAST(TimerMask8_2int, Atm128_TIMSK_t, uint8_t);

  async command void TimerCtrl.setInterruptMask( Atm128_TIMSK_t x ) { 
    TIMSK = TimerMask8_2int(x); 
  }

  //=== Read the interrupt flags. =====================================
  async command Atm128_TIFR_t TimerCtrl.getInterruptFlag() { 
    return *(Atm128_TIFR_t*)&TIFR; 
  }

  //=== Write the interrupt flags. ====================================
  DEFINE_UNION_CAST(TimerFlags8_2int, Atm128_TIFR_t, uint8_t);

  async command void TimerCtrl.setInterruptFlag( Atm128_TIFR_t x ) { 
    TIFR = TimerFlags8_2int(x); 
  }

  //=== Timer 8-bit implementation. ====================================
  async command void Timer.reset() { TIFR = 1 << TOV2; }
  async command void Timer.start() { SET_BIT(TIMSK,TOIE2); }
  async command void Timer.stop()  { CLR_BIT(TIMSK,TOIE2); }
  async command bool Timer.test()  { 
    return (call TimerCtrl.getInterruptFlag()).bits.tov2; 
  }
  async command bool Timer.isOn()  { 
    return (call TimerCtrl.getInterruptMask()).bits.toie2; 
  }
  async command void Compare.reset() { TIFR = 1 << OCF2; }
  async command void Compare.start() { SET_BIT(TIMSK,OCIE2); }
  async command void Compare.stop()  { CLR_BIT(TIMSK,OCIE2); }
  async command bool Compare.test()  { 
    return (call TimerCtrl.getInterruptFlag()).bits.ocf2; 
  }
  async command bool Compare.isOn()  { 
    return (call TimerCtrl.getInterruptMask()).bits.ocie2; 
  }

  //=== Read the compare registers. =====================================
  async command uint8_t Compare.get()   { return OCR2; }

  //=== Write the compare registers. ====================================
  async command void Compare.set(uint8_t t)   { OCR2 = t; }

  //=== Timer interrupts signals ========================================
  default async event void Compare.fired() { }
  AVR_NONATOMIC_HANDLER(SIG_OUTPUT_COMPARE2) {
    signal Compare.fired();
  }
  default async event void Timer.overflow() { }
  AVR_NONATOMIC_HANDLER(SIG_OVERFLOW2) {
    signal Timer.overflow();
  }
}
