/// $Id: HplAtm128Timer0AsyncP.nc,v 1.4 2008/06/26 03:38:27 regehr Exp $

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
 * HPL interface to Atmega128 timer 0 in ASYNC mode. This is a specialised
 * HPL component that assumes that timer 0 is used in ASYNC mode and
 * includes some workarounds for some of the weirdnesses (delayed overflow
 * interrupt) of that mode.
 *
 * @author Martin Turon <mturon@xbow.com>
 * @author David Gay <dgay@intel-research.net>
 */

#include <Atm128Timer.h>

module HplAtm128Timer0AsyncP @safe() {
  provides {
    // 8-bit Timers
    interface HplAtm128Timer<uint8_t>   as Timer;
    interface HplAtm128TimerCtrl8       as TimerCtrl;
    interface HplAtm128Compare<uint8_t> as Compare;
    interface McuPowerOverride;
    interface HplAtm128TimerAsync       as TimerAsync;
  }
}
implementation
{
  //=== Read the current timer value. ===================================
  async command uint8_t  Timer.get() { return TCNT0; }

  //=== Set/clear the current timer value. ==============================
  async command void Timer.set(uint8_t t)  {
    TCNT0 = t;
  }

  //=== Read the current timer scale. ===================================
  async command uint8_t Timer.getScale() { return TCCR0 & 0x7; }

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
    return *(Atm128TimerControl_t*)&TCCR0; 
  }

  //=== Write the control registers. ====================================
  async command void TimerCtrl.setControl( Atm128TimerControl_t x ) { 
    TCCR0 = x.flat; 
  }

  //=== Read the interrupt mask. =====================================
  async command Atm128_TIMSK_t TimerCtrl.getInterruptMask() { 
    return *(Atm128_TIMSK_t*)&TIMSK; 
  }

  //=== Write the interrupt mask. ====================================
  DEFINE_UNION_CAST(TimerMask8_2int, Atm128_TIMSK_t, uint8_t);
  DEFINE_UNION_CAST(TimerMask16_2int, Atm128_ETIMSK_t, uint8_t);

  async command void TimerCtrl.setInterruptMask( Atm128_TIMSK_t x ) { 
    TIMSK = TimerMask8_2int(x); 
  }

  //=== Read the interrupt flags. =====================================
  async command Atm128_TIFR_t TimerCtrl.getInterruptFlag() { 
    return *(Atm128_TIFR_t*)&TIFR; 
  }

  //=== Write the interrupt flags. ====================================
  DEFINE_UNION_CAST(TimerFlags8_2int, Atm128_TIFR_t, uint8_t);
  DEFINE_UNION_CAST(TimerFlags16_2int, Atm128_ETIFR_t, uint8_t);

  async command void TimerCtrl.setInterruptFlag( Atm128_TIFR_t x ) { 
    TIFR = TimerFlags8_2int(x); 
  }

  //=== Timer 8-bit implementation. ====================================
  async command void Timer.reset() { TIFR = 1 << TOV0; }
  async command void Timer.start() { SET_BIT(TIMSK, TOIE0); }
  async command void Timer.stop()  { CLR_BIT(TIMSK, TOIE0); }

  bool overflowed() {
    return (call TimerCtrl.getInterruptFlag()).bits.tov0; 
  }

  async command bool Timer.test()  { 
    return overflowed();
  }
  async command bool Timer.isOn()  { 
    return (call TimerCtrl.getInterruptMask()).bits.toie0; 
  }
  async command void Compare.reset() { TIFR = 1 << OCF0; }
  async command void Compare.start() { SET_BIT(TIMSK,OCIE0); }
  async command void Compare.stop()  { CLR_BIT(TIMSK,OCIE0); }
  async command bool Compare.test()  { 
    return (call TimerCtrl.getInterruptFlag()).bits.ocf0; 
  }
  async command bool Compare.isOn()  { 
    return (call TimerCtrl.getInterruptMask()).bits.ocie0; 
  }

  //=== Read the compare registers. =====================================
  async command uint8_t Compare.get()   { return OCR0; }

  //=== Write the compare registers. ====================================
  async command void Compare.set(uint8_t t)   { 
    OCR0 = t; 
  }

  //=== Timer interrupts signals ========================================
  inline void stabiliseTimer0() {
    TCCR0 = TCCR0;
    while (ASSR & 1 << TCR0UB)
      ;
  }

  /**
   * On the atm128, there is a small latency when waking up from
   * POWER_SAVE mode. So if a timer is going to go off very soon, it's
   * better to drop down until EXT_STANDBY, which has a 6 cycle wakeup
   * latency. This function calculates whether staying in EXT_STANDBY
   * is needed. If the timer is not running it returns POWER_DOWN.
   * Please refer to TEP 112 and the atm128 datasheet for details.
   */
  
  async command mcu_power_t McuPowerOverride.lowestState() {
    uint8_t diff;
    // We need to make sure that the sleep wakeup latency will not
    // cause us to miss a timer. POWER_SAVE 
    if (TIMSK & (1 << OCIE0 | 1 << TOIE0)) {
      // need to wait for timer 0 updates propagate before sleeping
      // (we don't need to worry about reentering sleep mode too early,
      // as the wake ups from timer0 wait at least one TOSC1 cycle
      // anyway - see the stabiliseTimer0 function)
      while (ASSR & (1 << TCN0UB | 1 << OCR0UB | 1 << TCR0UB))
	;
      diff = OCR0 - TCNT0;
      if (diff < EXT_STANDBY_T0_THRESHOLD ||
	  TCNT0 > 256 - EXT_STANDBY_T0_THRESHOLD) 
	return ATM128_POWER_EXT_STANDBY;
      return ATM128_POWER_SAVE;
    }
    else {
      return ATM128_POWER_DOWN;
    }
  }
  
  default async event void Compare.fired() { }
  AVR_ATOMIC_HANDLER(SIG_OUTPUT_COMPARE0) {
    stabiliseTimer0();
    signal Compare.fired();
  }

  default async event void Timer.overflow() { }
  AVR_ATOMIC_HANDLER(SIG_OVERFLOW0) {
    stabiliseTimer0();
    signal Timer.overflow();
  }

  // Asynchronous status register support
  async command Atm128Assr_t TimerAsync.getAssr() {
    return *(Atm128Assr_t *)&ASSR;
  }

  async command void TimerAsync.setAssr(Atm128Assr_t x) {
    ASSR = x.flat;
  }

  async command void TimerAsync.setTimer0Asynchronous() {
    ASSR |= 1 << AS0;
  }

  async command int TimerAsync.controlBusy() {
    return (ASSR & (1 << TCR0UB)) != 0;
  }

  async command int TimerAsync.compareBusy() {
    return (ASSR & (1 << OCR0UB)) != 0;
  }

  async command int TimerAsync.countBusy() {
    return (ASSR & (1 << TCN0UB)) != 0;
  }
}
