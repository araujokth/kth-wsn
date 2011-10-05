//$Id: HilTimerMilliC.nc,v 1.4 2006/12/12 18:23:43 vlahan Exp $

/* "Copyright (c) 2000-2003 The Regents of the University of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement
 * is hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
 * OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 */

/**
 * Millisecond timer for the mica family (see TEP102). The "millisecond"
 * timer system is built on hardware timer 0, running at 1024Hz.
 *
 * @author Cory Sharp <cssharp@eecs.berkeley.edu>
 * @author Martin Turon <mturon@xbow.com>
 */

#include "Timer.h"

configuration HilTimerMilliC
{
  provides interface Init;
  provides interface Timer<TMilli> as TimerMilli[uint8_t num];
  provides interface LocalTime<TMilli>;
}
implementation
{
  components AlarmCounterMilliP, new AlarmToTimerC(TMilli),
    new VirtualizeTimerC(TMilli, uniqueCount(UQ_TIMER_MILLI)),
    new CounterToLocalTimeC(TMilli);

  Init = AlarmCounterMilliP;

  TimerMilli = VirtualizeTimerC;
  VirtualizeTimerC.TimerFrom -> AlarmToTimerC;
  AlarmToTimerC.Alarm -> AlarmCounterMilliP;

  LocalTime = CounterToLocalTimeC;
  CounterToLocalTimeC.Counter -> AlarmCounterMilliP;
}


