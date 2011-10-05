/// $Id: TempP.nc,v 1.4 2006/12/12 18:23:43 vlahan Exp $

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
 * Internal component for temp sensor reading.
 *
 * @author Hu Siquan <husq@xbow.com>
 * @author David Gay
 */

module TempP {
  provides {
    interface ResourceConfigure;
    interface Atm128AdcConfig as TempConfig;
  }
  uses {
    interface GeneralIO as BatMon;
    interface GeneralIO as BatMonRef;
  }
}
implementation {
  async command uint8_t TempConfig.getChannel() {
    return CHANNEL_BATTERY_THERMISTOR;
  }

  async command uint8_t TempConfig.getRefVoltage() {
    return ATM128_ADC_VREF_OFF;
  }

  async command uint8_t TempConfig.getPrescaler() {
    return ATM128_ADC_PRESCALE;
  }

  async command void ResourceConfigure.configure() {
    call BatMonRef.makeOutput();
    call BatMonRef.set();
    call BatMon.makeOutput();
    call BatMon.clr();
  }

  async command void ResourceConfigure.unconfigure() {
    call BatMon.makeInput();
    call BatMonRef.makeInput();
  }
}
