// $Id: CC1000ActiveMessageC.nc,v 1.1 2008/07/12 02:25:59 liang_mike Exp $

/*
 * "Copyright (c) 2004-2005 The Regents of the University  of California.  
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
 * Copyright (c) 2004-2005 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */

/**
 *
 * The Active Message layer for the CC1000 radio. This configuration
 * just layers the AM dispatch (CC1000ActiveMessageM) on top of the
 * underlying CC1000 radio packet (CC1000CsmaRadioC), which is
 * inherently an AM packet (acknowledgements based on AM destination
 * addr and group).
 * 
 * @author Philip Levis
 * @date June 19 2005
 */

configuration CC1000ActiveMessageC {
  provides {
    interface SplitControl;
    interface AMSend[am_id_t id];
    interface Receive[am_id_t id];
    interface Receive as ReceiveDefault[am_id_t id];
    interface Receive as Snoop[am_id_t id];
    interface Receive as SnoopDefault[am_id_t id];
    interface AMPacket;
    interface Packet;
    interface PacketAcknowledgements;
    interface LinkPacketMetadata;
  }
}
implementation {

  components CC1000ActiveMessageP as AM, CC1000CsmaRadioC as Radio;
  components ActiveMessageAddressC as Address;
  
  SplitControl = Radio;
  Packet       = Radio;
  PacketAcknowledgements = Radio;
  LinkPacketMetadata = Radio;
  
  AMSend   = AM;
  Receive = AM.Receive;
  ReceiveDefault = AM.ReceiveDefault;
  Snoop = AM.Snoop;
  SnoopDefault = AM.SnoopDefault;
  AMPacket = AM;

  AM.SubSend    -> Radio.Send;
  AM.SubReceive -> Radio.Receive;
  AM.amAddress -> Address;
  AM.Packet     -> Radio;
  
}
