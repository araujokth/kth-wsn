// $Id: ActiveMessageC.nc,v 1.1 2008/07/24 20:07:21 liang_mike Exp $

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
/*
 *
 * Authors:             Philip Levis
 * Date last modified:  $Id: ActiveMessageC.nc,v 1.1 2008/07/24 20:07:21 liang_mike Exp $
 *
 */

/**
 *
 * The Active Message layer on the eyesIFX platforms. This is a naming wrapper
 * around the TDA5250 Active Message layer.
 *
 * @author Philip Levis
 * @author Vlado Handziski (TDA5250 modifications)
 * @date July 20 2005
 */

#include "Timer.h"

configuration ActiveMessageC {
  provides {
    interface SplitControl;

    interface AMSend[uint8_t id];
    interface Receive[am_id_t id];
    interface Receive as ReceiveDefault[am_id_t id];
    interface Receive as Snoop[am_id_t id];
    interface Receive as SnoopDefault[am_id_t id];

    interface Packet;
    interface AMPacket;

    interface PacketAcknowledgements;

    interface PacketTimeStamp<T32khz, uint32_t> as PacketTimeStamp32khz;
    interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;
  }
}
implementation {
  components ActiveMessageFilterC as Filter;
  components Tda5250ActiveMessageC as AM;
  components PacketStampC as PacketStamp;

  AMSend         = Filter;
  Receive        = Filter.Receive;
  ReceiveDefault = Filter.ReceiveDefault;
  Snoop          = Filter.Snoop;
  SnoopDefault   = Filter.SnoopDefault;

  Filter.SubAMSend  -> AM;
  Filter.SubReceive -> AM.Receive;
  Filter.SubSnoop   -> AM.Snoop;
  //Filter.AMPacket  -> AM;

  SplitControl = AM;
  Packet       = AM;
  AMPacket     = AM;

  PacketAcknowledgements = AM;

  PacketTimeStamp32khz = PacketStamp;
  PacketTimeStampMilli = PacketStamp;
}
