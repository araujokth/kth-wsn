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
 *
 * Date last modified:  $Id: MainC.nc,v 1.3 2008/09/25 03:14:28 regehr Exp $
 */

/**
 * MainC is the system interface the TinyOS boot sequence. It wires the
 * boot sequence implementation to the scheduler and hardware resources.
 *
 * @author Philip Levis
 * @author Kevin Klues <klueska@cs.stanford.edu>
 */

#include "hardware.h"

configuration MainC {
  provides interface Boot;
  uses {
    interface Init as SoftwareInit;
  }
}
implementation {
  components PlatformC;
  components TinyOSMainP;
  components RealMainP;
  
  components TinyTaskSchedulerC;
  components TinyThreadSchedulerC;
  components StaticThreadC;
    
#ifdef SAFE_TINYOS
  components SafeFailureHandlerC;
#endif

  // Export the SoftwareInit and Boot for applications
  SoftwareInit = TinyOSMainP.SoftwareInit;
  Boot = TinyOSMainP;
  
  //Wire up the platform specific code
  TinyOSMainP.PlatformInit -> PlatformC;
  TinyOSMainP.TaskScheduler -> TinyTaskSchedulerC;
  
  //Wire up the interdependent task and thread schedulers
  TinyTaskSchedulerC.ThreadScheduler -> TinyThreadSchedulerC;
  
  //Wire up the TinyOS code to its thread
  StaticThreadC.ThreadInfo[TOSTHREAD_TOS_THREAD_ID] -> TinyOSMainP;
  TinyOSMainP.TinyOSBoot -> TinyThreadSchedulerC;
  
  //Wire up the thread scheduler to start running
  TinyThreadSchedulerC.ThreadSchedulerBoot -> RealMainP;  
}

