/**
 *  Copyright (c) 2005-2006 Crossbow Technology, Inc.
 *  All rights reserved.
 *
 *  Permission to use, copy, modify, and distribute this software and its
 *  documentation for any purpose, without fee, and without written
 *  agreement is hereby granted, provided that the above copyright
 *  notice, the (updated) modification history and the author appear in
 *  all copies of this source code.
 *
 *  Permission is also granted to distribute this software under the
 *  standard BSD license as contained in the TinyOS distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS `AS IS'
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS
 *  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, LOSS OF USE, DATA,
 *  OR PROFITS) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 */
 /*
 *  @author Hu Siquan <husq@xbow.com>
 *
 *  $Id: MicStreamC.nc,v 1.2 2008/06/11 00:42:14 razvanm Exp $
 */

#include "mts300.h"

generic configuration MicStreamC() {
  provides interface ReadStream<uint16_t>;
  provides interface MicSetting;
}
implementation {
  enum {
    ID = unique(UQ_MIC_RESOURCE)
  };
  components MicReadStreamP, MicDeviceP, new AdcReadStreamClientC();

  ReadStream = MicReadStreamP.ReadStream[ID];
  MicReadStreamP.ActualRead[ID] -> AdcReadStreamClientC;
  AdcReadStreamClientC.Atm128AdcConfig -> MicDeviceP.Atm128AdcConfig;
  MicSetting = MicReadStreamP;
}
