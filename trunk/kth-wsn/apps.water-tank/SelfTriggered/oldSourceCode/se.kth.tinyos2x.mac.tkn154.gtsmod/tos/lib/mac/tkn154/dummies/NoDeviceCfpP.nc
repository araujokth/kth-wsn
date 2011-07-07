

/** 
 * The contention free period (CFP) in beacon mode, a.k.a. GTS, is not yet
 * implemented - this is only an empty placeholder. In contrast to the CAP
 * component the GTS component for an incoming superframe will probably be very
 * different from the GTS for an outgoing superframe. That is why there are two
 * separate placeholder components (DeviceCfpP and CoordCfpP) instead of one
 * generic CfpP component. This component would deal with the GTS for an
 * incoming superframe, i.e. from the perspective of a device.
 */

#include "TKN154_MAC.h"
module NoDeviceCfpP
{
	  provides {
	    interface Init;
	    interface Get<ieee154_GTSentry_t*> as GetGtsDeviceDb;

	  } uses {
	    
	    interface MLME_SYNC_LOSS;
		interface GtsUtility;

	  }
}
implementation
{
  command error_t Init.init()
  {
    // initialize any module variables
    return SUCCESS;
  }
    
  command ieee154_GTSentry_t* GetGtsDeviceDb.get() { return NULL;}

  event void MLME_SYNC_LOSS.indication (
                           ieee154_status_t lossReason,
                           uint16_t PANId,
                           uint8_t LogicalChannel,
                           uint8_t ChannelPage,
                           ieee154_security_t *security
                         )
   {}

  
}
