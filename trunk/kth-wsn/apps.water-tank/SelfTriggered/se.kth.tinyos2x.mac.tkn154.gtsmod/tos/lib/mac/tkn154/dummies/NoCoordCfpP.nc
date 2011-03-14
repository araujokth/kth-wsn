/**
 * The contention free period (CFP) in beacon mode, a.k.a. GTS, is not yet
 * implemented - this is only an empty placeholder. In contrast to the CAP
 * component the GTS component for an incoming superframe will probably be very
 * different from the GTS for an outgoing superframe. That is why there are two
 * separate placeholder components (DeviceCfpP and CoordCfpP) instead of one
 * generic CfpP component. This component would deal with the GTS for an
 * outgoing superframe, i.e. from the perspective of a coordinator.
 */

#include "TKN154_MAC.h"
module NoCoordCfpP
{
	provides
	{
		interface Init;
		interface WriteBeaconField as GtsInfoWrite;

		interface GetSet<ieee154_GTSdb_t*> as GetSetGtsCoordinatorDb;
	}
	uses
	{
		interface MLME_GET;
		interface MLME_SET;
		interface IEEE154Frame as Frame;
		interface IEEE154BeaconFrame as BeaconFrame;
		interface GtsUtility;
		interface GetNow<token_requested_t> as IsRadioTokenRequested;

	}
}
implementation
{
	command error_t Init.init()
	{
		// initialize any module variables
		return SUCCESS;
	}

	command uint8_t GtsInfoWrite.write(uint8_t *gtsSpecField, uint8_t maxlen)
	{
		// write the current GTS spec at the given address
		if (call GtsInfoWrite.getLength() > maxlen)
		return 0;
		gtsSpecField[0] = 0;
		return 1;
	}

	command uint8_t GtsInfoWrite.getLength()
	{
		// returns the length of the current GTS spec
		// must return the same value as GtsInfoWrite.write
		return 1;
	}
	command ieee154_GTSdb_t* GetSetGtsCoordinatorDb.get() {return NULL;}
	command void GetSetGtsCoordinatorDb.set(ieee154_GTSdb_t* dbParam) {return;}


}
