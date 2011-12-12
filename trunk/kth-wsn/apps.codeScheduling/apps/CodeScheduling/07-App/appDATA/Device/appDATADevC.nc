#include "LATIN.h"

configuration appDATADevC {

	provides interface OpenReceive as OpenReceiveFromLower;

	uses interface OpenSend as OpenSendToLower;

	uses interface Malloc;

	uses interface IDManager;

}

implementation {
	components appDATADevP;

	OpenReceiveFromLower = appDATADevP.OpenReceiveFromLower;
	OpenSendToLower = appDATADevP.OpenSendToLower;

	Malloc = appDATADevP.Malloc;

	components MainC;
	MainC.SoftwareInit->appDATADevP;

	components LedsC;
	appDATADevP.Leds -> LedsC;

	components PacketFunctionsC;
	appDATADevP.PacketFunctions->PacketFunctionsC;

	IDManager =appDATADevP.IDManager;

}
