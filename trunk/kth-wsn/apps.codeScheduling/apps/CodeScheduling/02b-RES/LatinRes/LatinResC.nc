#include "LATIN.h"
#include "IEEE802154E.h"
#include "UserButton.h"

configuration LatinResC {
	//down the stack
	provides interface OpenSend as OpenSendFromUpper;
	uses interface OpenSend as OpenSendToLower;
	//up the stack
	provides interface OpenReceive as OpenReceiveFromLower;
	uses interface OpenReceive as OpenReceiveToUpper;
	//misc
	uses interface Malloc;
	uses interface IDManager;
	uses interface OpenQueue;
	// Code Scheduling
	uses interface MAC;
	provides interface LatinMatrix;

}
implementation {
	components LatinResP;
	//down the stack
	OpenSendFromUpper = LatinResP.OpenSendFromUpper;
	OpenSendToLower = LatinResP.OpenSendToLower;
	//up the stack
	OpenReceiveFromLower = LatinResP.OpenReceiveFromLower;
	OpenReceiveToUpper = LatinResP.OpenReceiveToUpper;
	//misc
	Malloc = LatinResP.Malloc;
	IDManager = LatinResP.IDManager;
	components PacketFunctionsC;
	LatinResP.PacketFunctions->PacketFunctionsC;
	OpenQueue = LatinResP.OpenQueue;

	components MainC;
	MainC.SoftwareInit->LatinResP;

	components UserButtonC;
	LatinResP.UserButton -> UserButtonC;
	// Code Scheduling
	LatinResP.MAC = MAC;
	LatinMatrix = LatinResP.LatinMatrix;
	//Debug
	components LedsC;
	LatinResP.Leds -> LedsC;
	// Timers
	components new TimerMilliC() as TimerAdvWait;
	LatinResP.timerAdvWait->TimerAdvWait;

}
