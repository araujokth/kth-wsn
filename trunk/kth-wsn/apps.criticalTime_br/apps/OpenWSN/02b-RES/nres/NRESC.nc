#include "OpenWSN.h"
#include "IEEE802154E.h"

configuration NRESC {
	//down the stack
	provides interface OpenSend as OpenSendFromUpper;
	provides interface OpenSendADV as OpenSendFromUpperADV;
	//   provides interface OpenSend as OpenSendFromBridge;
	provides interface OpenSend as OpenSendKAFromNeighbors;
	uses interface OpenSend as OpenSendToLower;
	//up the stack
	provides interface OpenReceive as OpenReceiveFromLower;
	uses interface OpenReceive as OpenReceiveADVToNeighbor;
	//   uses     interface OpenReceive as OpenReceiveToBridge;
	uses interface OpenReceive as OpenReceiveToUpper;
	//misc
	uses interface NeighborGet;
	uses interface Malloc;
	//   uses interface OpenSerial;
	uses interface IDManager;
	provides interface DebugPrint;
	uses interface OpenQueue;

	// TSCH Critical Time interfaces
	provides interface ControlADV as ControlADVC;
}
implementation {
	components NRESP;
	//down the stack
	OpenSendFromUpper = NRESP.OpenSendFromUpper;
	OpenSendFromUpperADV = NRESP.OpenSendFromUpperADV;
	//   OpenSendFromBridge       = NRESP.OpenSendFromBridge;
	OpenSendKAFromNeighbors = NRESP.OpenSendKAFromNeighbors;
	OpenSendToLower = NRESP.OpenSendToLower;
	//up the stack
	OpenReceiveFromLower = NRESP.OpenReceiveFromLower;
	OpenReceiveADVToNeighbor = NRESP.OpenReceiveADVToNeighbor;
	//   OpenReceiveToBridge      = NRESP.OpenReceiveToBridge;
	OpenReceiveToUpper = NRESP.OpenReceiveToUpper;
	//misc
	NeighborGet = NRESP.NeighborGet;
	Malloc = NRESP.Malloc;
	//   OpenSerial               = NRESP.OpenSerial;
	IDManager = NRESP.IDManager;
	DebugPrint = NRESP.DebugPrint;

	components new TimerMilliC() as TimerADV;
	NRESP.timerADV->TimerADV;

	components RandomC;
	NRESP.Random->RandomC;

	components PacketFunctionsC;
	NRESP.PacketFunctions->PacketFunctionsC;

	components MainC;
	MainC.SoftwareInit->NRESP;

	components LedsC;
	NRESP.Leds -> LedsC;

	OpenQueue = NRESP.OpenQueue;

	// TSCH Critical Time interfaces
	ControlADVC = NRESP.ControlADV;
}
