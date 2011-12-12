#include "OpenWSN.h"

configuration OpenBridgeC {
   //down the stack
   provides interface Trigger as TriggerOpenBridge;
   uses     interface OpenSend as OpenSendToLower;
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower;
   //misc
   uses     interface OpenSerial;  //also to exchange the data
   uses     interface IDManager;
   uses     interface Malloc;
}
implementation {
   components OpenBridgeP;
   TriggerOpenBridge    = OpenBridgeP.TriggerOpenBridge;
   OpenSendToLower      = OpenBridgeP.OpenSendToLower;
   OpenReceiveFromLower = OpenBridgeP.OpenReceiveFromLower;
   OpenSerial           = OpenBridgeP.OpenSerial;
   IDManager            = OpenBridgeP.IDManager;
   Malloc               = OpenBridgeP.Malloc;

   components PacketFunctionsC;
   OpenBridgeP.PacketFunctions->PacketFunctionsC;
}
