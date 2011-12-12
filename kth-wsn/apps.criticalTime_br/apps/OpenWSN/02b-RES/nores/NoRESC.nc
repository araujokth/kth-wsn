#include "OpenWSN.h"

configuration NoRESC {
   //down the stack
   provides interface OpenSend as OpenSendFromUpper;
   provides interface OpenSend as OpenSendFromBridge;
   provides interface OpenSend as OpenSendKAFromNeighbors;
   uses     interface OpenSend as OpenSendToLower;
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower;
   uses     interface OpenReceive as OpenReceiveADVToNeighbor;
   uses     interface OpenReceive as OpenReceiveToBridge;
   uses     interface OpenReceive as OpenReceiveToUpper;
   //misc
   uses interface NeighborGet;
   uses interface Malloc;
   uses interface OpenSerial;
   uses interface IDManager;
   provides interface DebugPrint;
}
implementation {
   components NoRESP;
   //down the stack
   OpenSendFromUpper        = NoRESP.OpenSendFromUpper;
   OpenSendFromBridge       = NoRESP.OpenSendFromBridge;
   OpenSendKAFromNeighbors  = NoRESP.OpenSendKAFromNeighbors;
   OpenSendToLower          = NoRESP.OpenSendToLower;
   //up the stack
   OpenReceiveFromLower     = NoRESP.OpenReceiveFromLower;
   OpenReceiveADVToNeighbor = NoRESP.OpenReceiveADVToNeighbor;
   OpenReceiveToBridge      = NoRESP.OpenReceiveToBridge;
   OpenReceiveToUpper       = NoRESP.OpenReceiveToUpper;
   //misc
   NeighborGet              = NoRESP.NeighborGet;
   Malloc                   = NoRESP.Malloc;
   OpenSerial               = NoRESP.OpenSerial;
   IDManager                = NoRESP.IDManager;
   DebugPrint               = NoRESP.DebugPrint;

   components MainC;
   MainC.SoftwareInit->NoRESP;
}
