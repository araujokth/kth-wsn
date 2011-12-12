#include "LATIN.h"

configuration ForwardingC {
   //down the stack
   provides interface OpenSend as OpenSendFromUpper[uint8_t iana_number];
   uses     interface OpenSend as OpenSendToLower;
   //up the stack
   provides interface OpenReceiveIp;
   uses     interface OpenReceive as OpenReceiveToUpper[uint8_t iana_number];
   //misc
   provides interface DebugPrint;
   uses interface Malloc;
   uses interface IDManager;
   uses interface NeighborGet;
   uses interface OpenSerial;
}
implementation {
   components ForwardingP;
   OpenSendFromUpper  = ForwardingP.OpenSendFromUpper;
   OpenSendToLower    = ForwardingP.OpenSendToLower;
   OpenReceiveIp      = ForwardingP.OpenReceiveIp;
   OpenReceiveToUpper = ForwardingP.OpenReceiveToUpper;
   DebugPrint         = ForwardingP.DebugPrint;
   Malloc             = ForwardingP.Malloc;
   IDManager          = ForwardingP.IDManager;
   NeighborGet        = ForwardingP.NeighborGet;
   OpenSerial         = ForwardingP.OpenSerial;

   components MainC;
   MainC.SoftwareInit->ForwardingP;

   components PacketFunctionsC;
   ForwardingP.PacketFunctions->PacketFunctionsC;
}
