#include "LATIN.h"

configuration ICMPv6EchoC {
    //down the stack
   uses interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger as TriggerICMPv6Echo;
   provides interface DebugPrint;
}
implementation {
   components ICMPv6EchoP;
   OpenSendToLower       = ICMPv6EchoP.OpenSendToLower;
   OpenReceiveFromLower  = ICMPv6EchoP.OpenReceiveFromLower;
   Malloc                = ICMPv6EchoP.Malloc;
   IDManager             = ICMPv6EchoP.IDManager;
   OpenSerial            = ICMPv6EchoP.OpenSerial;
   TriggerICMPv6Echo     = ICMPv6EchoP.TriggerICMPv6Echo;
   DebugPrint            = ICMPv6EchoP.DebugPrint;

   components MainC;
   MainC.SoftwareInit->ICMPv6EchoP;

   components PacketFunctionsC;
   ICMPv6EchoP.PacketFunctions->PacketFunctionsC;
}
