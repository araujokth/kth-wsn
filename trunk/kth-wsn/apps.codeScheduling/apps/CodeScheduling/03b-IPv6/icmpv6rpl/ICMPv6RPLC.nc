#include "LATIN.h"

configuration ICMPv6RPLC {
    //down the stack
   uses interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger as TriggerICMPv6RPL;
   provides interface DebugPrint;
}
implementation {
   components ICMPv6RPLP;
   OpenSendToLower       = ICMPv6RPLP.OpenSendToLower;
   OpenReceiveFromLower  = ICMPv6RPLP.OpenReceiveFromLower;
   Malloc                = ICMPv6RPLP.Malloc;
   IDManager             = ICMPv6RPLP.IDManager;
   OpenSerial            = ICMPv6RPLP.OpenSerial;
   TriggerICMPv6RPL      = ICMPv6RPLP.TriggerICMPv6RPL;
   DebugPrint            = ICMPv6RPLP.DebugPrint;

   components MainC;
   MainC.SoftwareInit->ICMPv6RPLP;

   components PacketFunctionsC;
   ICMPv6RPLP.PacketFunctions->PacketFunctionsC;
}
