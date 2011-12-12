#include "OpenWSN.h"

configuration ICMPv6RouterC {
    //down the stack
   uses interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger as TriggerICMPv6Router;
   provides interface DebugPrint;
}
implementation {
   components ICMPv6RouterP;
   OpenSendToLower       = ICMPv6RouterP.OpenSendToLower;
   OpenReceiveFromLower  = ICMPv6RouterP.OpenReceiveFromLower;
   Malloc                = ICMPv6RouterP.Malloc;
   IDManager             = ICMPv6RouterP.IDManager;
   OpenSerial            = ICMPv6RouterP.OpenSerial;
   TriggerICMPv6Router   = ICMPv6RouterP.TriggerICMPv6Router;
   DebugPrint            = ICMPv6RouterP.DebugPrint;

   components MainC;
   MainC.SoftwareInit->ICMPv6RouterP;

   components PacketFunctionsC;
   ICMPv6RouterP.PacketFunctions->PacketFunctionsC;
}
