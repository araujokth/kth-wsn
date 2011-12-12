#include "OpenWSN.h"

configuration ICMPv6C {
   //down the stack
   provides interface OpenSend as OpenSendFromUpper[uint8_t icmpv6_type];
   uses interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   uses interface OpenReceive as OpenReceiveToUpper[uint8_t icmpv6_type];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface DebugPrint;
}
implementation {
   components ICMPv6P;
   OpenSendFromUpper     = ICMPv6P.OpenSendFromUpper;
   OpenSendToLower       = ICMPv6P.OpenSendToLower;
   OpenReceiveFromLower  = ICMPv6P.OpenReceiveFromLower;
   OpenReceiveToUpper    = ICMPv6P.OpenReceiveToUpper;
   Malloc                = ICMPv6P.Malloc;
   IDManager             = ICMPv6P.IDManager;
   OpenSerial            = ICMPv6P.OpenSerial;
   DebugPrint            = ICMPv6P.DebugPrint;

   components MainC;
   MainC.SoftwareInit->ICMPv6P;

   components PacketFunctionsC;
   ICMPv6P.PacketFunctions->PacketFunctionsC;
}
