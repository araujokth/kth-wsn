#include "OpenWSN.h"

configuration UDPC {
   //down the stack
   provides interface OpenSend as OpenSendFromUpper[uint16_t localPortNumber];
   uses     interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   uses     interface OpenReceive as OpenReceiveToUpper[uint16_t localPortNumber];
   //misc
   uses     interface IDManager; //for UDP checksum
   uses     interface OpenSerial;
   uses     interface Malloc;
}
implementation {
   components UDPP;
   OpenSendFromUpper     = UDPP.OpenSendFromUpper;
   OpenSendToLower       = UDPP.OpenSendToLower;
   OpenReceiveFromLower  = UDPP.OpenReceiveFromLower;
   OpenReceiveToUpper    = UDPP.OpenReceiveToUpper;
   IDManager             = UDPP.IDManager;
   OpenSerial            = UDPP.OpenSerial;
   Malloc                = UDPP.Malloc;

   components PacketFunctionsC;
   UDPP.PacketFunctions->PacketFunctionsC;
}
