#include "LATIN.h"

configuration TCPC {
   //down the stack
   provides interface TCPControl[uint16_t localPortNumber];
   provides interface OpenSend as OpenSendFromUpper[uint16_t localPortNumber];
   uses     interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   uses     interface OpenReceive as OpenReceiveToUpper[uint16_t localPortNumber];
   //misc
   uses     interface IDManager; //for TCP checksum
   uses     interface Malloc;
   uses     interface OpenSerial;
}

implementation {
   components TCPP;
   TCPControl            = TCPP.TCPControl;
   OpenSendFromUpper     = TCPP.OpenSendFromUpper;
   OpenSendToLower       = TCPP.OpenSendToLower;
   OpenReceiveFromLower  = TCPP.OpenReceiveFromLower;
   OpenReceiveToUpper    = TCPP.OpenReceiveToUpper;
   IDManager             = TCPP.IDManager;
   Malloc                = TCPP.Malloc;
   OpenSerial            = TCPP.OpenSerial;

   components MainC;
   MainC.SoftwareInit->TCPP;

   components PacketFunctionsC;
   TCPP.PacketFunctions->PacketFunctionsC;
}
