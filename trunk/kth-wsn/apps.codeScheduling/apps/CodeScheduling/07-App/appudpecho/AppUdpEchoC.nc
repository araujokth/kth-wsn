#include "LATIN.h"
#include "UserButton.h"

configuration AppUdpEchoC {
   //data flow
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
}
implementation {
   components AppUdpEchoP;
   IDManager             = AppUdpEchoP.IDManager;
   OpenReceiveFromLower  = AppUdpEchoP.OpenReceiveFromLower;
   OpenSendToLower       = AppUdpEchoP.OpenSendToLower;
   Malloc                = AppUdpEchoP.Malloc;
   OpenSerial            = AppUdpEchoP.OpenSerial;

   components MainC;
   MainC.SoftwareInit->AppUdpEchoP;
}

