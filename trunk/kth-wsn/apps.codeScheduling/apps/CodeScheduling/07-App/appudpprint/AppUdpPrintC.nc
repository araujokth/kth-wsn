#include "LATIN.h"
#include "UserButton.h"

configuration AppUdpPrintC {
   //data flow
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
}
implementation {
   components AppUdpPrintP;
   IDManager             = AppUdpPrintP.IDManager;
   OpenReceiveFromLower  = AppUdpPrintP.OpenReceiveFromLower;
   Malloc                = AppUdpPrintP.Malloc;
   OpenSerial            = AppUdpPrintP.OpenSerial;

   components MainC;
   MainC.SoftwareInit->AppUdpPrintP;
}

