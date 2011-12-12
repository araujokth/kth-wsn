#include "LATIN.h"
#include "UserButton.h"

configuration AppTcpPrintC {
   //data flow
   uses interface TCPControl[uint16_t localPortNumber];
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
}
implementation {
   components AppTcpPrintP;
   IDManager             = AppTcpPrintP.IDManager;
   TCPControl            = AppTcpPrintP.TCPControl;
   OpenReceiveFromLower  = AppTcpPrintP.OpenReceiveFromLower;
   Malloc                = AppTcpPrintP.Malloc;
   OpenSerial            = AppTcpPrintP.OpenSerial;

   components MainC;
   MainC.SoftwareInit->AppTcpPrintP;
}

