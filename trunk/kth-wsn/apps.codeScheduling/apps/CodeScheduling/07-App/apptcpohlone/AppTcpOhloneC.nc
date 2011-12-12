#include "LATIN.h"
#include "UserButton.h"

configuration AppTcpOhloneC {
   //data flow
   uses interface TCPControl[uint16_t localPortNumber];
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
}
implementation {
   components AppTcpOhloneP;
   TCPControl            = AppTcpOhloneP.TCPControl;
   OpenSendToLower       = AppTcpOhloneP.OpenSendToLower;
   OpenReceiveFromLower  = AppTcpOhloneP.OpenReceiveFromLower;
   Malloc                = AppTcpOhloneP.Malloc;
   IDManager             = AppTcpOhloneP.IDManager;
   OpenSerial            = AppTcpOhloneP.OpenSerial;

   components MainC;
   MainC.SoftwareInit->AppTcpOhloneP;

   components PacketFunctionsC;
   AppTcpOhloneP.PacketFunctions->PacketFunctionsC;
}

