#include "LATIN.h"
#include "UserButton.h"

configuration AppTcpEchoC {
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
   components AppTcpEchoP;
   TCPControl            = AppTcpEchoP.TCPControl;
   OpenSendToLower       = AppTcpEchoP.OpenSendToLower;
   OpenReceiveFromLower  = AppTcpEchoP.OpenReceiveFromLower;
   Malloc                = AppTcpEchoP.Malloc;
   IDManager             = AppTcpEchoP.IDManager;
   OpenSerial            = AppTcpEchoP.OpenSerial;

   components MainC;
   MainC.SoftwareInit->AppTcpEchoP;

   components PacketFunctionsC;
   AppTcpEchoP.PacketFunctions->PacketFunctionsC;
}

