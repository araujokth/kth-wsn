#include "LATIN.h"
#include "UserButton.h"

configuration AppTcpInjectC {
   //data flow
   uses interface TCPControl[uint16_t localPortNumber];
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger as TriggerTCPInject;
}
implementation {
   components AppTcpInjectP;
   IDManager             = AppTcpInjectP.IDManager;
   TCPControl            = AppTcpInjectP.TCPControl;
   OpenSendToLower       = AppTcpInjectP.OpenSendToLower;
   Malloc                = AppTcpInjectP.Malloc;
   OpenSerial            = AppTcpInjectP.OpenSerial;
   TriggerTCPInject      = AppTcpInjectP.TriggerTCPInject;

   components MainC;
   MainC.SoftwareInit->AppTcpInjectP;

   components PacketFunctionsC;
   AppTcpInjectP.PacketFunctions->PacketFunctionsC;
}

