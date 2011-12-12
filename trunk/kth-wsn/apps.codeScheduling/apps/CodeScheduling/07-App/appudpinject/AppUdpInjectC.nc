#include "LATIN.h"
#include "UserButton.h"

configuration AppUdpInjectC {
   //data flow
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger as TriggerUDPInject;
}
implementation {
   components AppUdpInjectP;
   IDManager             = AppUdpInjectP.IDManager;
   OpenSendToLower       = AppUdpInjectP.OpenSendToLower;
   Malloc                = AppUdpInjectP.Malloc;
   OpenSerial            = AppUdpInjectP.OpenSerial;
   TriggerUDPInject      = AppUdpInjectP.TriggerUDPInject;

   components MainC;
   MainC.SoftwareInit->AppUdpInjectP;

   components PacketFunctionsC;
   AppUdpInjectP.PacketFunctions->PacketFunctionsC;
}

