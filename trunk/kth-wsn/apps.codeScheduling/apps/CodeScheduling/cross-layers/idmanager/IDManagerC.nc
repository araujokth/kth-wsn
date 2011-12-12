#include "LATIN.h"

configuration IDManagerC {
   provides interface IDManager;
   provides interface DebugPrint;
   provides interface Init as SoftwareInit;
//   uses interface OpenSerial;
   provides interface Trigger as TriggerIDManagerAboutBridge;
   provides interface Trigger as TriggerIDManagerAboutRoot;
}
implementation {
   components IDManagerP;
   IDManager                   = IDManagerP.IDManager;
   SoftwareInit                = IDManagerP.SoftwareInit;
   DebugPrint                  = IDManagerP.DebugPrint;
//   OpenSerial                  = IDManagerP.OpenSerial;
   TriggerIDManagerAboutBridge = IDManagerP.TriggerIDManagerAboutBridge;
   TriggerIDManagerAboutRoot   = IDManagerP.TriggerIDManagerAboutRoot;

   components PacketFunctionsC;
   IDManagerP.PacketFunctions->PacketFunctionsC;

   components MainC;
   MainC.SoftwareInit->IDManagerP;
}
