#include "OpenWSN.h"

configuration OpenSerialC {
   provides interface OpenSerial;
   provides interface DebugPrint;
   uses interface IDManager;
   uses interface Trigger as TriggerIDManagerAboutBridge;
   uses interface Trigger as TriggerIDManagerAboutRoot;
   uses interface Trigger as TriggerTCPInject;
   uses interface Trigger as TriggerUDPInject;
   uses interface Trigger as TriggerUDPSensor;
   uses interface Trigger as TriggerICMPv6Echo;
   uses interface Trigger as TriggerICMPv6Router;
   uses interface Trigger as TriggerICMPv6RPL;
   uses interface Trigger as TriggerOpenBridge;
   uses interface DebugPrint as PrintOpenSerial;
   uses interface DebugPrint as PrintCellUsage;
   uses interface DebugPrint as PrintNeighbors;
   uses interface DebugPrint as PrintOpenQueue;
   uses interface DebugPrint as PrintIEEE802154E;
   uses interface DebugPrint as PrintMyDAGrank;
   uses interface DebugPrint as PrintRPL;
   uses interface DebugPrint as PrintIDManager;
}
implementation {
   components MainC;
   MainC.SoftwareInit->OpenSerialP;

   components OpenSerialP;
   OpenSerial                   = OpenSerialP.OpenSerial;
   IDManager                    = OpenSerialP.IDManager;
   DebugPrint                   = OpenSerialP.DebugPrint;
   PrintOpenSerial              = OpenSerialP.PrintOpenSerial;
   PrintCellUsage               = OpenSerialP.PrintCellUsage;
   PrintNeighbors               = OpenSerialP.PrintNeighbors;
   PrintOpenQueue               = OpenSerialP.PrintOpenQueue;
   PrintIEEE802154E             = OpenSerialP.PrintIEEE802154E;
   PrintMyDAGrank               = OpenSerialP.PrintMyDAGrank;
   PrintRPL                     = OpenSerialP.PrintRPL;
   PrintIDManager               = OpenSerialP.PrintIDManager;
   TriggerIDManagerAboutBridge  = OpenSerialP.TriggerIDManagerAboutBridge;
   TriggerIDManagerAboutRoot    = OpenSerialP.TriggerIDManagerAboutRoot;
   TriggerTCPInject             = OpenSerialP.TriggerTCPInject;
   TriggerUDPInject             = OpenSerialP.TriggerUDPInject;
   TriggerUDPSensor             = OpenSerialP.TriggerUDPSensor;
   TriggerICMPv6Echo            = OpenSerialP.TriggerICMPv6Echo;
   TriggerICMPv6Router          = OpenSerialP.TriggerICMPv6Router;
   TriggerICMPv6RPL             = OpenSerialP.TriggerICMPv6RPL;
   TriggerOpenBridge            = OpenSerialP.TriggerOpenBridge;

   components HplMsp430Usart1C;
   OpenSerialP.HplMsp430UsartInterrupts->HplMsp430Usart1C;
   OpenSerialP.AsyncStdControl->HplMsp430Usart1C;
   OpenSerialP.HplMsp430Usart->HplMsp430Usart1C;

   components HplMsp430GeneralIOC;
   OpenSerialP.Port63 -> HplMsp430GeneralIOC.Port63;

   components LedsC;
   OpenSerialP.Leds -> LedsC;

   components UserButtonC;
   OpenSerialP.Notify->UserButtonC;
}
