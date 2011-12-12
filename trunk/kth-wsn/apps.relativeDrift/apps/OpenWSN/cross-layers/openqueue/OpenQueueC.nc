#include "OpenWSN.h"
#include "OpenQueue.h"

configuration OpenQueueC {
   provides interface OpenQueue;
   provides interface Malloc;
   provides interface DebugPrint;
   uses interface GlobalTime;
//   uses interface OpenSerial;
}
implementation {
   components OpenQueueP;
   OpenQueue     = OpenQueueP.OpenQueue;
   Malloc        = OpenQueueP.Malloc;
   DebugPrint    = OpenQueueP.DebugPrint;
   GlobalTime    = OpenQueueP.GlobalTime;
//   OpenSerial      = OpenQueueP.OpenSerial;

   components MainC;
   MainC.SoftwareInit->OpenQueueP;
   components LedsC;
   OpenQueueP.Leds -> LedsC;
}
