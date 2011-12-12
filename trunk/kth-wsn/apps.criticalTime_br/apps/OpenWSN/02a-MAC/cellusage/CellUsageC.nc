#include "OpenWSN.h"
#include "CellUsage.h"

configuration CellUsageC {
   provides interface Init as SoftwareInit;
   provides interface CellUsageGet;
   provides interface DebugPrint;
   provides interface CellStats;
   uses interface GlobalTime;
//   uses interface OpenSerial;
}
implementation {
   components CellUsageP;
   SoftwareInit = CellUsageP.SoftwareInit;
   CellUsageGet = CellUsageP.CellUsageGet;
   DebugPrint   = CellUsageP.DebugPrint;
   CellStats    = CellUsageP.CellStats;
   GlobalTime   = CellUsageP.GlobalTime;
//   OpenSerial   = CellUsageP.OpenSerial;
}
