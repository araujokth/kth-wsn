#include "LATIN.h"
#include "URES.h"

configuration URESC {
   provides interface ReservationUpdate;
   provides interface DebugPrint;
   uses interface Receive;
   uses interface NeighborGet;
   uses interface SimpleSend as SendRES;
   uses interface CellUsageSet;
   uses interface CellUsageGet;
   uses interface EnQueue;
   uses interface GlobalTime;
   uses interface SerialIO;
}
implementation {
   components URESP;
   ReservationUpdate = URESP.ReservationUpdate;
   DebugPrint        = URESP.DebugPrint;
   Receive           = URESP.Receive;
   NeighborGet       = URESP.NeighborGet;
   SendRES           = URESP.SendRES;
   CellUsageSet      = URESP.CellUsageSet;
   EnQueue           = URESP.EnQueue;
   CellUsageGet      = URESP.CellUsageGet;
   GlobalTime        = URESP.GlobalTime;
   SerialIO          = URESP.SerialIO;

   components new TimerMilliC() as RemoveFrozenReservationsTimerC;
   URESP.RemoveFrozenReservationsTimer -> RemoveFrozenReservationsTimerC;

   components MainC;
   MainC.SoftwareInit->URESP;

   components RandomC;
   URESP.Random->RandomC;
}
