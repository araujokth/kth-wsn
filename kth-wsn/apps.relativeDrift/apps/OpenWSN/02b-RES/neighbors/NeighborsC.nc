#include "OpenWSN.h"
#include "Neighbors.h"

configuration NeighborsC {
   provides interface NeighborGet;
   provides interface DebugPrint;
   provides interface NeighborStats;
   provides interface OpenReceive as OpenReceiveADVFromLower;
   uses interface GlobalTime;
   uses interface GlobalSync;
   uses interface OpenQueue;
   uses interface OpenSend as OpenSendKAToLower;
   uses interface Malloc;
//   uses interface OpenSerial;
   uses interface IDManager;
}
implementation {
   components NeighborsP;
   NeighborGet              = NeighborsP.NeighborGet;
   DebugPrint               = NeighborsP.DebugPrint;
   NeighborStats            = NeighborsP.NeighborStats;
   GlobalTime               = NeighborsP.GlobalTime;
   GlobalSync               = NeighborsP.GlobalSync;
   OpenQueue                = NeighborsP.OpenQueue;
   OpenReceiveADVFromLower  = NeighborsP.OpenReceiveADVFromLower;
   OpenSendKAToLower        = NeighborsP.OpenSendKAToLower;
   Malloc                   = NeighborsP.Malloc;
//   OpenSerial               = NeighborsP.OpenSerial;
   IDManager                = NeighborsP.IDManager;

   components MainC;
   MainC.SoftwareInit->NeighborsP;

   components RandomC;
   NeighborsP.Random->RandomC;

   components PacketFunctionsC;
   NeighborsP.PacketFunctions->PacketFunctionsC;
}
