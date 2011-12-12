#include "printf.h"

configuration LatinMacC {
   //admin
   uses interface Boot;
   //time
   provides interface GlobalTime;
   provides interface GlobalSync;

   //down the stack
   provides interface OpenSend as OpenSendFromUpper;
   uses interface OpenQueue;
   uses interface RadioControl;
   uses interface RadioSend;
   uses interface RadioReceive;
   //up the stack
   uses interface OpenReceive as OpenReceiveToUpper;
   //misc
   uses interface NeighborStats;
   uses interface Malloc;
   uses interface IDManager;
   uses interface LatinMatrix;

   provides interface MAC;


}

implementation {
   components LatinMacP;
   Boot                = LatinMacP.Boot;
   GlobalTime          = LatinMacP.GlobalTime;
   GlobalSync          = LatinMacP.GlobalSync;
   OpenSendFromUpper   = LatinMacP.OpenSendFromUpper;
   OpenQueue           = LatinMacP.OpenQueue;
   RadioControl        = LatinMacP.RadioControl;
   RadioSend           = LatinMacP.RadioSend;
   RadioReceive        = LatinMacP.RadioReceive;
   OpenReceiveToUpper  = LatinMacP.OpenReceiveToUpper;
   NeighborStats       = LatinMacP.NeighborStats;
   Malloc              = LatinMacP.Malloc;
   IDManager           = LatinMacP.IDManager;


   components MainC;
   MainC.SoftwareInit->LatinMacP;

   components new Alarm32khz32C() as SlotAlarmC;
   LatinMacP.SlotAlarm -> SlotAlarmC;

   components new Alarm32khz32C() as FastAlarmC;
   LatinMacP.FastAlarm -> FastAlarmC;

   components LedsC;
   LatinMacP.Leds -> LedsC;

	// D E B U G
#ifdef TSCHDEBUG_ENABLED
	components PinDebugC as PinDebug;
#endif
	LatinMacP.PinDebug -> PinDebug;

   components RandomC;
   LatinMacP.Random->RandomC;

   components PacketFunctionsC;
   LatinMacP.PacketFunctions->PacketFunctionsC;

   LatinMacP.LatinMatrix = LatinMatrix;


   LatinMacP.MAC = MAC;
}
