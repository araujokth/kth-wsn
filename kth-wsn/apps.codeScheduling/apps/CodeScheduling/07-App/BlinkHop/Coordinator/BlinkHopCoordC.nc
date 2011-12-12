#include "LATIN.h"
#include "CC2420.h"

configuration BlinkHopCoordC {

   uses interface OpenSend as OpenSendToLower;

   uses interface Malloc;
   uses interface IDManager;
}

implementation {
   components BlinkHopCoordP;

   OpenSendToLower       = BlinkHopCoordP.OpenSendToLower;

   Malloc                = BlinkHopCoordP.Malloc;
   IDManager             = BlinkHopCoordP.IDManager;


   components MainC;
   MainC.Boot<-BlinkHopCoordP;

   components PacketFunctionsC;
   BlinkHopCoordP.PacketFunctions->PacketFunctionsC;

   components new Alarm32khz32C() as AlarmBlinkC;
   BlinkHopCoordP.AlarmBlinkP -> AlarmBlinkC;

//   components new Alarm62500hz32VirtualizedC() as AlarmBlinkC;
//   BlinkHopCoordP.AlarmBlinkP -> AlarmBlinkC;

   components LedsC;
   BlinkHopCoordP.Leds -> LedsC;


}
