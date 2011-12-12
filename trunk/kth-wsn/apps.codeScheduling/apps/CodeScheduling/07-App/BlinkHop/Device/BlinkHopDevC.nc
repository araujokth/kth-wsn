#include "LATIN.h"
#include "CC2420.h"

configuration BlinkHopDevC {

	provides interface OpenReceive as OpenReceiveFromLower;
	uses interface Malloc;
	uses interface IDManager;
}

implementation {
   components BlinkHopDevP;

   OpenReceiveFromLower       = BlinkHopDevP.OpenReceiveFromLower;

   Malloc                = BlinkHopDevP.Malloc;
   IDManager             = BlinkHopDevP.IDManager;


   components MainC;
   MainC.Boot<-BlinkHopDevP;


   components LedsC;
   BlinkHopDevP.Leds -> LedsC;

}
