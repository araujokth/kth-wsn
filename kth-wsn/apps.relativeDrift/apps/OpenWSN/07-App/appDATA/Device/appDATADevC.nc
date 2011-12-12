#include "OpenWSN.h"

configuration appDATADevC {

	provides interface OpenReceive as OpenReceiveFromLower;

	// TSCH Critical Time interfaces
	provides interface CriticalTime as CriticalTimeC;

	uses interface OpenSend as OpenSendToLower;
	uses interface OpenSendADV as OpenSendToLowerADV;
	uses interface Malloc;

	uses interface NeighborGet;

	uses interface IDManager;
	uses interface ControlSync;

	// TSCH Critical Time interfaces
	uses interface ControlADV as ControlADVC;


}

implementation {
   components appDATADevP;

   OpenReceiveFromLower       = appDATADevP.OpenReceiveFromLower;
   OpenSendToLower = appDATADevP.OpenSendToLower;
   OpenSendToLowerADV = appDATADevP.OpenSendToLowerADV;
   Malloc                = appDATADevP.Malloc;
   ControlSync	= appDATADevP.ControlSync;



   components MainC;
   MainC.SoftwareInit->appDATADevP;


   components LedsC;
   appDATADevP.Leds -> LedsC;

   NeighborGet         = appDATADevP.NeighborGet;

   components PacketFunctionsC;
   appDATADevP.PacketFunctions->PacketFunctionsC;

   IDManager =appDATADevP.IDManager;
   components SerialActiveMessageC as AMSerial;

   appDATADevP.SerialControl -> AMSerial;
   appDATADevP.UartSend -> AMSerial;
   appDATADevP.UartReceive -> AMSerial.Receive[AM_TOKENTRANSFERREDMSG];
   appDATADevP.UartAMPacket -> AMSerial;
   appDATADevP.Packet-> AMSerial;

	// TSCH Critical Time interfaces
   CriticalTimeC = appDATADevP.CriticalTime;
   appDATADevP.ControlADV = ControlADVC;

   components new SensirionSht11C();
   appDATADevP.ReadTemp -> SensirionSht11C.Temperature;

	components new TimerMilliC() as timerCriticalTimeC;
	appDATADevP.timerCriticalTime->timerCriticalTimeC;

}
