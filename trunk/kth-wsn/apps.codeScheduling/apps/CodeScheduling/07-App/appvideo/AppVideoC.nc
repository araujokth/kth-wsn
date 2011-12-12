#include "LATIN.h"
#include "UserButton.h"

configuration %s/AppVideo/AppVideo/gC {
   //admin
   provides interface Init as SoftwareInit;
   uses interface Boot;
   uses interface SplitControl;
   //data flow
   uses interface UDPSend;
   provides interface AppReceive;
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger;
}
implementation {
   components AppVideoP;
   SoftwareInit = AppVideoP.SoftwareInit;
   IDManager = AppVideoP.IDManager;
   Boot = AppVideoP.Boot;
   SplitControl = AppVideoP.SplitControl;
   UDPSend = AppVideoP.UDPSend;
   AppReceive = AppVideoP.AppReceive;
   Malloc = AppVideoP.Malloc;
   OpenSerial = AppVideoP.OpenSerial;
   Trigger = AppVideoP.Trigger;

   //button
   components UserButtonC;
   AppVideoP.Notify->UserButtonC;

   components HplMsp430GeneralIOC;
   AppVideoP.Port34 -> HplMsp430GeneralIOC.Port34;

   components new TimerMilliC() as TimerMeasureC;
   AppVideoP.TimerMeasure->TimerMeasureC;
}
