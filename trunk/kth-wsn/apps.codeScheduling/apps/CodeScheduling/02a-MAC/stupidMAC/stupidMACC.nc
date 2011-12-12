configuration stupidMACC {
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
   //debug
   provides interface DebugPrint;
   uses interface OpenSerial;
   uses interface CellStats;
   //misc
   uses interface NeighborStats;
   uses interface Malloc;
   uses interface CellUsageGet;
   uses interface IDManager;
   uses interface NeighborGet;
}

implementation {
   components stupidMACP;
   Boot               = stupidMACP.Boot;
   GlobalTime         = stupidMACP.GlobalTime;
   GlobalSync         = stupidMACP.GlobalSync;
   OpenSendFromUpper  = stupidMACP.OpenSendFromUpper;
   OpenQueue          = stupidMACP.OpenQueue;
   RadioControl       = stupidMACP.RadioControl;
   RadioSend          = stupidMACP.RadioSend;
   RadioReceive       = stupidMACP.RadioReceive;
   OpenReceiveToUpper = stupidMACP.OpenReceiveToUpper;
   DebugPrint         = stupidMACP.DebugPrint;
   OpenSerial         = stupidMACP.OpenSerial;
   CellStats          = stupidMACP.CellStats;
   NeighborStats      = stupidMACP.NeighborStats;
   Malloc             = stupidMACP.Malloc;
   CellUsageGet       = stupidMACP.CellUsageGet;
   IDManager          = stupidMACP.IDManager;
   NeighborGet        = stupidMACP.NeighborGet;

   components MainC;
   MainC.SoftwareInit->stupidMACP;

   components new TimerMilliC() as TimerDebugC;
   stupidMACP.timerDebug->TimerDebugC;

   components new TimerMilliC() as TimerBackoffC;
   stupidMACP.timerBackoff->TimerBackoffC;

   components new Alarm32khz32C() as AlarmWatchdogC;
   stupidMACP.AlarmWatchdog->AlarmWatchdogC;

   components LedsC;
   stupidMACP.Leds -> LedsC;

   components HplMsp430GeneralIOC;
   stupidMACP.Port26 -> HplMsp430GeneralIOC.Port26;
   stupidMACP.Port35 -> HplMsp430GeneralIOC.Port35;
   stupidMACP.Port67 -> HplMsp430GeneralIOC.Port67;
   stupidMACP.Port34 -> HplMsp430GeneralIOC.Port34;

   components PacketFunctionsC;
   stupidMACP.PacketFunctions -> PacketFunctionsC;

   components RandomC;
   stupidMACP.Random->RandomC;
}
