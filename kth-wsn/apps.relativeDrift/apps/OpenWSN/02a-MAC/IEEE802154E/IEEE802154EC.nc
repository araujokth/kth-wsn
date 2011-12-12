#include "printf.h"
configuration IEEE802154EC {
	//admin
	uses interface Boot;
	//time
	provides interface GlobalTime;
	provides interface GlobalSync;
	provides interface ControlSync;
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
	uses interface CellUsageGet;
	uses interface IDManager;
	uses interface NeighborGet;
	uses interface CellStats;

	// TSCH Critical Time interfaces
	uses interface CriticalTime as CriticalTimeC;
}

implementation {
	components IEEE802154EP;
	Boot = IEEE802154EP.Boot;
	GlobalTime = IEEE802154EP.GlobalTime;
	GlobalSync = IEEE802154EP.GlobalSync;
	OpenSendFromUpper = IEEE802154EP.OpenSendFromUpper;
	OpenQueue = IEEE802154EP.OpenQueue;
	RadioControl = IEEE802154EP.RadioControl;
	RadioSend = IEEE802154EP.RadioSend;
	RadioReceive = IEEE802154EP.RadioReceive;
	OpenReceiveToUpper = IEEE802154EP.OpenReceiveToUpper;
	CellStats = IEEE802154EP.CellStats;
	NeighborStats = IEEE802154EP.NeighborStats;
	Malloc = IEEE802154EP.Malloc;
	CellUsageGet = IEEE802154EP.CellUsageGet;
	IDManager = IEEE802154EP.IDManager;
	NeighborGet = IEEE802154EP.NeighborGet;
	ControlSync = IEEE802154EP.ControlSync;

	components MainC;
	MainC.SoftwareInit->IEEE802154EP;

	components new Alarm32khz32C() as SlotAlarmC;
	IEEE802154EP.SlotAlarm -> SlotAlarmC;

	components new Alarm32khz32C() as FastAlarmC;
	IEEE802154EP.FastAlarm -> FastAlarmC;

	components new TimerMilliC() as LosingSyncTimerC;
	IEEE802154EP.LosingSyncTimer -> LosingSyncTimerC;

	components new TimerMilliC() as LostSyncTimerC;
	IEEE802154EP.LostSyncTimer -> LostSyncTimerC;

	components LedsC;
	IEEE802154EP.Leds -> LedsC;

	// D E B U G
#ifdef TSCHDEBUG_ENABLED
	components PinDebugC as PinDebug;
#endif
	IEEE802154EP.PinDebug -> PinDebug;

	components RandomC;
	IEEE802154EP.Random->RandomC;

	components PacketFunctionsC;
	IEEE802154EP.PacketFunctions->PacketFunctionsC;

	// TSCH Critical Time interfaces
	IEEE802154EP.CriticalTime = CriticalTimeC;
	components new TimerMilliC() as CriticalTimeTimerC;
	IEEE802154EP.CriticalTimeTimer -> CriticalTimeTimerC;
}
