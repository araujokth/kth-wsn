#include "CC2420.h"
#include "IEEE802154.h"

configuration CC2420ControlC {
	provides interface CC2420Config;
	//   uses interface OpenSerial;
	uses interface GeneralIO as CSN;
	uses interface GeneralIO as RSTN;
	uses interface GeneralIO as VREN;
	uses interface GpioInterrupt as InterruptCCA;
	uses interface Alarm<T32khz,uint32_t> as StartupTimer;
}
implementation {
	components CC2420ControlP;
	//   OpenSerial   = CC2420ControlP;
	CC2420Config = CC2420ControlP;
	CSN = CC2420ControlP.CSN;
	RSTN = CC2420ControlP.RSTN;
	VREN = CC2420ControlP.VREN;
	InterruptCCA = CC2420ControlP;
	StartupTimer = CC2420ControlP;

	components MainC;
	MainC.SoftwareInit -> CC2420ControlP;

	components new CC2420SpiC();
	CC2420ControlP.SRXON -> CC2420SpiC.SRXON;
	CC2420ControlP.SRFOFF -> CC2420SpiC.SRFOFF;
	CC2420ControlP.SXOSCON -> CC2420SpiC.SXOSCON;
	CC2420ControlP.SXOSCOFF -> CC2420SpiC.SXOSCOFF;
	CC2420ControlP.FSCTRL -> CC2420SpiC.FSCTRL;
	CC2420ControlP.IOCFG0 -> CC2420SpiC.IOCFG0;
	CC2420ControlP.IOCFG1 -> CC2420SpiC.IOCFG1;
	CC2420ControlP.MDMCTRL0 -> CC2420SpiC.MDMCTRL0;
	CC2420ControlP.MDMCTRL1 -> CC2420SpiC.MDMCTRL1;
	CC2420ControlP.PANID -> CC2420SpiC.PANID;
	CC2420ControlP.SHORTADR -> CC2420SpiC.SHORTADR;
	CC2420ControlP.IEEEADR -> CC2420SpiC.IEEEADR;
	CC2420ControlP.RXCTRL1 -> CC2420SpiC.RXCTRL1;
	CC2420ControlP.RSSI -> CC2420SpiC.RSSI;

	components new CC2420SpiC() as OscSpiC;
	CC2420ControlP.OscResource -> OscSpiC;

	components new CC2420SpiC() as Spi;
	CC2420ControlP.SpiResource -> Spi;

	components new CC2420SpiC() as SyncSpiC;
	CC2420ControlP.SyncResource -> SyncSpiC;

	components new CC2420SpiC() as RxOnSpiC;
	CC2420ControlP.RxOnResource -> RxOnSpiC;

	components new CC2420SpiC() as RfOffSpiC;
	CC2420ControlP.RfOffResource -> RfOffSpiC;

	components new Alarm32khz32C() as StartupAlarmC;
	CC2420ControlP.StartupAlarm -> StartupAlarmC;

	// D E B U G
#ifdef TSCHDEBUG_ENABLED
	components PinDebugC as PinDebug;
#endif
	CC2420ControlP.PinDebug -> PinDebug;
}
