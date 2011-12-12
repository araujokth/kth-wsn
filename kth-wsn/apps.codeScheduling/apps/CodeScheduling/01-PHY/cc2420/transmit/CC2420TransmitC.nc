#include "IEEE802154.h"

configuration CC2420TransmitC {
   provides interface StdAsyncControl as TxControl;
   provides interface CC2420Transmit;
//   uses interface OpenSerial;
   uses interface GlobalTime;
   uses interface CC2420sfd;
   uses interface GeneralIO as CCA;
   uses interface GeneralIO as CSN;
   uses interface GeneralIO as SFD;
   uses interface GpioCapture as CaptureSFD;
}

implementation {

   components CC2420TransmitP;
   TxControl       = CC2420TransmitP;
   CC2420Transmit  = CC2420TransmitP;
   GlobalTime      = CC2420TransmitP;
   CC2420sfd       = CC2420TransmitP;
   CCA             = CC2420TransmitP.CCA;
   CSN             = CC2420TransmitP.CSN;
   SFD             = CC2420TransmitP.SFD;
   CaptureSFD      = CC2420TransmitP;
//   OpenSerial      = CC2420TransmitP;

   components MainC;
   MainC.SoftwareInit -> CC2420TransmitP;

   components new CC2420SpiC();
   CC2420TransmitP.SpiResource     -> CC2420SpiC;
   CC2420TransmitP.ChipSpiResource -> CC2420SpiC;
   CC2420TransmitP.SNOP            -> CC2420SpiC.SNOP;
   CC2420TransmitP.STXON           -> CC2420SpiC.STXON;
   CC2420TransmitP.SFLUSHTX        -> CC2420SpiC.SFLUSHTX;
   CC2420TransmitP.TXCTRL          -> CC2420SpiC.TXCTRL;
   CC2420TransmitP.TXFIFO          -> CC2420SpiC.TXFIFO;
   CC2420TransmitP.TXFIFO_RAM      -> CC2420SpiC.TXFIFO_RAM;
   CC2420TransmitP.MDMCTRL1        -> CC2420SpiC.MDMCTRL1;

   components new Alarm32khz32C() as AlarmWatchdogC;
   CC2420TransmitP.AlarmWatchdog -> AlarmWatchdogC;

	// D E B U G
#ifdef TSCHDEBUG_ENABLED
	components PinDebugC as PinDebug;
#endif
	CC2420TransmitP.PinDebug -> PinDebug;
}
