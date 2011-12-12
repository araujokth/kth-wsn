#include "CC2420.h"
#include "IEEE802154.h"

configuration CC2420DriversC {
   provides interface RadioControl;
   provides interface RadioSend;
   provides interface RadioReceive;
   uses interface Malloc;
//   uses interface OpenSerial;
   uses interface GlobalTime;
}

implementation {

   components CC2420DriversP;
   RadioControl    = CC2420DriversP;
   RadioSend       = CC2420DriversP;
//   OpenSerial      = CC2420DriversP;

   components CC2420ControlC;
//   OpenSerial      = CC2420ControlC;
   CC2420DriversP.CC2420Config           -> CC2420ControlC;

   components CC2420ReceiveC;
   RadioReceive    = CC2420ReceiveC;
   Malloc          = CC2420ReceiveC;
//   OpenSerial      = CC2420ReceiveC;
   CC2420DriversP.RxControl              -> CC2420ReceiveC;
   CC2420DriversP.CC2420ReceivedNothing  -> CC2420ReceiveC;
   CC2420TransmitC.CC2420sfd            -> CC2420ReceiveC;
   CC2420TransmitC.CC2420sfd            -> CC2420DriversP;

   components CC2420TransmitC;
   GlobalTime      = CC2420TransmitC;
//   OpenSerial      = CC2420TransmitC;
   CC2420DriversP.TxControl              -> CC2420TransmitC;
   CC2420DriversP.CC2420Transmit         -> CC2420TransmitC;

   components HplCC2420InterruptsC;
   CC2420TransmitC.CaptureSFD           -> HplCC2420InterruptsC.CaptureSFD;
   CC2420ReceiveC.InterruptFIFOP        -> HplCC2420InterruptsC.InterruptFIFOP;
   CC2420ControlC.InterruptCCA          -> HplCC2420InterruptsC.InterruptCCA;

   components HplCC2420PinsC;
   CC2420TransmitC.CCA                  -> HplCC2420PinsC.CCA;
   CC2420TransmitC.CSN                  -> HplCC2420PinsC.CSN;
   CC2420TransmitC.SFD                  -> HplCC2420PinsC.SFD;
   CC2420ReceiveC.CSN                   -> HplCC2420PinsC.CSN;
   CC2420ReceiveC.FIFO                  -> HplCC2420PinsC.FIFO;
   CC2420ReceiveC.FIFOP                 -> HplCC2420PinsC.FIFOP;
   CC2420ControlC.CSN                   -> HplCC2420PinsC.CSN;
   CC2420ControlC.RSTN                  -> HplCC2420PinsC.RSTN;
   CC2420ControlC.VREN                  -> HplCC2420PinsC.VREN;

   components new Alarm32khz32C() as RxWaitAlarmC;
   CC2420DriversP.RxWaitAlarm            -> RxWaitAlarmC;

   components HplMsp430GeneralIOC;
   CC2420DriversP.Port34                 -> HplMsp430GeneralIOC.Port34;
	// D E B U G
#ifdef TSCHDEBUG_ENABLED
	components PinDebugC as PinDebug;
#endif
	CC2420DriversP.PinDebug -> PinDebug;
}
