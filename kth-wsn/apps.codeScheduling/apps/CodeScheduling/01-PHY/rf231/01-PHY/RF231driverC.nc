#include "atmel.h"
#include "radio.h"

configuration RF231driverC {
   provides interface RadioControl;
   provides interface RadioSend;
   provides interface RadioReceive;

   //interrupt stuff
   uses interface GeneralIO as IRQRF;
   uses interface GeneralIO as IRQRFP;
   uses interface GpioInterrupt as InterruptIRQRFP;//
}

implementation {
   components RF231driverP,LedsC;
   RadioControl = RF231driverP.RadioControl;
   RadioSend = RF231driverP.RadioSend;
   RadioReceive = RF231driverP.RadioReceive;

   components BusyWaitMicroC as radioWaitC;
   components new RF231SpiC();
   components HplRF231InterruptsC;


   //interrupt stuff
   IRQRF                   = RF231driverP.IRQRF;
   IRQRFP                  = RF231driverP.IRQRFP;
   InterruptIRQRFP         = RF231driverP;//

   //added this:
   RF231driverP.InterruptIRQRFP -> HplRF231InterruptsC;

   RF231driverP.SpiResource     -> RF231SpiC;
   RF231driverP.ChipSpiResource -> RF231SpiC;
   RF231driverP.radioWait -> radioWaitC;
   RF231driverP.Leds -> LedsC;


}
