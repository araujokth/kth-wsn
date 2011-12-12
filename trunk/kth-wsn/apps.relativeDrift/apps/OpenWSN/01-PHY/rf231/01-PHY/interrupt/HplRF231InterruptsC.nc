configuration HplRF231InterruptsC {

  provides interface GpioInterrupt as InterruptIRQRFP;

}

implementation {

  components HplMsp430GeneralIOC as GeneralIOC;

  components HplMsp430InterruptC;
  components new Msp430InterruptC() as InterruptIRQRFPC;
  InterruptIRQRFPC.HplInterrupt -> HplMsp430InterruptC.Port16;//27

  InterruptIRQRFP = InterruptIRQRFPC.Interrupt;

}
