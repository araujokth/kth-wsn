configuration HplRF231PinsC {

  provides interface GeneralIO as IRQRFP;
}

implementation {

  components HplMsp430GeneralIOC as GeneralIOC;
  components new Msp430GpioC() as IRQRFPM;
  

  IRQRFPM -> GeneralIOC.Port16;//27
  
  IRQRFP = IRQRFPM;
}

