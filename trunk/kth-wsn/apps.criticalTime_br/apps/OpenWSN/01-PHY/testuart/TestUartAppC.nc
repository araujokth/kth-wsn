configuration TestUartAppC {
}
implementation {
   components MainC;

   components TestUartC;
   TestUartC.Boot->MainC;
   
   components HplMsp430Usart1C;
   TestUartC.HplMsp430UsartInterrupts->HplMsp430Usart1C;
   TestUartC.AsyncStdControl->HplMsp430Usart1C;
   TestUartC.HplMsp430Usart->HplMsp430Usart1C;

   components new TimerMilliC() as TimerC;
   TestUartC.Timer -> TimerC;

   components HplMsp430GeneralIOC;
   TestUartC.Port66 -> HplMsp430GeneralIOC.Port66;
}
