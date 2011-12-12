module TestUartC {
   uses interface Boot;
   uses interface HplMsp430UsartInterrupts;
   uses interface AsyncStdControl;
   uses interface HplMsp430Usart;
   uses interface Timer<TMilli> as Timer;
   uses interface HplMsp430GeneralIO as Port66;
}
implementation {

   msp430_uart_union_config_t msp430_uart_telos_config = {{ubr: UBR_1MHZ_115200, umctl: UMCTL_1MHZ_115200, ssel: 0x02, pena: 0, pev: 0, spb: 0, clen: 1, listen: 0, mm: 0, ckpl: 0, urxse: 0, urxeie: 1, urxwie: 0, utxe : 1, urxe : 1}};
   uint8_t counter;
   uint8_t frame[100];
   bool active;

   //Boot
   event void Boot.booted() {
      call HplMsp430Usart.setModeUart(&msp430_uart_telos_config);
      call HplMsp430Usart.enableIntr();
      call Timer.startPeriodic(1024);
   }

   //Timer
   event void Timer.fired() {
      call HplMsp430Usart.tx('y');
      call Port66.toggle();
      counter=0;
      active=FALSE;
   }

   //HplMsp430UsartInterrupts
   async event void HplMsp430UsartInterrupts.rxDone(uint8_t data) {
      frame[counter]=data;
      counter++;
      if (counter==99) {
         call Port66.toggle();
         call HplMsp430Usart.disableRxIntr();
         counter=0;
         active=TRUE;
         call HplMsp430Usart.tx(frame[counter]);
      }
   }
   async event void HplMsp430UsartInterrupts.txDone() {
      if (active) {
         call HplMsp430Usart.tx(frame[counter]);
         counter++;
         if (counter==99) {
            call Port66.toggle();
            call HplMsp430Usart.enableIntr();
            counter=0;
            active=FALSE;
         }
      }
   }
}
