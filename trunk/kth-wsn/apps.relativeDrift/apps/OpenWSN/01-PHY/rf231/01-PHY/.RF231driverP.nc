#include "atmel.h"
#include "radio.h"

module RF231driverP @safe() {
   provides interface RadioControl;
   //provides interface RadioSend;
   uses interface BusyWait<TMicro, uint32_t> as radioWait;

   //uses interface Alarm<T32khz,uint32_t> as RxWaitAlarm;
   //uses interface StdAsyncControl as TxControl;
   //uses interface StdAsyncControl as RxControl;
   //uses interface HplMsp430GeneralIO as Port34;//what are my ports?
}

implementation {

   /*-------------------------------- variables -----------------------------------------*/

   uint8_t radioState = P_ON; //this is the actual radio state
   error_t sendErr=SUCCESS;
   uint8_t frequencyChannel; //do i need it?

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskStartDone();
   //task void taskStopDone();
   //task void taskSendDone();
   void shutdown();

   uint8_t at_send(uint8_t* txstr, uint8_t* rxstr, uint8_t len);
   void at_set_reg(uint8_t reg, uint8_t val);
   uint8_t at_get_reg(uint8_t reg);
   uint8_t at_set_chan(uint8_t chan);



   /*-------------------------------- startup sequence ----------------------------------*/

   async command error_t RadioControl.start() {

            AT_SPI_OFF;
            AT_SLP_TR_CLR;
            AT_SPI_PDIR |= AT_SPI_PIN;                     // /SEL output
            AT_SLP_TR_PDIR |= AT_SLP_TR_PIN;               // SLP_TR output
            P3SEL |= BIT0 | BIT4 | BIT5;                  // Enable UCA0 SPI
  
            P1IE |= AT_IRQ_PIN;
  
            UCA0CTL0 = UCCKPH + UCMSB + UCMST + UCSYNC;   // polarity, MSB first, 3-pin SPI
          //  UCA0BR0 = 0x03;                           // UCLK/4
          //  UCA0BR1 = 0x00;                           // 0

            UCA0CTL1 &= ~UCSWRST;                     // Initialize USART state machine

            IE2 |= UCA0RXIE;         // Enable USCI_A0 RX/TX interrupt


            AT_SLP_TR_CLR;
            if (radioState != TRX_OFF){
               at_set_reg(SR_TRX_CMD,CMD_TRX_OFF); //goto state TRXOFF
               radioWait.wait(400);
               radioState = TRX_OFF;
            }
            
            if (radioState != PLL_ON){
               at_set_reg(SR_TRX_CMD,CMD_PLL_ON);
               radioWait.wait(110);
	       radioState = PLL_ON; //can't move directly from P_ON to PLL_ON; has to go through TRX_OFF
            }

            signal RadioControl.startDone(SUCCESS);
         }
      return SUCCESS;

   /*-------------------------------- receive sequence ----------------------------------

   async command error_t RadioControl.prepareReceive(uint8_t channel){
      uint8_t temp_state;
      atomic temp_state=state;
      if (temp_state!=S_STARTED) {
         return FAIL;
      }
      atomic syncForSend=FALSE;
      atomic state = S_SETTINGCHANNEL;
      at_set_chan(channel);
      //do i need to call set channel done?
      return SUCCESS;
   }
   async event void RF231Config.setChannelDone(error_t error) {
      bool temp_syncForSend;
      atomic temp_syncForSend = syncForSend;
      if (temp_syncForSend) {
         atomic state=S_READYTX;
         signal RadioSend.prepareSendDone(error);
      } else {
         atomic state=S_READYRX;
         signal RadioControl.prepareReceiveDone(error);
      }
   }
   async command error_t RadioControl.receiveNow(){
      atomic state = S_RECEIVING;
      //return call RF231Config.rxOn();
      at_set_reg(SR_TRX_CMD,CMD_RX_ON);
      radioState = RX_ON;
      //wait 110us

      call RxWaitAlarm.start(TsRxWaitTime); //poipoi comment out for stupidMAC

   }
   async command void RF231sfd.sfd( uint32_t time ) {
      call RxWaitAlarm.stop();
   }
   async command void RF231sfd.sfd_dropped() {                      //from transmitP
      call RxWaitAlarm.stop();
      call TxControl.start();
      call RxControl.start();
      atomic state=S_STARTED;
      signal RadioControl.receivedNothing();
   }
   async event void RF231ReceivedNothing.receivedNothing() {        //from receiveP
      call RxWaitAlarm.stop();
      call TxControl.start();
      call RxControl.start();
      atomic state=S_STARTED;
      signal RadioControl.receivedNothing();
   }
   async event void RxWaitAlarm.fired() {
      call RxWaitAlarm.stop();
      call TxControl.start();
      call RxControl.start();
      atomic state=S_STARTED;
      signal RadioControl.receivedNothing();
   }
//TO READ DATA:
//for ( ; len; len-- ) {
//      *data++ = call SpiByte.write( 0 );
   /*-------------------------------- send sequence -------------------------------------

   async command error_t RadioSend.prepareSend( OpenQueueEntry_t* msg, uint8_t channel) {
      if (radioState!= PLL_ON){
         at_set_reg(SR_TRX_CMD,CMD_PLL_ON);
         radioWait.wait(110);
         radioState = PLL_ON;
       }

      //LATER: set transmission power: PHY_TX_PWR = 0b11000110

      

      uint8_t unused;
      uint8_t copyOfLen = msg[0];
      uint8_T *length;
      length = &copyOfLen; //those three lines are to conserve the value of msg[0] inside the struct
      while(at_send(msg, unused, length)); //this fills the buffer inside the radio
      signal radioControl.loadPacketDone(SUCCESS);

      return SUCCESS;
   }


   async event void radioControl.loadPacketDone(error_t error) {
      if (error==SUCCESS)
         at_set_chan(channel);
      signal R      radioWait.wait(110);
      radioState = SLEEP;adioSend.prepareSendDone(SUCCESS);
         
         //if (call RF231Config.setChannel(frequencyChannel)!=SUCCESS) {
         //   call SerialIO.printError(COMPONENT_RF231DRIVER,ERR_SETCHANNEL_FAILED,(errorparameter_t)0,(errorparameter_t)0);
         }
      } else {
         //atomic state = S_STARTED;
         signal RadioSend.prepareSendDone(FAIL);
      }
   }

   async command error_t RadioSend.sendNow() {
      bool useCca = FALSE; //remove
      AT_TX_START; //this triggers the send sequence
      AT_UNTIL_STATUS(SR_IRQ_3_TRX_END); //wait until you get the TX_END interrupt
      signal radioControl.sendNowDone(SUCCESS);


      return SUCCESS;
   }

   /*-------------------------------- switching off RF ----------------------------------

   async command error_t RadioControl.rfOff() {
      error_t temp_return = SUCCESS;

      //write state change to TRX_OFF
      at_set_reg(SR_TRX_CMD,CMD_TRX_OFF);
      radioWait.wait(1);
      radioState = TRX_OFF;

      // i don't know what's going on here
      //atomic state=S_STARTED;
      //call TxControl.start();
      //call RxControl.start();
      return temp_return;
   }

   /*-------------------------------- radio shutdown sequence ---------------------------

   command error_t RadioControl.stop() {
      uint8_t temp_state;
      atomic temp_state=state;
      switch (temp_state) {
         case S_STARTED:
            atomic state=S_STOPPING;
            shutdown();
            return SUCCESS;
         case S_STOPPED:
            return EALREADY;
         case S_TRANSMITTING:
            atomic state=S_STOPPING;
            // At sendDone, the radio will shut down
            return SUCCESS;
         case S_STOPPING:
            return SUCCESS;
      }
      return EBUSY;
   }*/
   void shutdown() {
      //call TxControl.stop();
      //call RxControl.stop();
      //call RF231Config.stopVReg();
      //set SLP_TR = L => radio will go to sleep mode
      if (radioState != TRX_OFF){
         at_set_reg(SR_TRX_CMD,CMD_TRX_OFF);
         radioWait.wait(1);
      }


      AT_SLEEP; //dont forget to AT_WAKE
      radioWait.wait(110);
      radioState = SLEEP;
      signal RadioControl.stopDone( SUCCESS );
   }
}

   /*-------------------------------- helper functions ---------------------------*/

   int at_send(uint8_t *txstr, uint8_t *rxstr, uint8_t len) {
      if (AT_BUSY) return 1; // still transmitting

      at_tx_byte_str = txstr+1;
      at_rx_byte_str = rxstr;
      at_bytes_left = len-1;
      AT_SPI_ON;
      UCA0TXBUF = *txstr;     // output first uint8_t
      return 0;
   }

   void at_set_reg(uint8_t reg, uint8_t val) {
      at_tx_buf[0] = AT_WRITE_REG(reg);
      while(at_send(at_tx_buf, at_rx_buf, 2));
      at_tx_buf[1] = val;
   }

   uint8_t at_get_reg(uint8_t reg) {
      at_tx_buf[0] = AT_READ_REG(reg);
      while(at_send(at_tx_buf, at_rx_buf, 2));
      at_tx_buf[1] = 0;
      AT_WAIT;
      return AT_REG_VAL;
   }


   uint8_t at_set_chan(uint8_t chan) {
      if (chan) {
         at_current_channel = chan;
         if (chan < 11) chan = 11;
         if (chan > 26) chan = 26;
         at_set_reg(RG_PHY_CC_CCA,0x20 + chan);         // Set channel
      }
      return chan;
   }
