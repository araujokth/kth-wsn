module RF231driverP @safe() {
   provides interface RadioControl;
   provides interface RadioSend;
   provides interface RadioReceive;
   uses interface BusyWait<TMicro, uint16_t> as radioWait;
   uses interface Leds;
   uses interface Resource as SpiResource;
   uses interface ChipSpiResource;
   //interrupt stuff
   uses interface GeneralIO as IRQRF;
   uses interface GeneralIO as IRQRFP;
   uses interface GpioInterrupt as InterruptIRQRFP;//
}



implementation {
   /*-------------------------------- variables -----------------------------------------*/

   uint8_t radioState; //this is the actual radio state
   uint8_t driverState; //used because the interrupt is the same for tx and rx
   error_t sendErr=SUCCESS;
   uint8_t frequencyChannel; //do i need it?
   uint8_t globalCommand;
   uint8_t globalNewReg;
   uint8_t globalFrequency;
   uint8_t returnedRegValue;
   uint8_t internalSpiState;
   uint8_t statusRg;
   uint8_t myState;
   bool statusRead;
   bool preparingPacket;
   //OpenQueueEntry_t* globalMsg;
   uint8_t *globalMsg;
   uint8_t *globalReceivedMsg;
   bool isPacketRead;


enum{
   CHANGINGREG = 0,
   READINGREG = 1,
   CHANGINGFREQ = 2,
   LOADINGPACKET = 3,
   READINGPACKET = 4,
   CHANGINGSTATE = 5,
   READINGSTATUS = 6,
   DEFAULT = 7,
};

enum{//not needed?
   MISC = 0,
   STARTING = 1,
   RECEIVING = 2,
   TRANSMITTING = 3,
};

   /*------------------------------prototypes------------------*/

task void taskStartDone();
task void taskStopDone();
void shutdown();
void ISRserviced();

   /*------------------------------SPI------------------*/

event void SpiResource.granted(){

   uint8_t tempSpiState;
   atomic tempSpiState = internalSpiState; //to avoid many atomics


   atomic AT_SPI_ON;
   if (tempSpiState == CHANGINGSTATE){
   atomic call ChipSpiResource.changeState(globalNewReg);
   call SpiResource.release();
   atomic internalSpiState = DEFAULT;
   }

   else if (tempSpiState == CHANGINGREG){
   atomic call ChipSpiResource.setRegister(globalCommand, globalNewReg);
   call SpiResource.release();
   atomic internalSpiState = DEFAULT;
   }

   else if (tempSpiState == READINGSTATUS){
   atomic statusRg = call ChipSpiResource.readRegister(RG_IRQ_STATUS);
   call SpiResource.release();
   atomic statusRead = TRUE;
   atomic internalSpiState = DEFAULT;
   ISRserviced();
   }

   else if (tempSpiState == CHANGINGFREQ){
   atomic call ChipSpiResource.setChannel(globalFrequency);
   call SpiResource.release();
   atomic internalSpiState = DEFAULT;
   }

   else if (tempSpiState == LOADINGPACKET){
   atomic call ChipSpiResource.loadPacket((uint8_t*) &globalMsg[2], globalMsg[0]);
   call SpiResource.release();
   atomic internalSpiState = DEFAULT;
   }

   else if (tempSpiState == READINGPACKET){
   call ChipSpiResource.readPacket(globalReceivedMsg);
   call SpiResource.release();
   atomic internalSpiState = DEFAULT;//atomic isPacketRead = TRUE;
   }

   else if (tempSpiState == READINGREG){
//   call Leds.led1Toggle();
   atomic returnedRegValue = call ChipSpiResource.readRegister(globalCommand);
   //if (returnedRegValue == 0x14) call Leds.led3Toggle();
   call SpiResource.release();
   
   atomic internalSpiState = DEFAULT;
   }
   atomic AT_SPI_OFF;
}

async event void ChipSpiResource.releasing() {} //just because :D


/*-------------------------------- startup sequence ----------------------------------*/

   async command error_t RadioControl.start() {
            atomic radioState = P_ON;
            atomic internalSpiState = DEFAULT; //state mgmt
            atomic driverState = STARTING;

            atomic AT_SPI_OFF;
            atomic AT_SLP_TR_CLR;
            atomic AT_SPI_PDIR |= AT_SPI_PIN;                     // /SEL output
            atomic AT_SLP_TR_PDIR |= AT_SLP_TR_PIN;               // SLP_TR output            atomic AT_SLP_TR_CLR;
            atomic AT_WAKE; //equivalent

//I'm trying to write junk first to see if it makes any difference: (this is a hack, do not remove)
atomic{
               if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.setRegister(0x1C, 0x03);
                  call SpiResource.release();
                  atomic AT_SPI_OFF;
               }
               else{
               atomic internalSpiState = CHANGINGREG;
               atomic globalCommand = 0x1C;
               atomic globalNewReg = 0x03;
               call SpiResource.request();
               }
            }

//now set channel (I do this here because it might be required for pll settling). the selected channel is random
atomic preparingPacket = FALSE;
atomic{
               if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.setChannel(0x0D);
                  call SpiResource.release();
                  atomic AT_SPI_OFF;
               }
               else{
               atomic internalSpiState = CHANGINGFREQ;
               atomic globalFrequency = 0x0D;
               call SpiResource.request();
               }
            }

/*check if state change is correct
atomic{
if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON; call Leds.led2Toggle();
                  myState = call ChipSpiResource.readRegister(0x08);
                  call SpiResource.release();
                  atomic AT_SPI_OFF; 
                  if (myState & 0x40) call Leds.led3Toggle();//4, 10
                  if (myState & 0x80) call Leds.led0Toggle();//20,
               }
else {
atomic internalSpiState = READINGREG;
atomic globalCommand = 0x08;
call SpiResource.request();
}      }    
//check ends here*/

//start with setting radio in trx_off state:
atomic{
            if (radioState != TRX_OFF){
               if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.changeState(CMD_TRX_OFF);
                  call SpiResource.release();
                  atomic AT_SPI_OFF;
               }
               else{
                  atomic internalSpiState = CHANGINGSTATE;
                  atomic globalNewReg = CMD_TRX_OFF;
                  call SpiResource.request();
               }
             
             call radioWait.wait(500);//was 400//i don't think i really need this. i might have understood this wrong
             atomic radioState = TRX_OFF;
            }}

//auto_crc is already on. Here would be a good place to setup trx_ctrl_1 (spi command mode for example)

            //MASKS: PLL_LOCK, RX_START, TRX_END (this one is SEND AND RECEIVE!!) = 0x0D
            
            if (call SpiResource.immediateRequest()){
               atomic AT_SPI_ON;
               call ChipSpiResource.setRegister (RG_IRQ_MASK, 0x0D);
               call SpiResource.release();
               atomic AT_SPI_OFF;
            }

            else{
               atomic internalSpiState = CHANGINGREG;
               atomic globalCommand = RG_IRQ_MASK;
               atomic globalNewReg = 0x0D;
               call SpiResource.request();
            }
            
            atomic AT_SPI_OFF;//my way of doing a nop//check out the SPI timings in the datasheet for details

            call InterruptIRQRFP.enableRisingEdge();//enable interrupts on the MSP port 1.6

            atomic{
            if (radioState != PLL_ON){
               if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.changeState(CMD_PLL_ON);
                  call SpiResource.release();
                  atomic AT_SPI_OFF; 
               }
               else{
                  atomic internalSpiState = CHANGINGSTATE;
                  atomic globalNewReg = CMD_PLL_ON;
                  call SpiResource.request();
               }
               //interrupt will occur when pll settles and ISR is called
            }}
            //can't move directly from P_ON to PLL_ON; has to go through TRX_OFF//datasheet p33
            //call radioWait.wait(110);


           //post taskStartDone();
            return SUCCESS;
   }

   task void taskStartDone() {
      signal RadioControl.startDone(SUCCESS);
   }

/*-------------------------------- send sequence -------------------------------------*/

   /////////async command error_t RadioSend.prepareSend( OpenQueueEntry_t* msg) {
   async command error_t RadioSend.prepareSend( uint8_t* msg) {
      ///////////frequencyChannel = msg->l1_channel;
      uint8_t count;
      atomic driverState = TRANSMITTING;
      atomic count = 0;
      atomic frequencyChannel = msg[1];



//if(!(call SpiResource.immediateRequest())){}//hack

      if (call SpiResource.immediateRequest()){
         atomic AT_SPI_ON;
         //////////call ChipSpiResource.loadPacket((uint8_t*) msg->payload, msg->length);
         call ChipSpiResource.loadPacket((uint8_t*) &msg[2], msg[0]);
         call SpiResource.release();
         atomic AT_SPI_OFF;
      }

      else{
         atomic internalSpiState = LOADINGPACKET;
         atomic globalMsg = msg;
         call SpiResource.request();
      }

      return SUCCESS;
   }

   async event void ChipSpiResource.loadPacketDone(error_t error) {

      atomic preparingPacket = TRUE;
atomic{
if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.changeState(CMD_NOP);
                  call SpiResource.release();
                  atomic AT_SPI_OFF; 
               }
               else{
                  atomic internalSpiState = CHANGINGSTATE;
                  atomic globalNewReg = CMD_NOP;
                  call SpiResource.request();
               }}
 /*           if (call SpiResource.immediateRequest()){
               atomic AT_SPI_ON;
               call ChipSpiResource.setRegister (RG_IRQ_MASK, 0x0C);
               call SpiResource.release();
               atomic AT_SPI_OFF;
            }

            else{
               atomic internalSpiState = CHANGINGREG;
               atomic globalCommand = RG_IRQ_MASK;
               atomic globalNewReg = 0x0C;
               call SpiResource.request();
            }*/


atomic{
               if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.setChannel(frequencyChannel);
                  call SpiResource.release();
                  atomic AT_SPI_OFF;
               }
               else{
               atomic internalSpiState = CHANGINGFREQ;
               atomic globalFrequency = frequencyChannel;
               call SpiResource.request();
               }
            }
   }

   async event void ChipSpiResource.doneSettingChannel(){
      if (driverState == TRANSMITTING) {signal RadioSend.prepareSendDone(SUCCESS);}
      else if (driverState == RECEIVING) { signal RadioControl.prepareReceiveDone(SUCCESS);}
      else if (driverState == STARTING) return;
   }
      

   async command error_t RadioSend.sendNow() {
      atomic AT_SLP_TR_SET; //just toggle that pin and wait for the interrupt
      //call radioWait.wait(100);
      atomic AT_SLP_TR_CLR;
      //interrupt will post taskSendDonecall Leds.led0Toggle();
/*atomic{
               if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.changeState(CMD_TX_START);
                  call SpiResource.release();
                  atomic AT_SPI_OFF;
               }
               else{
                  atomic internalSpiState = CHANGINGSTATE;
                  atomic globalNewReg = CMD_TX_START;
                  call SpiResource.request();
               }}*/
      return SUCCESS;
   }
  
   task void taskSendDone() {
     //I don't know what this is about!
     /* if (temp_state==S_STOPPING) {
         shutdown();
      } else {
         atomic state=S_STARTED;
      }*/
//atomic TOGGLE_P11_OFF;
//atomic TOGGLE_P11_ON;
      signal RadioSend.sendNowDone(SUCCESS);

   }

/*-------------------------------- receive sequence ---------------------------*/

async command error_t RadioControl.prepareReceive(uint8_t someChannel){
atomic driverState = RECEIVING;

atomic{
      if (call SpiResource.immediateRequest()){//set the frequency
         atomic AT_SPI_ON;
         call ChipSpiResource.setChannel(someChannel);
         call SpiResource.release();
         atomic AT_SPI_OFF;
      }

      else{
         atomic internalSpiState = CHANGINGFREQ;
         globalFrequency = someChannel;
         call SpiResource.request();
      }}
return SUCCESS;
}


async command error_t RadioControl.receiveNow(){
//go to state RX_ON
            atomic{
            if (radioState != RX_ON){
               if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.changeState(CMD_RX_ON);
                  call SpiResource.release();
                  atomic AT_SPI_OFF; 
               }
               else{
                  atomic internalSpiState = CHANGINGSTATE;
                  atomic globalNewReg = CMD_RX_ON;
                  call SpiResource.request();
               }}}
               atomic radioState = RX_ON;

               return SUCCESS;
//if something is received, interrupt will occur
//should i start a timer and if it fires call receivedNothing?

}

void receivedSomething(){

call InterruptIRQRFP.enableRisingEdge();
//here: malloc and timestamp.

}

void unloadPacket(){
     //atomic isPacketRead = FALSE;
     atomic {if (isPacketRead) return;}
      if (call SpiResource.immediateRequest()){
         atomic AT_SPI_ON;
         //////////call ChipSpiResource.loadPacket((uint8_t*) msg->payload, msg->length);
         call ChipSpiResource.readPacket(globalReceivedMsg); //i was here but I need globalreceivedpacket var
         call SpiResource.release();
         atomic AT_SPI_OFF; //atomic isPacketRead = TRUE;
         
      }

      else{
         atomic internalSpiState = READINGPACKET;
         call SpiResource.request();
      }

signal RadioReceive.receive(globalReceivedMsg);


}

/*-------------------------------- radio shutdown sequence ---------------------------*/

   command error_t RadioControl.stop() {
      uint8_t temp_state;
      atomic temp_state=radioState;
      switch (temp_state) {
         case TRX_OFF:
            shutdown();
            return SUCCESS;
         case SLEEP:
            return SUCCESS;
         default:
            atomic{
            if (radioState != TRX_OFF){
               if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.changeState(CMD_TRX_OFF);
                  call SpiResource.release();
                  atomic AT_SPI_OFF;
               }
               else{
                  atomic internalSpiState = CHANGINGSTATE;
                  atomic globalNewReg = CMD_TRX_OFF;
                  call SpiResource.request();
               }
             call radioWait.wait(400);
             atomic radioState = TRX_OFF;
            }}
            call radioWait.wait(1);
            atomic radioState = TRX_OFF;
            } //goto state TRXOFF
            shutdown();
            return SUCCESS;
      }


   void shutdown() {
      //set SLP_TR = L => radio will go to sleep mode
      atomic AT_SLEEP; //dont forget to AT_WAKE
      call radioWait.wait(110);
      atomic radioState = SLEEP;
      post taskStopDone();
   }

   task void taskStopDone(){
        signal RadioControl.stopDone( SUCCESS );
   }
   

//async event void SpiPacket.sendDone( uint8_t* tx_buf, uint8_t* rx_buf, uint16_t len, error_t error ) {}
//async event void ChipSpiResource.releasing() {}


async command error_t RadioControl.rfOff(){}

/*-------------------------------- interrupt handling routines ---------------------------*/
async event void InterruptIRQRFP.fired() {
   //call Leds.led0Toggle();
   call InterruptIRQRFP.disable(); 
   atomic statusRead = FALSE;
//start by reading status rg
   if (call SpiResource.immediateRequest()){
      atomic AT_SPI_ON;
      atomic statusRg = call ChipSpiResource.readRegister(RG_IRQ_STATUS);
      call SpiResource.release();
      atomic AT_SPI_OFF;
      atomic statusRead = TRUE;
   }

   else{
      atomic internalSpiState = READINGSTATUS;
      call SpiResource.request();
    }
    if (statusRead == TRUE)
       ISRserviced();//else spi granted will call function below

}

void ISRserviced(){
   uint8_t tempstatusRg;
   atomic tempstatusRg = statusRg;


/*atomic{
if (driverState == TRANSMITTING){
 call Leds.led0Toggle();//0,2,3 off
 if (tempstatusRg == 0x8F)  call Leds.led2Toggle();//1 and 2 on
 if (tempstatusRg & 0x10)  call Leds.led3Toggle();//3 on
}}*/


   if (tempstatusRg == 0x01){//this means that pll_locked (removes need for busywaits)

/*This means that the radio has started and we don't need the PLL_LOCK interrupt anymore
//the reason is that even if we change to states tx or rx, the radio is going to wait until the pll settles and then
//it will start sending/receiving.
//this is why we change the masks to only rx_start and trx_end:
            if (call SpiResource.immediateRequest()){
               atomic AT_SPI_ON;
               call ChipSpiResource.setRegister (RG_IRQ_MASK, 0x0C);
               call SpiResource.release();
               atomic AT_SPI_OFF;
            }

            else{
               atomic internalSpiState = CHANGINGREG;
               atomic globalCommand = RG_IRQ_MASK;
               atomic globalNewReg = 0x0C;
               call SpiResource.request();
            }
            
            atomic AT_SPI_OFF;//my way of doing a nop//check out the SPI timings in the datasheet for details
/////////////*/

atomic{
if (call SpiResource.immediateRequest()){
                  atomic AT_SPI_ON;
                  call ChipSpiResource.changeState(CMD_NOP);
                  call SpiResource.release();
                  atomic AT_SPI_OFF; 
               }
               else{
                  atomic internalSpiState = CHANGINGSTATE;
                  atomic globalNewReg = CMD_NOP;
                  call SpiResource.request();
               }}
         atomic radioState = PLL_ON;
         post taskStartDone();
         }

    else if (tempstatusRg == 0x04){//means that a frame is being received
         receivedSomething();
         }

    else if (tempstatusRg == 0x8F || tempstatusRg == 0x08){//means that the frame has entirely left the buffer or that the frame has entirely arrived to the buffer // why 8f? i don't know. it's a hack
         
         atomic{
         if (driverState == TRANSMITTING) post taskSendDone();
         else if (driverState == RECEIVING) unloadPacket();
         }}
   call InterruptIRQRFP.enableRisingEdge();
}

}
