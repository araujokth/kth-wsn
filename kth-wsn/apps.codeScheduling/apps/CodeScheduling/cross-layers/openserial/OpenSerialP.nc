enum {
   MODE_OFF,
   MODE_INPUT,
   MODE_OUTPUT,
};

module OpenSerialP {
   provides interface Init as SoftwareInit;
   provides interface OpenSerial;
   provides interface DebugPrint;
   uses interface IDManager;
   uses interface Trigger as TriggerIDManagerAboutBridge;
   uses interface Trigger as TriggerIDManagerAboutRoot;
   uses interface Trigger as TriggerTCPInject;
   uses interface Trigger as TriggerUDPInject;
   uses interface Trigger as TriggerUDPSensor;
   uses interface Trigger as TriggerICMPv6Echo;
   uses interface Trigger as TriggerICMPv6Router;
   uses interface Trigger as TriggerICMPv6RPL;
   uses interface Trigger as TriggerOpenBridge;//called only by OpenLBR
   uses interface HplMsp430UsartInterrupts;
   uses interface AsyncStdControl;
   uses interface HplMsp430Usart;
   uses interface HplMsp430GeneralIO as Port63;
   uses interface Leds;    //only for Led0=red
   uses interface Notify<button_state_t>;
   uses interface DebugPrint as PrintOpenSerial;
   uses interface DebugPrint as PrintCellUsage;
   uses interface DebugPrint as PrintNeighbors;
   uses interface DebugPrint as PrintOpenQueue;
   uses interface DebugPrint as PrintIEEE802154E;
   uses interface DebugPrint as PrintMyDAGrank;
   uses interface DebugPrint as PrintRPL;
   uses interface DebugPrint as PrintIDManager;
}
implementation {

   /*------------------- variables ----------------------------------*/

   msp430_uart_union_config_t msp430_uart_telos_config = {{ubr:UBR_1MHZ_115200, umctl:UMCTL_1MHZ_115200, ssel:0x02, pena:0, pev:0, spb:0, clen:1, listen:0, mm:0, ckpl:0, urxse:0, urxeie:1, urxwie:0, utxe:1, urxe:1}};

   uint8_t  output_buffer[SERIAL_OUTPUT_BUFFER_SIZE];
   uint16_t output_buffer_index_write;
   uint16_t output_buffer_index_read;
   bool     somethingInOutputBuffer;

   uint8_t  input_buffer[SERIAL_INPUT_BUFFER_SIZE];
   uint16_t input_buffer_fill_level;
   uint8_t  input_buffer_bytes_still_to_come;
   uint8_t  received_command;

   bool     ready_receive_command;
   bool     ready_receive_length;
   uint8_t  input_command[8];
   uint8_t  input_command_index;

   uint8_t  mode;

   uint8_t  debugPrintCounter;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskTriggerStatus();
   task void taskPrintDebug();
   task void taskHandleCommand();
   uint16_t  output_buffer_index_write_increment();
   uint16_t  output_buffer_index_read_increment();

   /*------------------- interfaces ---------------------------------*/

   //SoftwareInit
   command error_t SoftwareInit.init() {
      call HplMsp430Usart.setModeUart(&msp430_uart_telos_config);
      call HplMsp430Usart.enableIntr();
      call Notify.enable();
      atomic {
         input_command[0] = (uint8_t)'^';
         input_command[1] = (uint8_t)'^';
         input_command[2] = (uint8_t)'^';
         input_command[3] = (uint8_t)'R';
         input_command[4] = 0;//to be filled out later
         input_command[5] = (uint8_t)'$';
         input_command[6] = (uint8_t)'$';
         input_command[7] = (uint8_t)'$';
         output_buffer_index_read=0;
         output_buffer_index_write=0;
         somethingInOutputBuffer=FALSE;
         mode = MODE_OFF;
      }
      return SUCCESS;
   }

   //------ [with components] store data in the output buffer
   command error_t OpenSerial.printStatus(uint8_t statusElement,uint8_t* buffer, uint16_t length) {
      uint8_t counter;
      atomic {
         somethingInOutputBuffer=TRUE;
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';                  //preamble
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'S';                  //this is an status update
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)((call IDManager.getMyID(ADDR_16B))->addr_16b[1]);
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)((call IDManager.getMyID(ADDR_16B))->addr_16b[0]);
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)statusElement;        //type of element
         for (counter=0;counter<length;counter++){
            output_buffer[output_buffer_index_write_increment()]=(uint8_t)buffer[counter];
         }
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';                  //postamble
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';
      }
      return SUCCESS;//TBC check not overwriting
   }
   async command error_t OpenSerial.printError(uint8_t calling_component, uint8_t error_code, errorparameter_t arg1, errorparameter_t arg2) {
      atomic {
         somethingInOutputBuffer=TRUE;
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';                  //preamble
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'E';                  //this is an error
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)((call IDManager.getMyID(ADDR_16B))->addr_16b[1]);
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)((call IDManager.getMyID(ADDR_16B))->addr_16b[0]);
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)calling_component;    //component generating error
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)error_code;           //error_code
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)((arg1 & 0xff00)>>8); //arg1
         output_buffer[output_buffer_index_write_increment()]=(uint8_t) (arg1 & 0x00ff);
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)((arg2 & 0xff00)>>8); //arg2
         output_buffer[output_buffer_index_write_increment()]=(uint8_t) (arg2 & 0x00ff);
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';                  //postamble
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';
      }
//      call Leds.led0Toggle();
      return SUCCESS;//TBC check not overwriting
   }
   command error_t OpenSerial.printData(uint8_t* buffer, uint8_t length) {
      uint8_t counter;
      atomic {
         somethingInOutputBuffer=TRUE;
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';                  //preamble
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'^';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'D';                  //this is data
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)((call IDManager.getMyID(ADDR_16B))->addr_16b[1]);
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)((call IDManager.getMyID(ADDR_16B))->addr_16b[0]);
         for (counter=0;counter<length;counter++){
            output_buffer[output_buffer_index_write_increment()]=(uint8_t)buffer[counter];
         }
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';                  //postamble
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';
         output_buffer[output_buffer_index_write_increment()]=(uint8_t)'$';
      }
      return SUCCESS;//TBC check not overwriting
   }

   //------ [with components] get data from the input buffer
   command uint8_t OpenSerial.getNumDataBytes() {
      uint16_t temp_input_buffer_fill_level;
      atomic temp_input_buffer_fill_level = input_buffer_fill_level;
      return temp_input_buffer_fill_level;
   }
   //the component calling that function should retrieve all the bytes stored in the buffer
   command uint8_t OpenSerial.getInputBuffer(uint8_t* bufferToWrite, uint8_t maxNumBytes) {
      uint8_t numBytesWritten;
      uint16_t temp_input_buffer_fill_level;
      atomic temp_input_buffer_fill_level = input_buffer_fill_level;
      if (maxNumBytes<temp_input_buffer_fill_level) {
         call OpenSerial.printError(COMPONENT_OPENSERIAL,ERR_GETDATA_ASKS_TOO_FEW_BYTES,
               (errorparameter_t)maxNumBytes,
               (errorparameter_t)temp_input_buffer_fill_level);
         numBytesWritten = 0;
      } else {
         numBytesWritten = temp_input_buffer_fill_level;
         memcpy(bufferToWrite,&(input_buffer[0]),numBytesWritten);
      }
      atomic input_buffer_fill_level=0;
      return numBytesWritten;
   }

   //------ [with uart] get data into the input buffer
   async command void OpenSerial.startInput() {
      if (input_buffer_fill_level>0) {
         call OpenSerial.printError(COMPONENT_OPENSERIAL,ERR_INPUTBUFFER_LENGTH,
               (errorparameter_t)input_buffer_fill_level,
               (errorparameter_t)0);
         input_buffer_fill_level = 0;
      }
      input_command[4] = SERIAL_INPUT_BUFFER_SIZE;
      call HplMsp430Usart.enableIntr();
      call Port63.set();
      atomic {
         mode                  = MODE_INPUT;
         input_command_index   = 0;
         ready_receive_command = FALSE;
         ready_receive_length  = FALSE;
         call HplMsp430Usart.tx(input_command[input_command_index]);
      }
   }

   //------ [with uart] get data out of the input buffer
   async command void OpenSerial.startOutput() {
      //schedule a task to get new status in the output buffer
      post taskTriggerStatus();
      //print out what's in the buffer now
      call HplMsp430Usart.enableIntr();
      call Port63.set();
      atomic mode=MODE_OUTPUT;
      atomic if (somethingInOutputBuffer) {
         call HplMsp430Usart.tx(output_buffer[output_buffer_index_read_increment()]);
      } else {
         call OpenSerial.stop();
      }
   }

   //------ [with uart] hardware details
   async event void HplMsp430UsartInterrupts.txDone() {
      bool temp_mode;
      atomic{
         temp_mode = mode;
      }
      switch (temp_mode) {
         case MODE_INPUT:
            input_command_index++;
            if (input_command_index<sizeof(input_command)) {
               call HplMsp430Usart.tx(input_command[input_command_index]);
            } else {
               ready_receive_command = TRUE;
            }
            break;
         case MODE_OUTPUT:
            if (output_buffer_index_write==output_buffer_index_read) {
               somethingInOutputBuffer=FALSE;
               call Port63.clr();
            }
            if (somethingInOutputBuffer) {
               call HplMsp430Usart.tx(output_buffer[output_buffer_index_read_increment()]);
            }
            break;
         case MODE_OFF:
         default:
            break;
      }
   }
   async event void HplMsp430UsartInterrupts.rxDone(uint8_t rx_char) {
      bool temp_mode;
      atomic{
         temp_mode = mode;
      }
      if (temp_mode==MODE_INPUT) {
         if (ready_receive_command==TRUE) {
            ready_receive_command=FALSE;
            received_command=rx_char;
             ready_receive_length=TRUE;
         } else if (ready_receive_length==TRUE) {
            ready_receive_length=FALSE;
            input_buffer_bytes_still_to_come=rx_char;
         } else {
            input_buffer[input_buffer_fill_level++]=rx_char;
            if (input_buffer_fill_level+1>SERIAL_INPUT_BUFFER_SIZE){
               call OpenSerial.printError(COMPONENT_OPENSERIAL,ERR_INPUT_BUFFER_OVERFLOW,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
               input_buffer_fill_level=0;
               call OpenSerial.stop();
            }
            input_buffer_bytes_still_to_come--;
            if (input_buffer_bytes_still_to_come==0) {
               call OpenSerial.stop();
            }
         }
      }
   }
   async command void OpenSerial.stop() {
      uint16_t temp_input_buffer_fill_level;
      atomic temp_input_buffer_fill_level = input_buffer_fill_level;
      call HplMsp430Usart.disableIntr();
      call Port63.clr();
      atomic mode=MODE_OFF;
      if (temp_input_buffer_fill_level>0) {
         post taskHandleCommand();
      }
   }

   /*------------------- helper functions ---------------------------*/

   //DebugPrint
   command void DebugPrint.print(){
      post taskPrintDebug();
   }
   task void taskPrintDebug() {
      uint16_t temp_buffer[2];
      atomic temp_buffer[0] = output_buffer_index_write;
      atomic temp_buffer[1] = output_buffer_index_read;
      call OpenSerial.printStatus(STATUS_SERIALIOP_OUTPUTBUFFERINDEXES,(uint8_t*)temp_buffer,sizeof(temp_buffer));
   }

   uint16_t output_buffer_index_write_increment() {
      atomic output_buffer_index_write=(output_buffer_index_write+1)%SERIAL_OUTPUT_BUFFER_SIZE;
      return output_buffer_index_write;  
   }

   uint16_t output_buffer_index_read_increment() {
      atomic output_buffer_index_read=(output_buffer_index_read+1)%SERIAL_OUTPUT_BUFFER_SIZE;
      atomic return output_buffer_index_read;
   }

   task void taskHandleCommand() {
      uint8_t temp_received_command;
      atomic {
         temp_received_command = received_command;
      }
      switch (temp_received_command) {
         case 'R': //Trigger IDManager about isRoot
            call TriggerIDManagerAboutRoot.trigger();
            break;
         case 'B': //Trigger IDManager about isBridge
            call TriggerIDManagerAboutBridge.trigger();
            break;
         case 'T': //Trigger TCPInject
            call TriggerTCPInject.trigger();
            break;
         case 'U': //Trigger UDPInject
            call TriggerUDPInject.trigger();
            break;
         case 'S': //Trigger UDPSensor
            call TriggerUDPSensor.trigger();
            break;
         case 'E': //Trigger ICMPv6Echo
            call TriggerICMPv6Echo.trigger();
            break;
         case 'O': //Trigger ICMPv6Router
            call TriggerICMPv6Router.trigger();
            break;
         case 'P': //Trigger ICMPv6RPL
            call TriggerICMPv6RPL.trigger();
            break;
         case 'D': //Trigger OpenBridge (called only by OpenLBR)
            call TriggerOpenBridge.trigger();
            break;
         default:
            call OpenSerial.printError(COMPONENT_OPENSERIAL,ERR_UNSUPPORTED_COMMAND,
                  (errorparameter_t)temp_received_command,
                  (errorparameter_t)0);
            atomic input_buffer_fill_level = 0;
            break;
      }
   }

   task void taskTriggerStatus() {
      uint8_t temp_debugPrintCounter; //to avoid many atomics
      atomic debugPrintCounter=(debugPrintCounter+1)%7;
      atomic temp_debugPrintCounter = debugPrintCounter;
      switch (temp_debugPrintCounter) {
         case 0: call PrintMyDAGrank.print();   break;
         case 1: call PrintCellUsage.print();   break;
         case 2: call PrintNeighbors.print();   break;
         case 3: call PrintOpenSerial.print();  break;
         case 4: call PrintOpenQueue.print();   break;
         case 5: call PrintIEEE802154E.print(); break;
         case 6: call PrintIDManager.print();   break;
                 //case ?: call PrintRPL.print();  break;
         default: atomic debugPrintCounter=0;
      }
   }

   event void Notify.notify(button_state_t val) {
      if (val==TRUE) {
         //do something
      }
   }
}
