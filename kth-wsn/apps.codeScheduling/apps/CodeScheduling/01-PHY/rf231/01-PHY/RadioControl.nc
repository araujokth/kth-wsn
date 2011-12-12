interface RadioControl {
   async command error_t  start();
   event   void     startDone(error_t error);
   async command error_t  prepareReceive(uint8_t someChannel);
   async event   void     prepareReceiveDone(error_t error);
   async command error_t  receiveNow();
   async event   void     receivedNothing();
   async command error_t  rfOff();
   command error_t  stop();
   event   void     stopDone(error_t error);
}
