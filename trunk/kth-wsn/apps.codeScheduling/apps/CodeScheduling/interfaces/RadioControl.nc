interface RadioControl
{
   /*
    * switch on the voltage regulator and the oscillator of the radio.
    * can be called only once.
    */
   async command error_t  start();
   /*
    * indicates that the radio is ready, i.e. you can now call start transmitting
    * or receiving
    */
   async event   void     startDone(error_t error);
   /*
    * configures the radio registers to the right frequency channel, but does *not* turn
    * the radio on in Rx mode
    */
   async command error_t  prepareReceive(uint8_t frequencyChannel);
   /* 
    * the configuration is done
    */
   async event   void     prepareReceiveDone(error_t error);
   /*
    * turn the radio to Rx mode. if nothing is received after the radio is on for rxWaitTime (iif timeLimitedRx==TRUE),
    * the drivers call receivedNothing.
    */
   async command error_t  receiveNow(bool timeLimitedRx, uint16_t rxWaitTime);
   /*
    * called when the radio is in Rx mode for TsRxWaitTime and hasn't received anything.
    */
   async event   void     receivedNothing();
   /* 
    * turn the radio to IDLE mode, i.e. not in Tx or Rx mode. Note that the radio should
    * keep its voltage regulator and oscillator on
    */
   async command error_t  rfOff();
   /*
    * turn off the voltage regulator and oscillator
    */
   async command error_t  stop();
   /*
    * the voltage regulator and oscillator have been turned off
    */
   async event   void     stopDone(error_t error);
}
