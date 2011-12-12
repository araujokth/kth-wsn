interface CC2420Config {
   async command error_t startVReg();
   async event   void    startVRegDone();
   async command error_t startOscillator();
   async event   void    startOscillatorDone();
   async command error_t writeId();
   async event   void    writeIdDone();
   async command error_t setChannel( uint8_t channel );
   async event   void    setChannelDone( error_t error );
   async command error_t rxOn();
   async command error_t rfOff();
   async command error_t stopOscillator();
   async command error_t stopVReg(); 
}
