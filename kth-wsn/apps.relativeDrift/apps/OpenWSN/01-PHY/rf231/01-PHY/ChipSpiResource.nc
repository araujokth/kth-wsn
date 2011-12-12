interface ChipSpiResource {
   async event void releasing();
   async command void abortRelease();
   async command error_t attemptRelease();

   async command void changeState(uint8_t newState);
   async command void setChannel(uint8_t newChannel );
   async command uint8_t readRegister(uint8_t registerAddress );
   async command void setRegister(uint8_t regAddr, uint8_t newValue );
   async command void loadPacket(uint8_t* data, uint8_t len);
   async event   void loadPacketDone(error_t error);
   async command void readPacket(uint8_t* data);
   async event   void doneSettingChannel();


}
