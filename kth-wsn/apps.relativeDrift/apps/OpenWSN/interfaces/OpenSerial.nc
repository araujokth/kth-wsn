interface OpenSerial {
   //getting data which was sent over the serial port
         command uint8_t getInputBuffer(uint8_t * bufferToWrite, uint8_t numBytes);
         command uint8_t getNumDataBytes();
   //printing debug information
         command error_t printData(uint8_t* buffer, uint8_t length);
         command error_t printStatus(uint8_t statusElement,uint8_t* buffer, uint16_t length);
   async command error_t printError(uint8_t calling_component,
         uint8_t error_code,
         errorparameter_t arg1,
         errorparameter_t arg2);
   //low-level commands to start using the serial port
   async command void    startInput();
   async command void    startOutput();
   async command void    stop();
}
