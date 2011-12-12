//#include "OpenWSN.h"

interface RadioSend {
   ///////async command error_t prepareSend(OpenQueueEntry_t* msg);
   async command error_t prepareSend(uint8_t* msg);
   async   event   void    prepareSendDone(error_t error);
   async   command error_t sendNow();
   event   void    sendNowDone(error_t error);
}
