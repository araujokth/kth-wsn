#include "OpenWSN.h"

interface RadioSend {
   /*
    * write msg into the radio's TXFIFO, and configure the radio's register to transmit on the specified
    * frequency channel. Do *not* turn on the radio in Tx mode.
    */
   async   command error_t prepareSend(OpenQueueEntry_t* msg);
   /*
    * radio is loaded with msg and channel configuration
    */
   async   event   void    prepareSendDone(error_t error);
   /*
    * turn the radio in Tx mode; assumes prepareSend was called before
    */
   async   command error_t sendNow();
   /*
    * packet has been sent. note that the radio stays on after sending.
    */
   async   event   void    sendNowDone(error_t error);
}
