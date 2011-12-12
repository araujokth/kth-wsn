#include "OpenWSN.h"

interface RadioReceive {
   /* called by the drivers when a packet is received with a correct length
    * and a correct CRC
    */
   async event void receive(OpenQueueEntry_t* msg);
}
