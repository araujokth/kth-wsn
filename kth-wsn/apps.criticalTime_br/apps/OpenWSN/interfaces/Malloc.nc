#include "OpenWSN.h"

interface Malloc {
   async command OpenQueueEntry_t* getFreePacketBuffer();
   async command error_t freePacketBuffer(OpenQueueEntry_t* pkt);
   async command error_t resetQueue();
}
