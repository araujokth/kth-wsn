#include "OpenWSN.h"

interface OpenQueue {
   async command OpenQueueEntry_t* inQueue(bool isADV);
         command error_t removeAllPacketsToNeighbor(open_addr_t* neighbor);
}
