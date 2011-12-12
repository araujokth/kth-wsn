#include "OpenWSN.h"

interface NeighborGet {
   command dagrank_t  getMyDAGrank();
   command uint8_t    getNumNeighbors();
   command void       getPreferredParent(open_addr_t* addressToWrite, uint8_t addr_type);
   command bool       isStableNeighbor(open_addr_t* address);
}
