#include "OpenWSN.h"

interface NeighborStats {
   async command void indicateTx(open_addr_t* dest, bool was_acked );
   async command void indicateRx(open_addr_t* src,  uint16_t rssi  );
}
