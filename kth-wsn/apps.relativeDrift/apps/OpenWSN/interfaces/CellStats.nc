#include "OpenWSN.h"

interface CellStats {
   async command void indicateUse(slotOffset_t slotOffset, bool ack);
}
