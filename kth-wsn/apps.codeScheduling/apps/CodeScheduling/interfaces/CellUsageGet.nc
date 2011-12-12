#include "LATIN.h"

interface CellUsageGet {
   async command cellType_t      getType(slotOffset_t slotOffset);
   async command channelOffset_t getChannelOffset(slotOffset_t slotOffset);
   async command bool            isADV(slotOffset_t slotOffset);
   async command bool            isTX(slotOffset_t slotOffset);
   async command bool            isRX(slotOffset_t slotOffset);
   async command bool            isSH_TX(slotOffset_t slotOffset);
}
