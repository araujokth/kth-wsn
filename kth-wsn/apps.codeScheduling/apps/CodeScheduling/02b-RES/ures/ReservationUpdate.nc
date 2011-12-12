#include "LATIN.h"

interface ReservationUpdate {
   command uint8_t numSlots(shortnodeid_t neighbor);
   command error_t addSlot(shortnodeid_t neighbor);
   command error_t removeSlot(shortnodeid_t neighbor);
   command error_t removeAllOngoingResToNeighbor(shortnodeid_t neighbor);
   event void      done(bool add, shortnodeid_t neighbor, error_t error);
}
