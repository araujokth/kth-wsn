#ifndef NEIGHBORS_H
#define NEIGHBORS_H



typedef struct neighborEntry_t {
   bool             used;
   uint8_t          parentPreference;
   bool             stableNeighbor;
   uint8_t          switchStabilityCounter;
   open_addr_t      addr_16b;
   open_addr_t      addr_64b;
   open_addr_t      addr_128b;
   dagrank_t        DAGrank;
   uint8_t          linkQuality;
   uint8_t          numRx;
   uint8_t          numTx;
   uint8_t          numTxACK;
   timervalue_t     timestamp;
} neighborEntry_t;

typedef struct debugNeighborEntry_t {
   slotOffset_t row;
   neighborEntry_t neighborEntry;
} debugNeighborEntry_t;

#endif
