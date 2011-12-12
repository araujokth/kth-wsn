#ifndef CELLUSAGE_H
#define CELLUSAGE_H

typedef struct cellUsageInformation_t {
   uint8_t       type;
   uint8_t       channelOffset;
   bool          ADV;
   bool          TX;
   bool          RX;
   bool          SH_TX;
   uint8_t       numUsed;
   uint8_t       numTxACK;
   timervalue_t  timestamp;
} cellUsageInformation_t;

typedef struct debugCellUsageInformation_t {
   uint8_t row;
   cellUsageInformation_t cellUsage;
} debugCellUsageInformation_t;

#endif
