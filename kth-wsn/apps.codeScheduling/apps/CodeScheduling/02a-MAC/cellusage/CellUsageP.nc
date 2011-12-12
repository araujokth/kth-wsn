#include "LATIN.h"
#include "CellUsage.h"

module CellUsageP {
   provides interface Init as SoftwareInit;
   provides interface CellUsageGet;
   provides interface DebugPrint;
   provides interface CellStats;
   uses interface GlobalTime;
//   uses interface OpenSerial;
}

implementation {
   /*-------------------------------- variables -----------------------------------------*/

   cellUsageInformation_t cellTable[LENGTHCELLFRAME];

   bool         cellStatsBusy;
   slotOffset_t cellStatsSlotOffset;
   bool         cellStatsACK;

   slotOffset_t DebugSlotOffset=0;

   /*-------------------------------- prototypes ----------------------------------------*/

   void initSlotStats(slotOffset_t slotOffset);
   task void taskCellStatsUpdate();
   task void taskPrintTable();

   /*-------------------------------- helper functions ----------------------------------*/

   void initSlotStats(slotOffset_t slotOffset) {
      cellTable[slotOffset].numUsed = 0;
      cellTable[slotOffset].numTxACK = 0;
      cellTable[slotOffset].timestamp = 0;
   }
   task void taskCellStatsUpdate() {
      atomic {
         if (cellTable[cellStatsSlotOffset].numUsed==0xFF) {
            cellTable[cellStatsSlotOffset].numUsed/=2;
            cellTable[cellStatsSlotOffset].numTxACK/=2;
         }
         cellTable[cellStatsSlotOffset].numUsed++;
         if (cellStatsACK==TRUE) {
            cellTable[cellStatsSlotOffset].numTxACK++;
         }
         cellTable[cellStatsSlotOffset].timestamp=(call GlobalTime.getLocalTime());
         cellStatsBusy = FALSE;
      }
   }
   task void taskPrintTable() {
      debugCellUsageInformation_t temp;
      DebugSlotOffset = (DebugSlotOffset+1)%LENGTHCELLFRAME;
      temp.row        = DebugSlotOffset;
      temp.cellUsage  = cellTable[DebugSlotOffset];
//      call OpenSerial.printStatus(STATUS_CELLUSAGEP_CELLTABLE,(uint8_t*)&temp,sizeof(debugCellUsageInformation_t));
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   //SoftwareInit
   command error_t SoftwareInit.init() {
      uint8_t slotCounter;
      atomic cellStatsBusy = FALSE;
      //all slots OFF
      for (slotCounter=0;slotCounter<LENGTHCELLFRAME;slotCounter++){
         cellTable[slotCounter].type          = CELLTYPE_OFF;
         cellTable[slotCounter].channelOffset = 0;
         cellTable[slotCounter].ADV           = FALSE;
         cellTable[slotCounter].SH_TX         = FALSE;
         cellTable[slotCounter].RX            = FALSE;
         cellTable[slotCounter].TX            = FALSE;
         initSlotStats(slotCounter);
      }
//      //for general OpenWSN demo:
//      //slot 0 is advertisement slot
//      cellTable[0].type                       = CELLTYPE_TXRX;
//      cellTable[0].ADV                        = TRUE;
//      cellTable[0].RX                         = TRUE;
//      cellTable[0].TX                         = TRUE;
//      //slot 1 is slotted ALOHA
//      cellTable[1].type                       = CELLTYPE_TXRX;
//      cellTable[1].SH_TX                      = TRUE;
//      cellTable[1].RX                         = TRUE;
//      //slot 2 is receive over serial
//      cellTable[2].type                       = CELLTYPE_RXSERIAL;
//      //slot 3 is OFF to write to serial
//      cellTable[3].type                       = CELLTYPE_OFF;
//
//      /* for video over WSNs
//      //slot 0 is advertisement slot
//      cellTable[0].type                       = CELLTYPE_TXRX;
//      cellTable[0].ADV                        = TRUE;
//      cellTable[0].RX                         = TRUE;
//      cellTable[0].TX                         = TRUE;
//      //slot 1 is slotted ALOHA
//      cellTable[1].type                       = CELLTYPE_TXRX;
//      cellTable[1].SH_TX                      = TRUE;
//      cellTable[1].RX                         = TRUE;
//      //slot 2 is slotted ALOHA
//      cellTable[2].type                       = CELLTYPE_TXRX;
//      cellTable[2].SH_TX                      = TRUE;
//      cellTable[2].RX                         = TRUE;
//      //slot 3 is receive over serial
//      cellTable[3].type                       = CELLTYPE_RXSERIAL;
//      //slot 4 is OFF to write to serial
//      cellTable[4].type                       = CELLTYPE_OFF;*/


      //for debugging:
      //slot 0 is advertisement slot
      cellTable[0].type                       = CELLTYPE_TXRX;
      cellTable[0].ADV                        = TRUE;
      cellTable[0].RX                         = TRUE;
      cellTable[0].TX                         = TRUE;
      //slot 1 is slotted ALOHA
      cellTable[1].type                       = CELLTYPE_TXRX;
      cellTable[1].SH_TX                      = TRUE;
      cellTable[1].RX                         = TRUE;
      //slot 2 is receive over serial
      cellTable[2].type                       = CELLTYPE_TXRX;
      cellTable[2].SH_TX                      = TRUE;
      cellTable[2].RX                         = TRUE;
      //slot 3 is OFF to write to serial
      cellTable[3].type                       = CELLTYPE_TXRX;
      cellTable[3].SH_TX                      = TRUE;
      cellTable[3].RX                         = TRUE;

      cellTable[4].type                       = CELLTYPE_TXRX;
      cellTable[4].SH_TX                      = TRUE;
      cellTable[4].RX                         = TRUE;

      cellTable[5].type                       = CELLTYPE_TXRX;
      cellTable[5].SH_TX                      = TRUE;
      cellTable[5].RX                         = TRUE;

      cellTable[6].type                       = CELLTYPE_TXRX;
      cellTable[6].SH_TX                      = TRUE;
      cellTable[6].RX                         = TRUE;

      cellTable[7].type                       = CELLTYPE_TXRX;
      cellTable[7].SH_TX                      = TRUE;
      cellTable[7].RX                         = TRUE;

      cellTable[8].type                       = CELLTYPE_TXRX;
      cellTable[8].SH_TX                      = TRUE;
      cellTable[8].RX                         = TRUE;

      cellTable[9].type                       = CELLTYPE_TXRX;
      cellTable[9].SH_TX                      = TRUE;
      cellTable[9].RX                         = TRUE;

      cellTable[10].type                       = CELLTYPE_TXRX;
      cellTable[10].SH_TX                      = TRUE;
      cellTable[10].RX                         = TRUE;

      cellTable[11].type                       = CELLTYPE_TXRX;
      cellTable[11].SH_TX                      = TRUE;
      cellTable[11].RX                         = TRUE;

      cellTable[12].type                       = CELLTYPE_TXRX;
      cellTable[12].SH_TX                      = TRUE;
      cellTable[12].RX                         = TRUE;
      cellTable[13].type                       = CELLTYPE_TXRX;
      cellTable[13].SH_TX                      = TRUE;
      cellTable[13].RX                         = TRUE;
      cellTable[14].type                       = CELLTYPE_TXRX;
      cellTable[14].SH_TX                      = TRUE;
      cellTable[14].RX                         = TRUE;
      cellTable[15].type                       = CELLTYPE_TXRX;
      cellTable[15].SH_TX                      = TRUE;
      cellTable[15].RX                         = TRUE;




      return SUCCESS;
   }

   //CellUsageGet
   async command cellType_t CellUsageGet.getType(slotOffset_t slotOffset) {
      return cellTable[slotOffset].type;
   }
   async command channelOffset_t CellUsageGet.getChannelOffset(slotOffset_t slotOffset) {
      return cellTable[slotOffset].channelOffset;
   }
   async command bool CellUsageGet.isADV(slotOffset_t slotOffset) {
      return cellTable[slotOffset].ADV;
   }
   async command bool CellUsageGet.isTX(slotOffset_t slotOffset) {
      return cellTable[slotOffset].TX;
   }
   async command bool CellUsageGet.isRX(slotOffset_t slotOffset) {
      return cellTable[slotOffset].RX;
   }
   async command bool CellUsageGet.isSH_TX(slotOffset_t slotOffset) {
      return cellTable[slotOffset].SH_TX;
   }

   //DebugPrint
   command void DebugPrint.print() {
      post taskPrintTable();
   }

   //CellStats
   async command void CellStats.indicateUse(slotOffset_t slotOffset, bool ack){
      atomic cellStatsBusy = TRUE;
      atomic cellStatsSlotOffset = slotOffset;
      atomic cellStatsACK = ack;
      post taskCellStatsUpdate();
   }
}
