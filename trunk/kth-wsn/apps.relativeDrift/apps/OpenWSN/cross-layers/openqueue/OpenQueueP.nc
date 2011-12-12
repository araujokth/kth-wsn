#include "OpenWSN.h"
#include "OpenQueue.h"

module OpenQueueP {
   provides interface Init as SoftwareInit;
   provides interface OpenQueue;
   provides interface Malloc;
   provides interface DebugPrint;
   uses interface GlobalTime;
   uses interface Leds;
//   uses interface OpenSerial;
}
implementation {

   /*-------------------------------- variables -----------------------------------------*/

   OpenQueueEntry_t queue[QUEUELENGTH];

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskDebugPrint();
   void reset_entry(uint8_t i);

   /*-------------------------------- helper functions ----------------------------------*/

   task void taskDebugPrint() {
      debugOpenQueueEntry_t output[QUEUELENGTH];
      uint8_t i;
      for (i=0;i<QUEUELENGTH;i++) {
         atomic {
            output[i].creator = queue[i].creator;
            output[i].owner   = queue[i].owner;
         }
      }
//      call OpenSerial.printStatus(STATUS_OPENQUEUEP_QUEUE,(uint8_t*)&output,QUEUELENGTH*sizeof(debugOpenQueueEntry_t));
   }

   void reset_entry(uint8_t i) {
      atomic {
         //admin
         queue[i].creator                     = COMPONENT_NULL;
         queue[i].owner                       = COMPONENT_NULL;
         queue[i].payload                     = &(queue[i].packet[126]);
         queue[i].length                      = 0;
         //l4
         queue[i].l4_protocol                 = IANA_UNDEFINED;
         //l3
         queue[i].l3_destinationORsource.type = ADDR_NONE;
         //l2
         queue[i].l2_nextORpreviousHop.type   = ADDR_NONE;
         queue[i].l2_frameType                = IEEE154_TYPE_UNDEFINED;
         queue[i].l2_retriesLeft              = 0;
      }
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   //SoftwareInit
   command error_t SoftwareInit.init() {
      uint8_t i;
      for (i=0;i<QUEUELENGTH;i++){
         reset_entry(i);
      }
      return SUCCESS;
   }

   //Malloc
   async command OpenQueueEntry_t* Malloc.getFreePacketBuffer() {
      uint8_t i;
      for (i=0;i<QUEUELENGTH;i++) {
         atomic if (queue[i].owner==COMPONENT_NULL) {
            atomic queue[i].owner=COMPONENT_OPENQUEUE;
            return &queue[i];
         }
      }
      return NULL;
   }
   async command error_t Malloc.freePacketBuffer(OpenQueueEntry_t* pkt) {
      uint8_t i;
      for (i=0;i<QUEUELENGTH;i++) {
         if (&queue[i]==pkt) {
            reset_entry(i);
            return SUCCESS;
         }
      }
      return FAIL;
   }
   async command error_t Malloc.resetQueue() {
	      uint8_t i;
	      for (i=0;i<QUEUELENGTH;i++){
	         reset_entry(i);
	      }
	      return SUCCESS;
   }

   //OpenQueue
   async command OpenQueueEntry_t* OpenQueue.inQueue(bool isADV) {
      uint8_t i;
      switch(isADV) {
         case IS_ADV:

            for (i=0;i<QUEUELENGTH;i++){
//               atomic if (queue[i].owner==COMPONENT_MAC && queue[i].creator==COMPONENT_RES) {
            	atomic if (queue[i].creator==COMPONENT_RES) {
                  return &queue[i];
               }
            }
            break;
         case IS_NOT_ADV:

            for (i=0;i<QUEUELENGTH;i++) {
               atomic if (queue[i].owner==COMPONENT_MAC && queue[i].creator!=COMPONENT_RES) {
                  return &queue[i];
               }
            }
            break;
      }
      return NULL;
   }
   command error_t OpenQueue.removeAllPacketsToNeighbor(open_addr_t* neighbor) {
      error_t returnValue=FAIL;
      /*uint8_t i;
        for (i=0;i<QUEUELENGTH;i++){
        atomic if (queue[i].owner==COMPONENT_MAC && ((IEEE802154_ht*)(queue[i].payload))->dest==neighbor) {
        queue[i].owner=COMPONENT_NULL;
        queue[i].l2_retriesLeft=0;
        returnValue=SUCCESS;
        }
        }poipoistupid*/
      return returnValue;
   }
   //DebugPrint
   command void DebugPrint.print() {
      post taskDebugPrint();
   }
}
