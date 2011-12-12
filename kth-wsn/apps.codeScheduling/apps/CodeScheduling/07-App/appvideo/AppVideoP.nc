#include "LATIN.h"
#include "UserButton.h"
#include "AppVideo.h"

module AppVideoP {
   //admin
   provides interface Init as SoftwareInit;
   uses interface Boot;
   uses interface SplitControl;
   //data flow
   uses interface UDPSend;
   provides interface AppReceive;
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger;
   uses interface Timer<TMilli> as TimerMeasure;
   //private
   uses interface Notify<button_state_t>;
   uses interface HplMsp430GeneralIO as Port34;  //private, general debug
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   OpenQueueEntry_t* pkt;
   bool              busyFillingPacket;
   uint8_t           fillingPacketIndex;
   uint8_t           dataCounter=0;
   open_addr_t       destination;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskBuildDATA();

   /*-------------------------------- measurement sequence ------------------------------*/

   command void Trigger.trigger() {
      //available if needed
   }
   event void Notify.notify(button_state_t val) {
      post taskBuildDATA();
   }
   event void TimerMeasure.fired() {
      post taskBuildDATA(); 
   }

   task void taskBuildDATA() {
      uint8_t numDataBytes;
      if (busyFillingPacket==FALSE) {
         busyFillingPacket=TRUE;
         pkt = call Malloc.getFreePacketBuffer();
         if (pkt==NULL) {
            call OpenSerial.printError(COMPONENT_APP,ERR_NO_FREE_PACKET_BUFFER,
                  (errorparameter_t)0,
                  (errorparameter_t)0);
            busyFillingPacket=FALSE;
            return;
         }
         pkt->creator             = COMPONENT_APP;
         pkt->owner               = COMPONENT_APP;
         pkt->transmitInFirstSlot = TRUE;
         pkt->payload -= sizeof(video_data_ht);
         numDataBytes  = call OpenSerial.getData(pkt->payload,sizeof(video_data_ht));
         if (numDataBytes>0) {
            pkt->length  += numDataBytes;
            destination.type = ADDR_128B;
            destination.addr_128b[0]  = 0x20;
            destination.addr_128b[1]  = 0x01;
            destination.addr_128b[2]  = 0x04;
            destination.addr_128b[3]  = 0x70;
            destination.addr_128b[4]  = 0x1F;
            destination.addr_128b[5]  = 0x05;
            destination.addr_128b[6]  = 0x09;
            destination.addr_128b[7]  = 0x8E;
            destination.addr_128b[8]  = 0x00;
            destination.addr_128b[9]  = 0x00;
            destination.addr_128b[10] = 0x00;
            destination.addr_128b[11] = 0x00;
            destination.addr_128b[12] = 0x00;
            destination.addr_128b[13] = 0x00;
            destination.addr_128b[14] = 0x00;
            destination.addr_128b[15] = 0x07;
            if ((call UDPSend.send(pkt,&destination))==FAIL) {
               call Malloc.freePacketBuffer(pkt);
               busyFillingPacket=FALSE;
            }
         } else {
            //call OpenSerial.printError(COMPONENT_APP,ERR_POIPOI,(errorparameter_t)0,(errorparameter_t)1);
            call Malloc.freePacketBuffer(pkt);
            busyFillingPacket=FALSE;
            call TimerMeasure.startOneShot(100);
         }
      } else {
         //call OpenSerial.printError(COMPONENT_APP,ERR_POIPOI,(errorparameter_t)0,(errorparameter_t)2);        
      }
   }

   event void UDPSend.sendDone(OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_APP;
      if (msg->creator!=COMPONENT_APP) {
         call OpenSerial.printError(COMPONENT_APP,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      call Malloc.freePacketBuffer(msg);
      busyFillingPacket=FALSE;
      post taskBuildDATA();
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      busyFillingPacket=FALSE;
      return SUCCESS;
   }

   event void Boot.booted() {
      call OpenSerial.printError(COMPONENT_APP,ERR_BOOTED,0,0);
      call Notify.enable();
      call SplitControl.start();
   }

   event void SplitControl.startDone(error_t error) {
      //call OpenSerial.printError(COMPONENT_APP,ERR_POIPOI,(errorparameter_t)0,(errorparameter_t)3);
      call TimerMeasure.startOneShot(1000);
   }
   event void SplitControl.stopDone(error_t error) {
   }

   command void AppReceive.receive(OpenQueueEntry_t* msg) {
      call OpenSerial.printData((uint8_t*)(msg->payload),msg->length);
      call Malloc.freePacketBuffer(msg);
   }
}
