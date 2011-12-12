#include "LATIN.h"
#include "CC2420.h"
#include "IEEE802154.h"

enum {
   //default
   S_IDLE_LISTENING      =     0,
   //transmitter
   S_TX_TXDATAPREPARE    =     1,
   S_TX_TXDATAREADY      =     2,
   S_TX_TXDATA           =     3,
   S_TX_RXACK            =     4,
   //receiver
   S_RX_TXACKPREPARE     =     5,
   S_RX_TXACKREADY       =     6,
   S_RX_TXACK            =     7,
};

enum {
   IMMEDIATELY           =     1, //used as timer value which is very small
   WATCHDOG_PREPARESEND  = 16000, //500ms
};

enum {
   WAS_ACKED             =   TRUE,
   WAS_NOT_ACKED         =  FALSE,
};

enum {
   ACK_WAIT_TIME         =   2200, //68ms poipoi 
   FREQUENCYCHANNEL      =     15,
   DEBUGPERIOD           =     30, //ms
   MINBACKOFF            =     20, //ms
   BACKOFFWINDOW         =     80, //ms
};

module stupidMACP {
   //admin
   uses interface Boot;
   provides interface Init as SoftwareInit;
   //time
   uses interface Timer<TMilli> as timerDebug;
   uses interface Timer<TMilli> as timerBackoff;
   uses interface Alarm<T32khz,uint32_t> as AlarmWatchdog; //watchdog for ack and preparesend
   provides interface GlobalTime;
   provides interface GlobalSync;
   //down the stack
   provides interface OpenSend as OpenSendFromUpper;
   uses interface OpenQueue;
   uses interface RadioControl;
   uses interface RadioSend;
   uses interface RadioReceive;
   //up the stack
   uses interface OpenReceive as OpenReceiveToUpper;
   //debug
   provides interface DebugPrint;
   uses interface OpenSerial;
   uses interface CellStats;
   uses interface Leds;                          //Led2=blue=bridge, Led1=green=radio on
   uses interface HplMsp430GeneralIO as Port26;  //new slot
   uses interface HplMsp430GeneralIO as Port35;  //radio on
   uses interface HplMsp430GeneralIO as Port67;  //new slotframe
   uses interface HplMsp430GeneralIO as Port34;  //general debug
   //misc
   uses interface PacketFunctions;
   uses interface NeighborStats;
   uses interface Malloc;
   uses interface CellUsageGet;
   uses interface IDManager;
   uses interface NeighborGet;
   uses interface Random;                              //private
}

implementation {

   /*------------------------------ variables -------------------------------------------*/

   OpenQueueEntry_t*  dataFrameToSend;     //NULL at beginning and end of slot
   OpenQueueEntry_t*  packetACK;           //NULL at beginning and end, free at end of slot
   OpenQueueEntry_t*  dataFrameReceived;   //!NULL between data received, and sent to upper layer
   uint8_t            dsn;
   uint8_t            state;
   bool               serialInOutputMode;
   error_t            sendDoneError;

   /*------------------------------ prototypes ------------------------------------------*/

#include "IEEE802154_common.c"
   void change_state(uint8_t newstate);
   task void taskDebugPrint();
   task void taskStartDebugClock();
   task void taskArmRandomBackoffTimer();

   //the two following tasks are used to break the asynchronicity: everything in MAC and below is async, all the above not
   task void taskReceive();
   task void taskSendDone();

   /*------------------------------ start sequence --------------------------------------*/

   //in stupidMAC, the radio is always on, listening
   event void Boot.booted() {
      call OpenSerial.printError(COMPONENT_MAC,ERR_BOOTED,0,0);
      call RadioControl.start();
   }
   async event void RadioControl.startDone(error_t error) {
      if ((call RadioControl.prepareReceive(FREQUENCYCHANNEL))!=SUCCESS) {
         call OpenSerial.printError(COMPONENT_MAC,ERR_PREPARERECEIVE_FAILED,
               (errorparameter_t)0,
               (errorparameter_t)0);
      };
   }
   async event void RadioControl.prepareReceiveDone(error_t error) {
      if (error!=SUCCESS) {
         call OpenSerial.printError(COMPONENT_MAC,ERR_PREPARERECEIVEDONE_FAILED,
               (errorparameter_t)error,
               (errorparameter_t)0);
      }
      if ( (call RadioControl.receiveNow(INFITE_RX, 0))!=SUCCESS ) {
         call OpenSerial.printError(COMPONENT_MAC,ERR_RECEIVENOW_FAILED,
               (errorparameter_t)0,
               (errorparameter_t)0);
         change_state(S_IDLE_LISTENING);
      };
      post taskStartDebugClock();
   }

   task void taskStartDebugClock() {
      call timerDebug.startPeriodic(DEBUGPERIOD);
   }

   async event void RadioControl.stopDone(error_t error) {
      return;//radio turned off not implemented
   }

   /*------------------------------send sequence ----------------------------------------*/

   //a packet sent from the upper layer is simply stored into the OpenQueue buffer.
   //The timerBackoff is armed to service the packet later on.
   command error_t OpenSendFromUpper.send(OpenQueueEntry_t* msg) {
      call OpenSerial.stop();
      post taskStartDebugClock();//ensures debug fired after packet sent
      //reset WDT
      atomic WDTCTL = WDTPW + WDTCNTCL + WDTSSEL;
      //metadata
      msg->owner = COMPONENT_MAC;
      if (call PacketFunctions.isBroadcastMulticast(&(msg->l2_nextORpreviousHop))==TRUE) {
         msg->l2_retriesLeft = 1;
      } else {
         msg->l2_retriesLeft = TXRETRIES;
      }
      msg->l1_txPower = TX_POWER;
      msg->l1_channel = FREQUENCYCHANNEL;
      //IEEE802.15.4 header
      prependIEEE802154header(msg,
            msg->l2_frameType,
            IEEE154_SEC_NO_SECURITY,
            dsn++,
            &(msg->l2_nextORpreviousHop)
            );
      //arm timer to send in the future
      call timerBackoff.startOneShot(IMMEDIATELY);
      return SUCCESS;
   }

   //this function is the one which really initiates the transmission of a packet.
   //It only does so if the MAC layer is in S_IDLE_LISTENING state, otherwise it defers
   event void timerBackoff.fired() {
      OpenQueueEntry_t* temp_dataFrameToSend;
      uint8_t temp_state;
      atomic {
         temp_state = state;
      }
      if (temp_state==S_IDLE_LISTENING) {
         atomic temp_dataFrameToSend = dataFrameToSend;
         if (temp_dataFrameToSend!=NULL) {
            call OpenSerial.printError(COMPONENT_MAC,ERR_DATAFRAMETOSEND_ERROR,
                  (errorparameter_t)0,
                  (errorparameter_t)0);
         }
         atomic {
            dataFrameToSend = call OpenQueue.inQueue(IS_NOT_ADV);
            if (dataFrameToSend==NULL) {
               dataFrameToSend = call OpenQueue.inQueue(IS_ADV);
            }
            temp_dataFrameToSend = dataFrameToSend;
         }
         if (temp_dataFrameToSend!=NULL) {
            change_state(S_TX_TXDATAPREPARE);
            if ((call RadioSend.prepareSend(temp_dataFrameToSend))!=SUCCESS) {
               atomic sendDoneError = FAIL;
               post taskSendDone();
               call OpenSerial.printError(COMPONENT_MAC,ERR_PREPARESEND_FAILED,
                     (errorparameter_t)0,
                     (errorparameter_t)0);
            } else {
               call AlarmWatchdog.start(WATCHDOG_PREPARESEND);
            }
         }
      } else {
         //retry later on
         post taskArmRandomBackoffTimer();
      }
   }

   async event void AlarmWatchdog.fired() {
      uint8_t           temp_state;
      OpenQueueEntry_t* temp_dataFrameToSend;
      atomic{
         temp_state           = state;
         temp_dataFrameToSend = dataFrameToSend;
      }
      switch (temp_state) {
         case S_TX_TXDATAPREPARE:
            atomic call OpenSerial.printError(COMPONENT_MAC,ERR_POIPOI,
                  (errorparameter_t)0,
                  (errorparameter_t)0);
            atomic sendDoneError = FAIL;
            post taskSendDone();
            //poipoi reset radio (call RadioControl.prepareReceive(FREQUENCYCHANNEL)?)
            break;
         case S_TX_RXACK:
            //I'm a transmitter, didn't receive ACK (end of TX sequence).
            call NeighborStats.indicateTx(&(temp_dataFrameToSend->l2_nextORpreviousHop),WAS_NOT_ACKED);
            temp_dataFrameToSend->l2_retriesLeft--;
            if (temp_dataFrameToSend->l2_retriesLeft==0) {
               atomic sendDoneError = FAIL;
               post taskSendDone();
               break;
            }
            //retransmit later on
            post taskArmRandomBackoffTimer();
            atomic dataFrameToSend = NULL;
            change_state(S_IDLE_LISTENING);
            break;
         default:
            call OpenSerial.printError(COMPONENT_MAC,ERR_WRONG_STATE_IN_FASTTIMER_FIRED,
                  (errorparameter_t)temp_state,
                  (errorparameter_t)0);
            change_state(S_IDLE_LISTENING);
            break;
      }
   }

   async event void RadioSend.prepareSendDone(error_t error) {
      uint8_t temp_state;
      OpenQueueEntry_t* temp_dataFrameToSend;
      atomic {
         temp_state = state;
         temp_dataFrameToSend = dataFrameToSend;
      }
      call AlarmWatchdog.stop();
      if (error!=SUCCESS) {
         atomic sendDoneError = FAIL;
         post taskSendDone();
         call OpenSerial.printError(COMPONENT_MAC,ERR_PREPARESENDDONE_FAILED,
               (errorparameter_t)temp_state,
               (errorparameter_t)0);
      }
      switch (temp_state) {
         case S_TX_TXDATAPREPARE:
            change_state(S_TX_TXDATAREADY);
            if ((call RadioSend.sendNow())!=SUCCESS) {
               atomic sendDoneError = FAIL;
               post taskSendDone();
               call OpenSerial.printError(COMPONENT_MAC,ERR_SENDNOW_FAILED,
                     (errorparameter_t)temp_state,
                     (errorparameter_t)0);
               return;
            }
            change_state(S_TX_TXDATA);
            break;
         case S_RX_TXACKPREPARE:
            change_state(S_RX_TXACKREADY);
            if ((call RadioSend.sendNow())!=SUCCESS) {
               atomic call Malloc.freePacketBuffer(packetACK);
               call OpenSerial.printError(COMPONENT_MAC,ERR_SENDNOW_FAILED,
                     (errorparameter_t)temp_state,
                     (errorparameter_t)0);
               change_state(S_IDLE_LISTENING);
               return;
            }
            change_state(S_RX_TXACK);
            break;
         default:
            call OpenSerial.printError(COMPONENT_MAC,ERR_WRONG_STATE_IN_PREPARESENDDONE,
                  (errorparameter_t)temp_state,
                  (errorparameter_t)0);
            break;
      }
   }

   async event void RadioSend.sendNowDone(error_t error) {
      uint8_t           temp_state;
      OpenQueueEntry_t* temp_dataFrameToSend;
      atomic {
         temp_state           = state;
         temp_dataFrameToSend = dataFrameToSend;
      }
      switch (temp_state) {
         case S_TX_TXDATA:                                           //[sendNowDone] transmitter
            if (error!=SUCCESS) {
               atomic sendDoneError = FAIL;
               post taskSendDone();
               call OpenSerial.printError(COMPONENT_MAC,ERR_SENDNOWDONE_FAILED,
                     (errorparameter_t)temp_state,
                     (errorparameter_t)0);
               return;
            } else {
               call AlarmWatchdog.start(ACK_WAIT_TIME);
               change_state(S_TX_RXACK);
            }
            break;
         case S_RX_TXACK:                                            //[sendNowDone] receiver
            //I'm a receiver, finished sending ACK (end of RX sequence)
            atomic call Malloc.freePacketBuffer(packetACK);
            post taskReceive();
            change_state(S_IDLE_LISTENING);
            break;
         default:
            call OpenSerial.printError(COMPONENT_MAC,ERR_WRONG_STATE_IN_SUBSEND_SENDDONE,
                  (errorparameter_t)temp_state,
                  (errorparameter_t)0);
            change_state(S_IDLE_LISTENING);
            break;
      }
   }

   task void taskSendDone() {
      error_t temp_sendDoneError;
      OpenQueueEntry_t*  temp_dataFrameToSend;
      atomic {
         temp_sendDoneError = sendDoneError;
         temp_dataFrameToSend = dataFrameToSend;
      }
      signal OpenSendFromUpper.sendDone(temp_dataFrameToSend,temp_sendDoneError);
      post taskArmRandomBackoffTimer();
      atomic dataFrameToSend = NULL;
      change_state(S_IDLE_LISTENING);
   }

   /*------------------------------ receive sequence ------------------------------------*/

   async event void RadioReceive.receive(OpenQueueEntry_t* msg) {
      uint8_t                 temp_state;
      OpenQueueEntry_t*       temp_dataFrameToSend;
      OpenQueueEntry_t*       temp_dataFrameReceived;
      OpenQueueEntry_t*       temp_packetACK;
      error_t                 temp_error;
      ieee802154_header_iht   received_ieee154_header;
      ieee802154_header_iht   transmitted_ieee154_header;
      atomic {
         temp_state             = state;
         temp_dataFrameToSend   = dataFrameToSend;
         temp_dataFrameReceived = dataFrameReceived;
      }

      call Leds.led1On();
      atomic P3OUT |= 0x20;

      call OpenSerial.stop();
      //ensure debug fires only after packet fully received
      post taskStartDebugClock();
      //reset WDT
      atomic WDTCTL = WDTPW + WDTCNTCL + WDTSSEL;

      msg->owner = COMPONENT_MAC;

      atomic P3OUT &= ~0x20;
      call Leds.led1Off();

      if (temp_state!=S_TX_RXACK && temp_state!=S_IDLE_LISTENING) {
         //not expecting this packet, throw away
         //do not go back to S_IDLE_LISTENING, just don't receive the packet and let the state machine be where it was
         call Malloc.freePacketBuffer(msg);
         return;
      }

      received_ieee154_header = retrieveIEEE802154header(msg);
      call PacketFunctions.tossHeader(msg,received_ieee154_header.headerLength);
      call PacketFunctions.tossFooter(msg,2);

      msg->l2_frameType = received_ieee154_header.frameType;
      memcpy(&(msg->l2_nextORpreviousHop),&(received_ieee154_header.src),sizeof(open_addr_t));
      if (   received_ieee154_header.frameType==IEEE154_TYPE_DATA      &&
            !(call IDManager.isMyAddress(&received_ieee154_header.panid))) {
         call OpenSerial.printError(COMPONENT_MAC,ERR_WRONG_PANID,
               (errorparameter_t)received_ieee154_header.panid.panid[0]*256+received_ieee154_header.panid.panid[1],
               (errorparameter_t)0);
         call Malloc.freePacketBuffer(msg);
         return;
      }

      switch (temp_state) {

         /*------------------- TX sequence ------------------------*/
         case S_TX_RXACK:                                            //[receive] transmitter
            transmitted_ieee154_header = retrieveIEEE802154header(temp_dataFrameToSend);
            if (received_ieee154_header.dsn == transmitted_ieee154_header.dsn) {
               //I'm a transmitter, just received ACK (end of TX sequence)
               call AlarmWatchdog.stop();
               call NeighborStats.indicateTx(&(temp_dataFrameToSend->l2_nextORpreviousHop),WAS_ACKED);
               atomic sendDoneError = SUCCESS;
               post taskSendDone();
            }
            call Malloc.freePacketBuffer(msg);//free packet I received         
            break;

            /*------------------- RX sequence ------------------------*/
         case S_IDLE_LISTENING:                                           //[receive] receiver
            //I'm a receiver, just received a packet
            if (received_ieee154_header.frameType==IEEE154_TYPE_DATA || received_ieee154_header.frameType==IEEE154_TYPE_CMD) {
               call NeighborStats.indicateRx(&(msg->l2_nextORpreviousHop),msg->l1_rssi);
               if (call IDManager.isMyAddress(&received_ieee154_header.dest)) {
                  //this packet is unicast to me
                  if (temp_dataFrameReceived==NULL) {
                     atomic dataFrameReceived = msg;
                  } else {
                     call OpenSerial.printError(COMPONENT_MAC,ERR_BUSY_RECEIVING,
                           (errorparameter_t)0,
                           (errorparameter_t)0);
                     atomic call Malloc.freePacketBuffer(msg);
                  }
                  //the sender requests an ACK
                  if (received_ieee154_header.ackRequested) {
                     change_state(S_RX_TXACKPREPARE);
                     atomic {
                        packetACK = call Malloc.getFreePacketBuffer();
                        temp_packetACK = packetACK;
                     }
                     if (temp_packetACK!=NULL) {
                        //send ACK
                        temp_packetACK->creator        = COMPONENT_MAC;
                        temp_packetACK->owner          = COMPONENT_MAC;
                        temp_packetACK->l1_txPower     = TX_POWER;
                        temp_packetACK->l1_channel     = FREQUENCYCHANNEL;
                        temp_packetACK->l2_retriesLeft = 1;
                        prependIEEE802154header(temp_packetACK,
                              IEEE154_TYPE_ACK,
                              IEEE154_SEC_NO_SECURITY,
                              received_ieee154_header.dsn,
                              NULL
                              );
                        atomic temp_error = call RadioSend.prepareSend(temp_packetACK);
                        if (temp_error!=SUCCESS) {
                           //abort
                           call OpenSerial.printError(COMPONENT_MAC,ERR_PREPARESEND_FAILED,
                                 (errorparameter_t)0,(errorparameter_t)2);
                           atomic call Malloc.freePacketBuffer(packetACK);
                           change_state(S_IDLE_LISTENING);
                        }
                     } else {
                        call OpenSerial.printError(COMPONENT_MAC,ERR_NO_FREE_PACKET_BUFFER,
                              (errorparameter_t)0,(errorparameter_t)0);
                        change_state(S_IDLE_LISTENING);
                        post taskReceive();
                        return;
                     }
                  } else {
                     post taskReceive();
                  }
               } else if (call PacketFunctions.isBroadcastMulticast(&received_ieee154_header.dest)==TRUE) {
                  //this packet is broadcast
                  if (temp_dataFrameReceived==NULL) {
                     atomic dataFrameReceived = msg;
                  } else {
                     call OpenSerial.printError(COMPONENT_MAC,ERR_BUSY_RECEIVING,
                           (errorparameter_t)1,
                           (errorparameter_t)0);
                     atomic call Malloc.freePacketBuffer(msg);
                  }
                  post taskReceive();
               } else {
                  call Malloc.freePacketBuffer(msg);
               }
            } else {
               //not data. I could be an ACK but I'm not in S_TX_RXACK state, so I discard
               call Malloc.freePacketBuffer(msg);
            }
            break;

         default:
            //this should never happen as error was caught above
            //do not go back to S_IDLE_LISTENING, just don't receive the packet and let the state machine be where it was
            call Malloc.freePacketBuffer(msg);
            call OpenSerial.printError(COMPONENT_MAC,ERR_WRONG_STATE_IN_RECEIVE,
                  (errorparameter_t)temp_state,
                  (errorparameter_t)0);
            break;
      }
   }

   task void taskReceive() {
      OpenQueueEntry_t*  temp_dataFrameReceived;
      atomic {
         temp_dataFrameReceived = dataFrameReceived;
      }
      if (temp_dataFrameReceived->length>0) {
         //packet contains payload destined to an upper layer
         call OpenReceiveToUpper.receive(temp_dataFrameReceived);
      } else {
         //packet contains no payload (KA)
         call Malloc.freePacketBuffer(temp_dataFrameReceived);
      }
      atomic dataFrameReceived = NULL;
   }

   /*------------------------------ misc ------------------------------------------------*/

   command error_t SoftwareInit.init() {
      change_state(S_IDLE_LISTENING);
      atomic dataFrameToSend = NULL;
      //WDT configuration
      WDTCTL = WDTPW + WDTHOLD;
      WDTCTL = WDTPW + WDTCNTCL + WDTSSEL;//run from ACLK, ~1s
      return SUCCESS;
   }

   //periodic timer used to reset WDT, to trigger transmit, and to trigger serial input/output
   event void timerDebug.fired() {
      //OpenQueueEntry_t*  temp_packet;
      call Port26.toggle();
      call OpenSerial.stop();
      //reset WDT
      atomic WDTCTL = WDTPW + WDTCNTCL + WDTSSEL;
      //trigger transmit
      post taskArmRandomBackoffTimer();
      //trigger serial input/output
      if ((call IDManager.getIsDAGroot())==TRUE) {
         call Leds.led2On();
      } else {
         call Leds.led2Off();
      }
      atomic serialInOutputMode = !serialInOutputMode;
      atomic if (serialInOutputMode) {
         call OpenSerial.startOutput();
      } else {
         call OpenSerial.startInput();
      }
   }

   task void taskArmRandomBackoffTimer() {
      call timerBackoff.startOneShot(MINBACKOFF+(call Random.rand16())%BACKOFFWINDOW);
   }

   void change_state(uint8_t newstate) {
      atomic state = newstate;
      switch (newstate) {
         case S_TX_TXDATAPREPARE:
         case S_TX_TXDATA:
         case S_RX_TXACKPREPARE:
         case S_RX_TXACK:
            atomic P3OUT |= 0x20;
            call Leds.led1On();
            break;
         case S_TX_TXDATAREADY:
         case S_TX_RXACK:
         case S_RX_TXACKREADY:
         case S_IDLE_LISTENING:
            atomic P3OUT &= ~0x20;
            call Leds.led1Off();
            break;
      }
   }

   /*----------- functions inherited from IEEE802154E but useless here ------------------*/

   async event void RadioControl.receivedNothing() {
      return;
   }
   async command timervalue_t GlobalTime.getGlobalSlotOffset() {
      return 0;
   }
   async command timervalue_t GlobalTime.getLocalTime() {
      return 0;
   }
   async command asn_t GlobalTime.getASN() {
      return 0;
   }
   async command bool GlobalSync.getIsSync() {
      return TRUE;
   }
   command void DebugPrint.print() {
      post taskDebugPrint();
   }
   task void taskDebugPrint() {
      return;
   }   
}
