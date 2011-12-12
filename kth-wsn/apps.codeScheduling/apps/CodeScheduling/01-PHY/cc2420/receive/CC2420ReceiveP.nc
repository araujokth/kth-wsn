#include "LATIN.h"
#include "IEEE802154.h"

#include "printf.h"

module CC2420ReceiveP @safe() {

   provides interface Init;
   provides interface StdAsyncControl;
   provides interface CC2420sfd;
   provides interface RadioReceive;
   provides interface CC2420ReceivedNothing;

//   uses interface OpenSerial;
   uses interface Malloc;

   uses interface GeneralIO as CSN;
   uses interface GeneralIO as FIFO;
   uses interface GeneralIO as FIFOP;
   uses interface GpioInterrupt as InterruptFIFOP;

   uses interface Resource as SpiResource;
   uses interface CC2420Fifo as RXFIFO;
   uses interface CC2420Strobe as SFLUSHRX;

   uses interface HplMsp430GeneralIO as Port34;  //private, general debug
}

implementation {

   /*-------------------------------- variables -----------------------------------------*/

   typedef enum {
      S_STOPPED,
      S_STARTED,
      S_RX_LENGTH,
      S_RX_PAYLOAD,
   } cc2420_receive_state_t;

   enum {
      RXFIFO_SIZE          = 128,
      TIMESTAMP_QUEUE_SIZE = 8,
      SACK_HEADER_LENGTH   = 7,
   };

   uint32_t timestamp_queue[ TIMESTAMP_QUEUE_SIZE ];
   uint8_t timestamp_head;
   uint8_t timestamp_size;
   uint8_t missed_packets;        //number of packets we missed (*not* lost) because we were doing something else
   bool receivingPacket;          //TRUE if we are receiving a valid packet into the stack
   norace uint8_t free_bytes_left_in_fifo;
   norace OpenQueueEntry_t* receptionBuffer;
   cc2420_receive_state_t state;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskReceiveDone();

   void reset();
   void beginReceive();
   void receiveLengthByte();
   void waitForNextPacket();

   /*-------------------------------- reception sequence --------------------------------*/

   async command void CC2420sfd.sfd( uint32_t time ) {
      if ( timestamp_size < TIMESTAMP_QUEUE_SIZE ) {
         uint8_t tail =  ( ( timestamp_head + timestamp_size ) % TIMESTAMP_QUEUE_SIZE );
         timestamp_queue[ tail ] = time;
         timestamp_size++;
      }
   }
   async command void CC2420sfd.sfd_dropped() {
      if ( timestamp_size ) {
         timestamp_size--;
      }
   }

   async event void InterruptFIFOP.fired() {
      if ( state == S_STARTED ) {
         beginReceive();
      } else {
         missed_packets++;
      }
   }

   void beginReceive() { 
      state = S_RX_LENGTH;
      atomic receivingPacket = TRUE;
      if (call SpiResource.isOwner()) {
         receiveLengthByte();
      } else if (call SpiResource.immediateRequest() == SUCCESS) {
         receiveLengthByte();
      } else {
         call SpiResource.request();
      }
   }

   event void SpiResource.granted() {
      receiveLengthByte();
   }

   void receiveLengthByte() {
      call CSN.clr();
      call RXFIFO.beginRead((uint8_t*)(receptionBuffer->payload),1);
   }

   async event void RXFIFO.readDone( uint8_t* rx_buf, uint8_t rx_len, error_t error ) {
      uint8_t* buf = (uint8_t*)(receptionBuffer->payload);
      receptionBuffer->length = buf[ 0 ]+1;//+1 for the length byte itself

      switch( state ) {

         case S_RX_LENGTH:
            state = S_RX_PAYLOAD;
            if (receptionBuffer->length>128) {                                      //much too large
//               call OpenSerial.printError(COMPONENT_CC2420RECEIVE,ERR_PACKET_TOO_LARGE,
//                     (errorparameter_t)receptionBuffer->length,
//                     (errorparameter_t)0);
               signal CC2420ReceivedNothing.receivedNothing();
               reset();//do last so has time to fill up
            } else if (receptionBuffer->length>free_bytes_left_in_fifo) {           //still too large
//               call OpenSerial.printError(COMPONENT_CC2420RECEIVE,ERR_PACKET_LARGER_BYTES_LEFT,
//                     (errorparameter_t)receptionBuffer->length,
//                     (errorparameter_t)free_bytes_left_in_fifo);
               signal CC2420ReceivedNothing.receivedNothing();
               reset();
            } else if (receptionBuffer->length==1) {                                //empty PHY payload
//               call OpenSerial.printError(COMPONENT_CC2420RECEIVE,ERR_PACKET_EMPTY,
//                     (errorparameter_t)receptionBuffer->length,
//                     (errorparameter_t)0);
               atomic receivingPacket = FALSE;
               call CSN.set();
               call SpiResource.release();
               waitForNextPacket();
            } else if (receptionBuffer->length<6) {                                 //still too short
//               call OpenSerial.printError(COMPONENT_CC2420RECEIVE,ERR_PACKET_TOO_SHORT,
//                     (errorparameter_t)receptionBuffer->length,
//                     (errorparameter_t)0);
               signal CC2420ReceivedNothing.receivedNothing();
               reset();
            } else {                                                                //acceptable length
               if ( !call FIFO.get() && !call FIFOP.get() ) {
                  free_bytes_left_in_fifo -= receptionBuffer->length;
               }
               state = S_RX_PAYLOAD;
               call RXFIFO.continueRead(buf+1,receptionBuffer->length-1);//don't read the length byte again
            }
            break;

         case S_RX_PAYLOAD:
            call CSN.set();
            if(!missed_packets) {
            // Release the SPI only if there are no more frames to download
            call SpiResource.release();
            }
            //new packet is buffered up, or we don't have timestamp in fifo
            if ((missed_packets && call FIFO.get()) || !call FIFOP.get() || !timestamp_size) {
               receptionBuffer->l1_rxTimestamp = INVALID_TIMESTAMP;
            } else {
               receptionBuffer->l1_rxTimestamp = timestamp_queue[timestamp_head];
               timestamp_head = ( timestamp_head + 1 ) % TIMESTAMP_QUEUE_SIZE;
               timestamp_size--;
            }
            if ( ( buf[ receptionBuffer->length-1 ] >> 7 ) && rx_buf ) { //checks the CRC
               post taskReceiveDone();
               return;
            } else {
//               call OpenSerial.printError(COMPONENT_CC2420RECEIVE,ERR_INCORRECT_CRC,
//                     (errorparameter_t)0,
//                     (errorparameter_t)0);
               signal CC2420ReceivedNothing.receivedNothing();       //CRC was incorrect
               reset();
            }
            waitForNextPacket();
            break;

         default:
            atomic receivingPacket = FALSE;
            call CSN.set();
            call SpiResource.release();
            break;
      }
   }

   task void taskReceiveDone() {
      uint8_t* buf = (uint8_t*)receptionBuffer->payload;
      uint8_t length = buf[0]+1;
      OpenQueueEntry_t* new_receptionBuffer;
      //copy in Rx details from packet body
      receptionBuffer->l1_crc  = buf[length-1] >> 7;
      receptionBuffer->l1_lqi  = buf[length-1] & 0x7f;
      receptionBuffer->l1_rssi = buf[length-2];
      //get a new reception buffer
      new_receptionBuffer   = call Malloc.getFreePacketBuffer();
      if (new_receptionBuffer==NULL) {
//         call OpenSerial.printError(COMPONENT_CC2420RECEIVE,ERR_NO_FREE_PACKET_BUFFER,
//               (errorparameter_t)0,(errorparameter_t)0);
         return;
      }
      new_receptionBuffer->creator = COMPONENT_CC2420RECEIVE;
      new_receptionBuffer->owner   = COMPONENT_CC2420RECEIVE;
      new_receptionBuffer->payload = new_receptionBuffer->packet;
      new_receptionBuffer->length  = 0;
      //pass up
      signal RadioReceive.receive(receptionBuffer);
      receptionBuffer = new_receptionBuffer;
      //wrap up
      atomic receivingPacket = FALSE;
      waitForNextPacket();
   }

   void waitForNextPacket() {
      atomic {
         if ( state == S_STOPPED ) {
            call SpiResource.release();
            return;
         }
         atomic receivingPacket = FALSE;
         /*
          * The FIFOP pin here is high when there are 0 bytes in the RX FIFO
          * and goes low as soon as there are bytes in the RX FIFO.  The pin
          * is inverted from what the datasheet says, and its threshold is 127.
          * Whenever the FIFOP line goes low, as you can see from the interrupt
          * handler elsewhere in this module, it means we received a new packet.
          * If the line stays low without generating an interrupt, that means
          * there's still more data to be received.
          */
         if ( ( missed_packets && call FIFO.get() ) || !call FIFOP.get() ) {
            // A new packet is buffered up
            if ( missed_packets ) {
               missed_packets--;
            }
            beginReceive();
         } else {
            // wait for the next packet
            state = S_STARTED;
            missed_packets = 0;
            call SpiResource.release();
         }
      }
   }

   /*-------------------------------- misc ----------------------------------------------*/

   command error_t Init.init() {
      receptionBuffer = call Malloc.getFreePacketBuffer();
      if (receptionBuffer==NULL) {
//         call OpenSerial.printError(COMPONENT_CC2420RECEIVE,ERR_NO_FREE_PACKET_BUFFER,
//               (errorparameter_t)1,
//               (errorparameter_t)0);
      }
      receptionBuffer->creator = COMPONENT_CC2420RECEIVE;
      receptionBuffer->owner   = COMPONENT_CC2420RECEIVE;
      receptionBuffer->payload = receptionBuffer->packet;
      receptionBuffer->length  = 0;
      return SUCCESS;
   }

   async command error_t StdAsyncControl.start() {
      atomic {
         reset();
         state = S_STARTED;
         atomic receivingPacket = FALSE;
         /* Note:
            We use the falling edge because the FIFOP polarity is reversed. 
            This is done in CC2420Power.startOscillator from CC2420ControlP.nc.
            */
         call InterruptFIFOP.enableFallingEdge();
      }
      return SUCCESS;
   }

   void reset() {
      //reset state
      free_bytes_left_in_fifo = RXFIFO_SIZE;
      atomic receivingPacket = FALSE;
      timestamp_head = 0;
      timestamp_size = 0;
      missed_packets = 0;
      //flush
      call CSN.set();
      call CSN.clr();
      call SFLUSHRX.strobe();
      call SFLUSHRX.strobe();
      call CSN.set();
      call SpiResource.release();
      //go on
      waitForNextPacket();
   }

   async command error_t StdAsyncControl.stop() {
      atomic {
         state = S_STOPPED;
         reset();
         call CSN.set();
         call InterruptFIFOP.disable();
      }
      return SUCCESS;
   } 

   async event void RXFIFO.writeDone( uint8_t* tx_buf, uint8_t tx_len, error_t error ) {
   }
}

