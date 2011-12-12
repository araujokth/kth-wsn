#include "LATIN.h"
#include "CC2420.h"
#include "crc.h"
#include "message.h"

module CC2420TransmitP @safe() {

   provides interface Init;
   provides interface StdAsyncControl;
   provides interface CC2420Transmit;

   uses interface Alarm<T32khz,uint32_t> as AlarmWatchdog;
//   uses interface OpenSerial;

   uses interface GpioCapture as CaptureSFD;
   uses interface GeneralIO as CCA;
   uses interface GeneralIO as CSN;
   uses interface GeneralIO as SFD;

   uses interface Resource as SpiResource;
   uses interface ChipSpiResource;
   uses interface CC2420Fifo     as TXFIFO;
   uses interface CC2420Ram      as TXFIFO_RAM;
   uses interface CC2420Register as TXCTRL;
   uses interface CC2420Strobe   as SNOP;
   uses interface CC2420Strobe   as STXON;
   uses interface CC2420Strobe   as SFLUSHTX;
   uses interface CC2420Register as MDMCTRL1;

   uses interface CC2420sfd;

   uses interface GlobalTime;
   uses interface PinDebug;
}

implementation {

   /*-------------------------------- variables -----------------------------------------*/

   typedef enum {
      S_STOPPED    = 0,
      S_READY      = 1,
      S_LOAD       = 2,
      S_LOADED     = 3,
      S_SFD        = 4,
      S_EFD        = 5,
   } cc2420_transmit_state_t;

   enum {
      // how many jiffies after STXON.strobe to receive SFD interrupt before assuming something is wrong
      CC2420_ABORT_PERIOD_SFD  =  32, // 1ms
      // how many jiffies after TXFIFO.write() to receive TXFIFO.writeDone before assuming something is wrong
      CC2420_ABORT_PERIOD_LOAD = 320, //10ms
   };

   norace OpenQueueEntry_t* m_msg;
   norace bool m_cca;
   norace uint8_t m_tx_power;
   cc2420_transmit_state_t state = S_STOPPED;
   bool m_receiving = FALSE;
   uint16_t m_prev_time;
   bool sfdHigh; //byte reception/transmission indicator

   /*-------------------------------- prototypes ----------------------------------------*/

   void loadTXFIFO();
   void send();
   void signalDone(error_t err);
   void reset();

   /*-------------------------------- load a packet -------------------------------------*/

   async command error_t CC2420Transmit.loadPacket(OpenQueueEntry_t* p_msg) {
      //p_msg->owner = COMPONENT_CC2420TRANSMIT;//don't own to enable L2 retransmits
      atomic {
         if ( state != S_READY ) {
            reset();
//            call OpenSerial.printError(COMPONENT_CC2420TRANSMIT,ERR_WRONG_STATE_LOADPACKET,
//                  (errorparameter_t)state,
//                  (errorparameter_t)0);
            return FAIL;
         }
         state = S_LOAD;
         m_msg = p_msg;
      }
      if ( (call SpiResource.immediateRequest()) == SUCCESS ) {
         loadTXFIFO();
      } else {
         call SpiResource.request();
      }
      return SUCCESS;
   }

   event void SpiResource.granted() {
      uint8_t cur_state;
      atomic {
         cur_state = state;
      }
      switch( cur_state ) {
         case S_LOAD:
            loadTXFIFO();
            break;
         case S_LOADED:
            send();
            break;
         default:
            call SpiResource.release();
            break;
      }
   }

   void loadTXFIFO() {
      uint8_t tx_power = m_msg->l1_txPower;
      if ( !tx_power ) {
         tx_power = CC2420_DEF_RFPOWER;
      }
      call CSN.clr();
      if ( m_tx_power != tx_power ) {
         call TXCTRL.write( ( 2 << CC2420_TXCTRL_TXMIXBUF_CUR ) |
               ( 3 << CC2420_TXCTRL_PA_CURRENT ) |
               ( 1 << CC2420_TXCTRL_RESERVED ) |
               ( (tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL ) );
      }
      m_tx_power = tx_power;
      call AlarmWatchdog.start(CC2420_ABORT_PERIOD_LOAD);
      call TXFIFO.write((uint8_t*)m_msg->payload,m_msg->length);
   }

   async event void AlarmWatchdog.fired() {
      call AlarmWatchdog.stop();
      atomic {
         switch(state) {
            case S_LOAD:
               //no TXFIFO.writeDone() within CC2420_ABORT_PERIOD_SFD jiffies after TXFIFO.write()
               //no break!
            case S_SFD:
               //no SFD interrupt within CC2420_ABORT_PERIOD_LOAD jiffies after send()
               reset();
               signalDone(FAIL);
               break;
            default:
               break;
         }
      }
   }

   async event void TXFIFO.writeDone(uint8_t* tx_buf, uint8_t tx_len, error_t error) {
      call AlarmWatchdog.stop();
      if (error==SUCCESS) {
         atomic state = S_LOADED;
      } else {
         reset();
//         call OpenSerial.printError(COMPONENT_CC2420DRIVER,ERR_TXFIFOWRITEDONE_FAILED,
//               (errorparameter_t)error,
//               (errorparameter_t)0);
      }
      call CSN.set();
      call SpiResource.release();
      signal CC2420Transmit.loadPacketDone(error);
   }

   /*-------------------------------- send a packet -------------------------------------*/

   async command error_t CC2420Transmit.sendNow(bool useCca) {
      m_cca = useCca;
      atomic {
         if ( state != S_LOADED ) {
            reset();
//            call OpenSerial.printError(COMPONENT_CC2420TRANSMIT,ERR_WRONG_STATE_SENDNOW,
//                  (errorparameter_t)state,
//                  (errorparameter_t)0);
            return FAIL;
         }
      }
      if ( (call SpiResource.immediateRequest()) == SUCCESS ) {
         send();
      } else {
         call SpiResource.request();
      }
      return SUCCESS;
   }

   void send() {
      atomic {
         call CSN.clr();
         call STXON.strobe();
         call CSN.set();
      }
      atomic state = S_SFD;
      call AlarmWatchdog.start(CC2420_ABORT_PERIOD_SFD);
//      call PinDebug.ADC1toggle();
   }

   /**
    * The CaptureSFD event is an interrupt from the capture pin
    * which is connected to timing circuitry and timer modules.  This
    * type of interrupt allows us to see what time (being some relative value)
    * the event occurred, and lets us accurately timestamp our packets.  This
    * allows higher levels in our system to synchronize with other nodes.
    *
    * Because the SFD events can occur so quickly, and the interrupts go
    * for both transmitting or receiving, we set up the interrupt but check the SFD pin to
    * determine if that interrupt condition has already been met - meaning,
    * we should fall through and continue executing code where that interrupt
    * would have picked up and executed had our microcontroller been fast enough.
    */
   async event void CaptureSFD.captured( uint16_t time ) {
      uint32_t globalSlotOffset;
      uint8_t sfd_state = 0;
      bool falling_through = FALSE;
      atomic {
         globalSlotOffset = (nxle_uint32_t)(call GlobalTime.getGlobalSlotOffset());
         switch( state ) {
            case S_SFD: //applies only to transmission (no S_SFD/S_EFD states during reception)
               call AlarmWatchdog.stop();
               call SpiResource.release();
               state = S_EFD;
               sfdHigh = TRUE;
               // in case we got stuck in the receive SFD interrupts, we can reset
               // the state here since we know that we are not receiving anymore
               m_receiving = FALSE;
               call CaptureSFD.captureFallingEdge();

               if ( call SFD.get() ) {
                  //the packet is still being transmitted, we break here and
                  //will resume when SFD goes low (i.e. when the packet is fully transmitted)
                  break;
               }
               //we fall through because it's already past the end of the packet: we
               //would have gotten an interrupt had our uC been fast enough

            case S_EFD: //applies only to transmission (no S_SFD/S_EFD states during reception)
               sfdHigh = FALSE;
               signalDone(SUCCESS);
               if ( !call SFD.get() ) {
                  //the SFD is low, so we can stop processing
                  break;
               }
               //the SFD is high again, meaning we started receiving a packet. We fall through.

            default: //we are in reception mode because SFD interrupt while not in S_SFD or S_EFD
               if ( m_receiving==FALSE && sfdHigh == FALSE ) { //we are at the start of the packet
                  sfdHigh     = TRUE;
                  call CaptureSFD.captureFallingEdge();
                  sfd_state = call SFD.get(); //save the SFD pin status to determine whether we fall through
                  falling_through = TRUE;
                  m_receiving = TRUE;
                  m_prev_time = time;
                  call CC2420sfd.sfd(globalSlotOffset);//this goes to CC2420Receive
                  if ( call SFD.get() ) {
                     //the packet is still being received, we stop here and
                     //will resume when SFD goes low (i.e. when the packet is fully received)
                     return;
                  }
                  //we fall through because it's already past the end of the packet: we consider we would have 
                  //gotten and interrupt had our uC be fast enough
               }
               if ( sfdHigh == TRUE ) { //we fell through or received a second interrupt at the end of the packet
                  sfdHigh = FALSE;
                  call CaptureSFD.captureRisingEdge();
                  m_receiving = FALSE;
                  /* if sfd_state is 1, then we fell through, but at the time of
                   * saving the time stamp the SFD was still high. Thus, the timestamp
                   * is valid.
                   * if the sfd_state is 0, then either we fell through and SFD
                   * was low while we saved the time stamp, or we didn't fall through.
                   * Thus, we check for the time between the two interrupts.
                   * Why 5 tics? A 5+1B ACK message takes 192us to arrive; 6 tics take 183us, a little less.
                   * There can hence be no packet which takes less than 6 tics
                   *
                   if ((sfd_state == 0) && (time - m_prev_time < 6) ) {
                   call CC2420sfd.sfd_dropped();
                   call OpenSerial.printError(COMPONENT_CC2420TRANSMIT,ERR_SFD_DROPPED,
                   (errorparameter_t)time,
                   (errorparameter_t)m_prev_time);
                  //poipoi if we read falling_through==TRUE, something is wrong. 
                  call OpenSerial.printError(COMPONENT_CC2420TRANSMIT,ERR_POIPOI,
                  (errorparameter_t)falling_through,
                  (errorparameter_t)0);
                  if (&(m_msg->packet)) {
                  m_msg->l1_rxTimestamp = INVALID_TIMESTAMP; 
                  }
                  }*///poipoi uncomment this! testing what sfd_dropped is actually used for
                  break;
               }
         }
      }
   }

   /**
    * The backoff timer is used to timeout the wait for an SFD interrupt when
    * we should have gotten one.
    */

   void signalDone(error_t err) {
      call CaptureSFD.captureRisingEdge();
      atomic state = S_READY;
      signal CC2420Transmit.sendNowDone(err);
   }

   /*-------------------------------- misc ----------------------------------------------*/

   void reset() {
      call CSN.clr();
      call SFLUSHTX.strobe();
      call CSN.set();
      call SpiResource.release();
      atomic state = S_READY;
   }

   command error_t Init.init() {
      call CCA.makeInput();
      call CSN.makeOutput();
      call SFD.makeInput();
      return SUCCESS;
   }

   async command error_t StdAsyncControl.start() {
      atomic {
         reset();
         call CaptureSFD.captureRisingEdge();
         state         = S_READY;
         m_receiving   = FALSE;
         m_tx_power    = 0;
      }
      return SUCCESS;
   }

   async command error_t StdAsyncControl.stop() {
      atomic {
         state = S_STOPPED;
         call AlarmWatchdog.stop();
         call CaptureSFD.disable();
         call SpiResource.release();
         call CSN.set();
      }
      return SUCCESS;
   }

   async event void ChipSpiResource.releasing() {
   }

   async event void TXFIFO.readDone( uint8_t* tx_buf, uint8_t tx_len, error_t error ) {
   }
}
