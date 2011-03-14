
/*
 * BaseStation154C.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.ieee802154.tkn154
 *        Created on: 2010/05/10 
 * Last modification: 
 *            Author: aitorhh
 *     
 */


#include "AM.h"
#include "Serial.h"
#include "TKN154.h"
#include "app_sensors.h"
#include "UserButton.h"
#include "math.h"

module BaseStation154P {
  uses {
    interface Boot;
    interface MCPS_DATA;
    interface MLME_RESET;
    interface MLME_START;
    interface MLME_SET;
    interface MLME_GET;
    interface IEEE154Frame as Frame;
    interface IEEE154BeaconFrame as BeaconFrame;
    interface Leds;
    interface Packet;
    interface Timer<TMilli> as TimerTimeout;
#ifndef TKN154_BEACON_DISABLED
    interface IEEE154TxBeaconPayload;
#endif
    
    interface SplitControl as SerialControl;
    interface AMSend as UartSend;
    interface Receive as UartReceive;
    interface AMPacket as UartAMPacket; 
    
    //To change the rate
	interface Notify<button_state_t>;   
	interface Get<button_state_t>;
  }
}
  implementation {

   message_t m_frame;
   am_id_t id = AM_ENCMSG;
   EncMsg2Sensors *m_sensor;
	uint16_t rate = DEFAULT_RATE;

   uint8_t m_payloadLen = sizeof(EncMsg);
   uint8_t m_payloadLen2 = sizeof(EncMsg2Sensors);
   ieee154_PANDescriptor_t m_PANDescriptor;
   
   message_t  uartQueueBufs[UART_QUEUE_LEN];
   message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
   uint8_t    uartIn, uartOut;
   bool       uartBusy, uartFull;
   
   uint16_t ACTUATOR_NODE = 0x03;

   task void uartSendTask();
   task void sendRatePacket();
   task void sendPacket();
   void setAddressingFields(uint16_t address);
   
   
   bool gotTheta, gotXc;


   event void Boot.booted() {
     uint8_t i;
     	
     	 
      for (i = 0; i < UART_QUEUE_LEN; i++)
        uartQueue[i] = &uartQueueBufs[i];
     
      uartIn = uartOut = 0;
      uartBusy = FALSE;
      uartFull = TRUE;
      
      gotTheta = FALSE;
      gotXc = FALSE;

      call SerialControl.start();
      
      call MLME_RESET.request(TRUE);
      call Notify.enable();
      

      m_sensor = (EncMsg2Sensors*)(call Packet.getPayload(&m_frame,m_payloadLen2));
      m_sensor->data_xc = 0;
      m_sensor->data_theta = 0;
      
   }
   
   
   event void TimerTimeout.fired() {
		
	atomic {
          if (!uartFull && ((gotTheta && !gotXc)||(!gotTheta && gotXc)))
            {
              
              //post sendPacket();
              
              uartQueue[uartIn] = &m_frame;
              uartIn = (uartIn + 1) % UART_QUEUE_LEN;
              
              gotTheta = FALSE;
              gotXc = FALSE;
            
              if (!uartBusy)
                {
                  post uartSendTask();
                  uartBusy = TRUE;
                }
            }
        }
		
	}
   
   event void MLME_RESET.confirm(ieee154_status_t status)
     {
	   if (status != IEEE154_SUCCESS)
	      return;
	    call MLME_SET.phyTransmitPower(TX_POWER_COORDINATOR);
	    call MLME_SET.macShortAddress(TOS_NODE_ID);
	    call MLME_SET.macAssociationPermit(FALSE);
	    call MLME_SET.macRxOnWhenIdle(TRUE);
	    
	    call MLME_START.request(
	                          PAN_ID,               // PANId
	                          RADIO_CHANNEL,        // LogicalChannel
	                          0,                    // ChannelPage,
	                          0,                    // StartTime,
	                          BEACON_ORDER,         // BeaconOrder
	                          SUPERFRAME_ORDER,     // SuperframeOrder
	                          TRUE,                 // PANCoordinator
	                          FALSE,                // BatteryLifeExtension
	                          FALSE,                // CoordRealignment
	                          0,                    // CoordRealignSecurity,
	                          0                     // BeaconSecurity
	                        );
     
     }

   void setAddressingFields(uint16_t address)
   {
     ieee154_address_t deviceShortAddress;
     deviceShortAddress.shortAddress = address; // destination

     call Frame.setAddressingFields(
         &m_frame,                
         ADDR_MODE_SHORT_ADDRESS,        // SrcAddrMode,
         ADDR_MODE_SHORT_ADDRESS,        // DstAddrMode,
         PAN_ID,                         // DstPANId,
         &deviceShortAddress,            // DstAddr,
         NULL                            // security
         );

   }
   
/************************************************************************************
 *  DEVICE -----15.4----- BASE STATION -------EncMsg------- PC
 ************************************************************************************/
   
   //Store the new message in the UART input buffer
   event message_t* MCPS_DATA.indication (message_t* frame)
    {
     ieee154_address_t deviceAddr;
     EncMsg *m_device;
     message_t* ret = frame;

     // We assume that the packets with length equal to EncMsg
     // are a EncMsg
     if (call Frame.getPayloadLength(frame) == m_payloadLen &&
         call Frame.getFrameType(frame) == FRAMETYPE_DATA){
         
         call Frame.getSrcAddr(frame, &deviceAddr);
         if(deviceAddr.shortAddress == 1) {
         	m_device = (EncMsg*)(call Packet.getPayload(frame,m_payloadLen));
         	m_sensor->data_xc = m_device->data;
         	gotXc = TRUE;
         	
         	call TimerTimeout.startOneShot(5);
         	
         }
         if(deviceAddr.shortAddress == 2) {
         	m_device = (EncMsg*)(call Packet.getPayload(frame,m_payloadLen));
         	m_sensor->data_theta = m_device->data;
         	gotTheta = TRUE;
         	
         	call TimerTimeout.startOneShot(5);
         	
         }

       atomic {
          if (!uartFull && gotTheta && gotXc)
            {
              
              //post sendPacket();
              
              ret = uartQueue[uartIn];
              uartQueue[uartIn] = &m_frame;
              uartIn = (uartIn + 1) % UART_QUEUE_LEN;
              
              gotTheta = FALSE;
              gotXc = FALSE;
            
              if (!uartBusy)
                {
                  post uartSendTask();
                  uartBusy = TRUE;
                }
            }
          else{
           //call Leds.led0Toggle();
          }
        }
     }
      return frame;
    }
   
   //Send to the serial port
   task void uartSendTask() {
     message_t* msg;

     atomic {
       if (uartIn == uartOut && !uartFull) {
         uartBusy = FALSE;
         return;
       }
     }

     msg = uartQueue[uartOut];
     
     if (call UartSend.send(AM_BROADCAST_ADDR, uartQueue[uartOut], m_payloadLen2 ) == SUCCESS) {
       //call Leds.led1Toggle();
     }else  {
       call Leds.led0Toggle();
       post uartSendTask();
     }
   }
   
   event void MLME_START.confirm(ieee154_status_t status) {}
      
   event void SerialControl.startDone(error_t error) {

     if (error == SUCCESS) {
       uartFull = FALSE;
     }
   }

   event void SerialControl.stopDone(error_t error) {}

   event void UartSend.sendDone(message_t* msg, error_t error) {
     if (error != SUCCESS){
         call Leds.led0Toggle();
     }else
       atomic
         if (msg == uartQueue[uartOut])
           {
             if (++uartOut >= UART_QUEUE_LEN)
               uartOut = 0;
             if (uartFull)
               uartFull = FALSE;
           }
     post uartSendTask();
   }
   
   /************************************************************************************
    *  PC -----EncMsg----- BASE SATITON -------15.4------- DEVICE
    ************************************************************************************/
   event message_t *UartReceive.receive(message_t *msg,
                                                     void *payload,
                                                     uint8_t len) {
      message_t* ret = msg;
      
      call Leds.led0Toggle();

      setAddressingFields(BROADCAST_DESTINATION);

      if (call MCPS_DATA.request  ( &m_frame,                   // frame,
                                    len,                   // payloadLength,
                                    0,                    // msduHandle,
                                    TX_OPTIONS_ACK        // TxOptions,
                                    ) != IEEE154_SUCCESS){
           call Leds.led0On(); //fail!
      }else{ 
        call Leds.led0Off();
        call Leds.led1Toggle();
      }
      return ret;
    }
    
    task void sendPacket() {
		
		
      setAddressingFields(ACTUATOR_NODE);
		if (call MCPS_DATA.request (
						&m_frame, // frame,
						sizeof(EncMsg2Sensors), // payloadLength,
						0, // msduHandle,
						TX_OPTIONS_ACK // TxOptions,
				) != IEEE154_SUCCESS) {
			call Leds.led0Toggle(); //fail!
		}
		else {
			call Leds.led1Toggle();
		}
	}

   event void MCPS_DATA.confirm    (
                           message_t *msg,
                           uint8_t msduHandle,
                           ieee154_status_t status,
                           uint32_t timestamp
                         ){
                         
                         if (status != IEEE154_SUCCESS) call Leds.led0Toggle();
                         
                         }
   /************************************************************************************
    *  BEACON FUNCTIONS
    ************************************************************************************/
#ifndef TKN154_BEACON_DISABLED
   event void IEEE154TxBeaconPayload.aboutToTransmit() { }

   event void IEEE154TxBeaconPayload.setBeaconPayloadDone(void *beaconPayload, uint8_t length) { }

   event void IEEE154TxBeaconPayload.modifyBeaconPayloadDone(uint8_t offset, void *buffer, uint8_t bufferLength) { }

   event void IEEE154TxBeaconPayload.beaconTransmitted() 
   {
     ieee154_macBSN_t beaconSequenceNumber = call MLME_GET.macBSN();
     if (beaconSequenceNumber & 1)
       call Leds.led2On();
     else
       call Leds.led2Off();
   }  
#endif
   
  event void Notify.notify( button_state_t state ) {
   		if ( state == BUTTON_PRESSED ) {
   			atomic{
   				if(rate < 100){
   					rate+= 10;
   				} else {
   					rate = 10;
   				}
   			}
   			call Leds.led2On();
   		} else if ( state == BUTTON_RELEASED ) {
   			call Leds.led2Off();
   		}
   		post sendRatePacket();
   	}
   
  task void sendRatePacket(){
	  RateMsg* btrpkt = (RateMsg*) call Packet.getPayload(&m_frame, sizeof(RateMsg));
	  btrpkt -> rate = rate;
	  if (call MCPS_DATA.request  ( &m_frame,                   // frame,
			  sizeof(RateMsg),                   // payloadLength,
              0,                    // msduHandle,
              TX_OPTIONS_ACK        // TxOptions,
              ) != IEEE154_SUCCESS){
			  call Leds.led0On(); //fail!
	  }
  }
}
