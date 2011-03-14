
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
    interface Timer<TSymbolIEEE802154> as TimerBeacon;
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
   EncMsg2SensorsAct *m_sensor;
	uint16_t rate = DEFAULT_RATE;

	uint32_t beacontime;

   uint8_t m_payloadLen = sizeof(EncMsg);
   uint8_t m_payloadLen2 = sizeof(EncMsg2SensorsAct);
   ieee154_PANDescriptor_t m_PANDescriptor;
   
   message_t  uartQueueBufs[UART_QUEUE_LEN];
   message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
   uint8_t    uartIn, uartOut;
   bool       uartBusy, uartFull;

   task void uartSendTask();
   task void sendRatePacket();
   void setAddressingFields(uint16_t address);
   task void calcAndAct();


   	/* LQG and Kalman filter */

   	nx_float L[4], K[4][2], A_tau[4][4], A_htau[4][4], B_tau[4], B_htau[4], C[2][4];
    nx_float x[4], u;
    uint8_t i,j;
    uint32_t bi_bo4;
   
   bool gotTheta, gotXc;

   event void TimerBeacon.fired() {}
   event void Boot.booted() {

     atomic {
         			ADC12CTL0 = REF2_5V +REFON;
         			DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;
     		}

     bi_bo4 = 15360;

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
      

      m_sensor = (EncMsg2SensorsAct*)(call Packet.getPayload(&m_frame,m_payloadLen2));
      m_sensor->data_xc = 0;
      m_sensor->data_theta = 0;
      m_sensor->u = 0;
      
      	  	x[0]=0;
      		x[1]=0;
      		x[2]=0;
      		x[3]=0;

      		u=0;

      		// LQ Gain
      		L[0]=-27.9205;
      		L[1]=133.5942;
      		L[2]=-34.225;
      		L[3]=19.522;

      		// Wireless
      		K[0][0]=1.0000;
      		K[0][1]=0.0000;
      		K[1][0]=0.0000;
      		K[1][1]=1.0000;
      		K[2][0]=0.0000;
      		K[2][1]=0.0011;
      		K[3][0]=0.0010;
      		K[3][1]=0.5869;

      		A_tau[0][0]=1.00;
      		A_tau[0][1]=0.000847131340963129;
      		A_tau[0][2]=0.0240625846979718;
      		A_tau[0][3]=0.00000605644181647328;
      		A_tau[1][0]=0.00;
      		A_tau[1][1]=1.01213514891668;
      		A_tau[1][2]=-0.0136946676192767;
      		A_tau[1][3]=0.0300828963670533;
      		A_tau[2][0]=0.00;
      		A_tau[2][1]=0.0525780049495745;
      		A_tau[2][2]=0.632064732925621;
      		A_tau[2][3]=0.000678154224755660;
      		A_tau[3][0]=0.00;
      		A_tau[3][1]=0.801112383040290;
      		A_tau[3][2]=-0.849971331178926;
      		A_tau[3][3]=1.00955363381334;

      		A_htau[0][0]=1.00;
      		A_htau[0][1]=0.00000431194071106424;
      		A_htau[0][2]=0.00196974549031381;
      		A_htau[0][3]=-0.0000000109757913270413;
      		A_htau[1][0]=0.00;
      		A_htau[1][1]=1.00005513915775;
      		A_htau[1][2]=-0.0000697066949740868;
      		A_htau[1][3]=0.00199985911172103;
      		A_htau[2][0]=0.00;
      		A_htau[2][1]=0.00428999948243722;
      		A_htau[2][2]=0.969898932861259;
      		A_htau[2][3]=-0.00000947530087570435;
      		A_htau[3][0]=0.00;
      		A_htau[3][1]=0.0550876802739018;
      		A_htau[3][2]=-0.0693519884267960;
      		A_htau[3][3]=0.999877639643667;

      		B_tau[0]=0.000851066179853329;
      		B_tau[1]=0.00196301316566279;
      		B_tau[2]=0.0527396598540434;
      		B_tau[3]=0.121836225123766;

      		B_htau[0]=0.00000433666647868565;
      		B_htau[1]=0.00000999183985705511;
      		B_htau[2]=0.00431467209951842;
      		B_htau[3]=0.00994099639333767;

      		C[0][0]=1;
      		C[0][1]=0;
      		C[0][2]=0;
      		C[0][3]=0;
      		C[1][0]=0;
      		C[1][1]=1;
      		C[1][2]=0;
      		C[1][3]=0;

   }
   
	task void calcAndAct() {

		nx_float y[2], x_bar[4], res_0[4];


		y[0] = ((int16_t) m_sensor->data_xc) * 9.2E-5;
		y[1] = (((int16_t) m_sensor->data_theta) / 1024.0) * 3.1415;

		//x_bar= x_old_mote+ K*(y-C*x_old_mote);
		memset(x_bar, 0, sizeof(x_bar));
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<2;i++) for(j=0;j<4;j++) res_0[i] += C[i][j] * x[j];
		for(i=0;i<2;i++) res_0[i] = y[i] - res_0[i];
		for(i=0;i<4;i++) for(j=0;j<2;j++) x_bar[i] += K[i][j] * res_0[j];
		for(i=0;i<4;i++) x_bar[i] += x[i];

		//x_bar= A_tau*x_bar- B_tau*u_old;
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<4;i++) for(j=0;j<4;j++) res_0[i] += A_tau[i][j] * x_bar[j];
		for(i=0;i<4;i++) x_bar[i] = res_0[i] - B_tau[i] * u;

		//u_mote= L*x_bar;
		u = 0;
		for(i=0;i<4;i++) u += L[i] * x_bar[i];

		//x_mote= A_htau*x_bar- B_htau*u_old;
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<4;i++) for(j=0;j<4;j++) res_0[i] += A_htau[i][j] * x_bar[j];
		for(i=0;i<4;i++) x[i] = res_0[i] - B_htau[i] * u;

		if(u > 10) u = 10.0;
		if(u < -10) u = -10.0;
		u = u * 0.6; //That's LabVIEW's output!!

		atomic {
			m_sensor->u = (uint16_t) (((u + 6.0) / 4.8) * 1638.0);
			DAC12_0DAT = m_sensor->u;
			//DAC12_0DAT = (uint16_t) (((u + 6.0) / 4.8) * 1638.0);
			call Leds.led1Toggle();
		}

	}
   
   event void TimerTimeout.fired() {
		
	atomic {
          if (!uartFull && ((gotTheta && !gotXc)||(!gotTheta && gotXc)))
            {
        	  if((call TimerBeacon.getNow()) - beacontime < ( (((uint32_t) 1 << (uint32_t) ((call MLME_GET.macBeaconOrder())-4)) * bi_bo4)-2000)) {
        		  post calcAndAct();

        		  gotTheta = FALSE;
        		  gotXc = FALSE;

        		  /*uartQueue[uartIn] = &m_frame;
        		  uartIn = (uartIn + 1) % UART_QUEUE_LEN;

        		  if (!uartBusy)
        		  {
        			  post uartSendTask();
        			  uartBusy = TRUE;
        		  }*/
        	  }
            } else {
            	//call Leds.led0Toggle();
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
        	  if((call TimerBeacon.getNow()) - beacontime < ( (((uint32_t) 1 << (uint32_t) ((call MLME_GET.macBeaconOrder())-4)) * bi_bo4)-2000)){
        		  post calcAndAct();

        		  gotTheta = FALSE;
        		  gotXc = FALSE;

        		  /*ret = uartQueue[uartIn];
        		  uartQueue[uartIn] = &m_frame;
        		  uartIn = (uartIn + 1) % UART_QUEUE_LEN;

        		  if (!uartBusy)
        		  {
        			  post uartSendTask();
        			  uartBusy = TRUE;
        		  }*/
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
        //call Leds.led1Toggle();
      }
      return ret;
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
     beacontime = call TimerBeacon.getNow();
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
