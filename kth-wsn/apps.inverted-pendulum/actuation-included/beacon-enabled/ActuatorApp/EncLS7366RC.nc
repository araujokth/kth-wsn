/*
 * EncLS7366RC.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.ieee802154.tkn154
 *        Created on: 2010/05/10 
 * Last modification: 2010/05/28
 *            Author: khakulov
 *     
 */

#include "app_sensors.h"
#include "UserButton.h"

module EncLS7366RC {
	uses {
		interface Boot;
		interface Leds;

		interface Packet;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_START;
		interface MLME_SET;
		interface MLME_GET;
		interface IEEE154Frame as Frame;
		
		interface Timer<TMilli> as TimerCalc;

#ifndef TKN154_BEACON_DISABLED
		interface MLME_SCAN;
		interface MLME_SYNC;
		interface MLME_BEACON_NOTIFY;
		interface MLME_SYNC_LOSS;
		interface IEEE154BeaconFrame as BeaconFrame;

		interface Random;
#endif

	}
}
implementation {

	uint16_t rate = DEFAULT_RATE;

#ifndef TKN154_BEACON_DISABLED
	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;
#endif
	void startApp();
	task void calcAndAct();
	
	
	/* LQG and Kalman filter */
	
	nx_float L[4], K[4][2], A_tau[4][4], A_htau[4][4], B_tau[4], B_htau[4], C[2][4];
     	nx_float x[4], u;
	int32_t Li[4], Ki[4][2], A_taui[4][4], A_htaui[4][4], B_taui[4], B_htaui[4], Ci[2][4];
     	int32_t xi[4], ui;
     	uint8_t i,j;
		
	
	EncMsg2Sensors *m_sensor;
	
	uint16_t out;
	

	event void TimerCalc.fired() {
		//call Leds.led2Toggle();

		post calcAndAct();
	}
	

	/**----------------------------------------------*/
	event void Boot.booted() {
		
		atomic {
    			ADC12CTL0 = REF2_5V +REFON;
    			DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;
		}
				out = 0x000;
		
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
		
		for(i=0;i<4;i++) Li[i] = (int32_t) (L[i] * 100.0);
		for(i=0;i<4;i++) for(j=0;j<4;j++) A_taui[i][j] = (int32_t) (A_tau[i][j] * 100.0);
		for(i=0;i<4;i++) for(j=0;j<4;j++) A_htaui[i][j] = (int32_t) (A_htau[i][j] * 100.0);
		for(i=0;i<4;i++) B_taui[i] = (int32_t) (B_tau[i] * 100.0);
		for(i=0;i<4;i++) B_htaui[i] = (int32_t) (B_htau[i] * 100.0);
		for(i=0;i<2;i++) for(j=0;j<4;j++) Ci[i][j] = (int32_t) (C[i][j] * 10.0);
		for(i=0;i<4;i++) for(j=0;j<2;j++) Ki[i][j] = (int32_t) (K[i][j] * 10.0);
		
		call TimerCalc.startPeriodicAt(0,100);

		//Init the MAC layer
		call MLME_RESET.request(TRUE);
	}

	task void calcAndAct() {
		
		int32_t y[2], x_bar[4], res_0[4];
		
		u = 6.0;
		  				
		atomic {
			DAC12_0DAT = (uint16_t) (((u + 6.0) / 4.8) * 1638.0);
			call Leds.led0Toggle();
		}
		
		y[0] = ((int16_t) m_sensor->data_xc) * 9.2E-5;
		y[1] = (((int16_t) m_sensor->data_theta) / 1024) * 3.1415;
		
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
		
		if(u > 10) u = -10.0;
		if(u < -10) u = -10.0;
		u = u * 0.6; //That's LabVIEW's output!!
		
		/*
		//x_bar= x_old_mote+ K*(y-C*x_old_mote);
		memset(x_bar, 0, sizeof(x_bar));
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<2;i++) for(j=0;j<4;j++) res_0[i] += Ci[i][j] * x[j];
		for(i=0;i<2;i++) res_0[i] = y[i] - res_0[i];
		for(i=0;i<4;i++) for(j=0;j<2;j++) x_bar[i] += Ki[i][j] * res_0[j];
		for(i=0;i<4;i++) x_bar[i] += x[i];
		
		//x_bar= A_tau*x_bar- B_tau*u_old;
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<4;i++) for(j=0;j<4;j++) res_0[i] += A_taui[i][j] * x_bar[j];
		for(i=0;i<4;i++) x_bar[i] = res_0[i] - B_taui[i] * u;
		
		//u_mote= L*x_bar;
		u = 0;
		for(i=0;i<4;i++) u += Li[i] * x_bar[i];
		
		//x_mote= A_htau*x_bar- B_htau*u_old;
		memset(res_0, 0, sizeof(res_0));
		for(i=0;i<4;i++) for(j=0;j<4;j++) res_0[i] += A_htaui[i][j] * x_bar[j];
		for(i=0;i<4;i++) x[i] = res_0[i] - B_htaui[i] * u;
		
		u=0;
		if(u > 10) u = -10.0;
		if(u < -10) u = -10.0;
		u = u * 0.6; //That's LabVIEW's output!!
		*/
		
		u = -6;
		  				
		atomic {
			DAC12_0DAT = (uint16_t) (((u + 6.0) / 4.8) * 1638.0);
			call Leds.led0Toggle();
		}	
		
	}

	/*********************************************************************
	 * 802.15.4 functions
	 *********************************************************************/
	void startApp()
	{
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);

		// scan only the channel where we expect the coordinator
		channelMask = ((uint32_t) 1) << RADIO_CHANNEL;

		// we want all received beacons to be signalled 
		// through the MLME_BEACON_NOTIFY interface, i.e.
		// we set the macAutoRequest attribute to FALSE
		call MLME_SET.macAutoRequest(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);
		m_wasScanSuccessful = FALSE;
		call MLME_SCAN.request (
				PASSIVE_SCAN, // ScanType
				channelMask, // ScanChannels
				scanDuration, // ScanDuration
				0x00, // ChannelPage
				0, // EnergyDetectListNumEntries
				NULL, // EnergyDetectList
				0, // PANDescriptorListNumEntries
				NULL, // PANDescriptorList
				0 // security
		);
	}

	event message_t* MLME_BEACON_NOTIFY.indication (message_t* frame)
	{
		// received a beacon frame
		ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();
		ieee154_macBSN_t beaconSequenceNumber = call BeaconFrame.getBSN(frame);

		if (!m_wasScanSuccessful) {
			// received a beacon during channel scanning
			if (call BeaconFrame.parsePANDescriptor(
							frame, RADIO_CHANNEL, page, &m_PANDescriptor) == SUCCESS) {
				// let's see if the beacon is from our coordinator...
				if (m_PANDescriptor.CoordAddrMode == ADDR_MODE_SHORT_ADDRESS &&
						m_PANDescriptor.CoordPANId == PAN_ID &&
						m_PANDescriptor.CoordAddress.shortAddress == COORDINATOR_ADDRESS) {
					// yes! wait until SCAN is finished, then syncronize to the beacons
					m_wasScanSuccessful = TRUE;
				}
			}
		} else {
			// received a beacon during synchronization, toggle LED2
			if (beaconSequenceNumber & 1)
			call Leds.led2On();
			else
			call Leds.led2Off();
		}

		return frame;
	}

	event void MLME_SCAN.confirm (
			ieee154_status_t status,
			uint8_t ScanType,
			uint8_t ChannelPage,
			uint32_t UnscannedChannels,
			uint8_t EnergyDetectListNumEntries,
			int8_t* EnergyDetectList,
			uint8_t PANDescriptorListNumEntries,
			ieee154_PANDescriptor_t* PANDescriptorList
	)
	{
		if (m_wasScanSuccessful) {
		
			// we received a beacon from the coordinator before
			call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress.shortAddress);
			call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
			call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor.ChannelPage, TRUE);

		} else
		startApp();
	}

	event void MLME_SYNC_LOSS.indication(
			ieee154_status_t lossReason,
			uint16_t PANId,
			uint8_t LogicalChannel,
			uint8_t ChannelPage,
			ieee154_security_t *security)
	{
		m_wasScanSuccessful = FALSE;
		call Leds.led2Off();
		startApp();
	}

	event void MLME_RESET.confirm(ieee154_status_t status)
	{
		if (status != IEEE154_SUCCESS) return;

		call MLME_SET.phyTransmitPower(TX_POWER);
		call MLME_SET.macShortAddress(TOS_NODE_ID);
		call MLME_SET.macAssociationPermit(FALSE);
		call MLME_SET.macRxOnWhenIdle(TRUE);

		call MLME_START.request(
				PAN_ID, // PANId
				RADIO_CHANNEL, // LogicalChannel
				0, // ChannelPage,
				0, // StartTime,
				BEACON_ORDER, // BeaconOrder
				SUPERFRAME_ORDER, // SuperframeOrder
				TRUE, // PANCoordinator
				FALSE, // BatteryLifeExtension
				FALSE, // CoordRealignment
				0, // CoordRealignSecurity,
				0 // BeaconSecurity
		);

		startApp();
	}
	event void MLME_START.confirm(ieee154_status_t status) {}


	event void MCPS_DATA.confirm (
			message_t *msg,
			uint8_t msduHandle,
			ieee154_status_t status,
			uint32_t timestamp
	) {}
	/*************************** Receive functions *************************/
	event message_t* MCPS_DATA.indication (message_t* frame) {
	
		ieee154_address_t deviceAddr;
	     	

	     	// We assume that the packets with length equal to EncMsg2Sensors
	     	// are a EncMsg2Sensors
	     	if (call Frame.getPayloadLength(frame) == sizeof(EncMsg2Sensors) &&
	         	call Frame.getFrameType(frame) == FRAMETYPE_DATA){
	         
	         		call Frame.getSrcAddr(frame, &deviceAddr);
	         		if(deviceAddr.shortAddress == 0) {
	         			m_sensor = (EncMsg2Sensors*)(call Packet.getPayload(frame,sizeof(EncMsg2Sensors)));
	         			
	         			post calcAndAct();
	         			
	         		}
	     	}
	      	return frame;
	
	}
	/*********************************************************************
	 * END OF 802.15.4 functions
	 *********************************************************************/
}
