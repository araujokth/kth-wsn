/*
 * ActuatorAppC.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.ieee802154.tkn154
 *        Created on: 2010/05/10 
 * Last modification: 
 *            Author: aziz
 *     
 */

configuration ActuatorAppC{
}
implementation {

	components new TimerMilliC() as TimerBeacon;
	components MainC, ActuatorC, LedsC, SerialActiveMessageC as Serial, new TimerMilliC() as TimerTimeout;
	ActuatorC.Boot -> MainC;
	ActuatorC.Leds -> LedsC;
	
	ActuatorC.TimerTimeout -> TimerTimeout;
	ActuatorC.TimerBeacon -> TimerBeacon;
		
	/****************************************
	* 802.15.4
	*****************************************/
#ifdef TKN154_BEACON_DISABLED
	components Ieee802154NonBeaconEnabledC as MAC;
#else
	components Ieee802154BeaconEnabledC as MAC;
	ActuatorC.MLME_SCAN -> MAC;
	ActuatorC.MLME_SYNC -> MAC;
	ActuatorC.MLME_BEACON_NOTIFY -> MAC;
	ActuatorC.MLME_SYNC_LOSS -> MAC;
	ActuatorC.BeaconFrame -> MAC;
	
	components RandomC;
	ActuatorC.Random -> RandomC;


#endif
	
	ActuatorC.MLME_RESET -> MAC;
	ActuatorC.MLME_SET -> MAC;
	ActuatorC.MLME_GET -> MAC;
	  
	ActuatorC.MLME_START -> MAC;
	ActuatorC.MCPS_DATA -> MAC;
	ActuatorC.Frame -> MAC;
	ActuatorC.Packet -> MAC;
	
	ActuatorC.SerialControl -> Serial;
	ActuatorC.UartSend -> Serial.AMSend[AM_ENCMSG];
	ActuatorC.UartReceive -> Serial.Receive[AM_ENCMSG];
	ActuatorC.UartAMPacket -> Serial;

}

