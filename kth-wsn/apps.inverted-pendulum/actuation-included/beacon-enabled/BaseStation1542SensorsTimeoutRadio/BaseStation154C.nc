
/*
 * BaseStation154C.nc
 * 
 * KTH | Royal Institute of Technology
 * Automatic Control
 *
 *           Project: se.kth.tinyos2x.ieee802154.tkn154
 *        Created on: 2010/05/10 
 * Last modification: 2010/04/19  
 *            Author: aitorhh
 *     
 */

configuration BaseStation154C {
}implementation {
  components MainC, LedsC, new TimerMilliC() as TimerTimeout, new Timer62500C() as Timer;
  
#ifdef TKN154_BEACON_DISABLED
	components Ieee802154NonBeaconEnabledC as MAC;
#else
	components Ieee802154BeaconEnabledC as MAC;
	App.IEEE154TxBeaconPayload -> MAC;	  
#endif

  components BaseStation154P as App,
             SerialActiveMessageC as Serial;
  
  App.MLME_START -> MAC;
  App.MCPS_DATA -> MAC;
  App.Frame -> MAC;
  App.BeaconFrame -> MAC;
  App.Packet -> MAC;

  MainC.Boot <- App;
  App.Leds -> LedsC;
  App.MLME_RESET -> MAC;
  App.MLME_SET -> MAC;
  App.MLME_GET -> MAC;
  
  App.SerialControl -> Serial;
  App.UartSend -> Serial.AMSend[AM_ENCMSG];
  App.UartReceive -> Serial.Receive[AM_ENCMSG];
  App.UartAMPacket -> Serial;
 
 
  App.TimerTimeout -> TimerTimeout;
  
  components UserButtonC;
  App.Get -> UserButtonC;
  App.Notify -> UserButtonC;

}
