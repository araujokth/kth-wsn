/* The Active Message abstraction on top of the TKN15.4 MAC. Take a look at the
 * top of TKN154ActiveMessageP.nc to understand the approach (and issues).
 **/

/** Disable scanning (MLME_SCAN will not work):
#define IEEE154_SCAN_DISABLED

// Disable promiscuous mode (PromiscuousMode.start() will not work):
#define IEEE154_PROMISCUOUS_MODE_DISABLED

// Disable association (MLME_ASSOCIATE will not work):
#define IEEE154_ASSOCIATION_DISABLED

// Disable association (MLME_DISASSOCIATE will not work):
#define IEEE154_DISASSOCIATION_DISABLED

// Disable coordinator realignment (MLME_ORPHAN will not work):
#define IEEE154_COORD_REALIGNMENT_DISABLED

// Disable transmission of broadcasts from coordinator to devices:
#define IEEE154_COORD_BROADCAST_DISABLED

// Disable indirect transmissions (MCPS_DATA.request with TX_OPTIONS_INDIRECT will not work):
#define IEEE154_INDIRECT_TX_DISABLED

// Disallow next higher layer to switch to receive mode (MLME_RX_ENABLE will fail):
#define IEEE154_RXENABLE_DISABLED

// Set the MAC Tx queue sizes to minimum (one DATA frame):
#ifdef TXCONTROL_POOL_SIZE
#undef TXCONTROL_POOL_SIZE
#endif
#define TXCONTROL_POOL_SIZE 0

#ifdef TXFRAME_POOL_SIZE 
#undef TXFRAME_POOL_SIZE 
#endif
#define TXFRAME_POOL_SIZE 1

#ifdef CAP_TX_QUEUE_SIZE 
#undef CAP_TX_QUEUE_SIZE
#endif
#define CAP_TX_QUEUE_SIZE 1

#if defined(LOW_POWER_LISTENING)
// If LOW_POWER_LISTENING is defined, then AMSenderC() instantiates an LplAMSenderC 
// instead of a DirectAMSenderC(). Currently we only support a DirectAMSenderC().
#error "LOW_POWER_LISTENING must not be defined when TKN15.4 MAC is used!"
#endif**/

configuration ActiveMessageC {
  provides {
    interface SplitControl;
	interface AMSend[am_id_t id];
    interface Receive[am_id_t id];
    interface Receive as Snoop[am_id_t id];
    interface Packet;
    interface AMPacket;
    interface PacketAcknowledgements;
	interface Capisrunning;
	// The following interface may be useful to access more metadata 
    // (RSSI, LQI, timestamp) of a frame
    interface IEEE154Frame;
    interface HasTimeSlot;

/*    interface PacketTimeStamp<T32khz, uint32_t> as PacketTimeStamp32khz;*/
/*    interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;*/
/*    interface LowPowerListening;*/
  }
}
implementation {

  components TKN154ActiveMessageP as AM;

  SplitControl = AM;
  AMSend       = AM;
  Receive      = AM.Receive;
  Snoop        = AM.Snoop;
  AMPacket     = AM;
  Packet       = AM;
  PacketAcknowledgements = AM;
  Capisrunning = AM;		
  HasTimeSlot  = AM;
/*  LowPowerListening = AM;*/
/*  PacketTimeStamp32khz = AM;*/
/*  PacketTimeStampMilli = AM;*/

  //components ActiveMessageAddressC;
  //AM.ActiveMessageAddress -> ActiveMessageAddressC;

  components new StateC();
  AM.SplitControlState -> StateC;


  components Ieee802154BeaconEnabledC as MAC;
  
  components new Alarm62500hz32VirtualizedC() as EndAlarm;    
  components new Alarm62500hz32VirtualizedC() as TSBeginAlarm;
  
  components RandomC;
  components LedsC;
  


  IEEE154Frame = MAC;

  AM.Leds -> LedsC;
  	
  AM.MLME_RESET -> MAC;
  AM.MCPS_DATA -> MAC;
  AM.MLME_SET -> MAC;
  AM.MLME_GET -> MAC;
  AM.Frame-> MAC;
  AM.SubPacket -> MAC.Packet;
  AM.MLME_SCAN -> MAC;
  AM.MLME_SYNC -> MAC;
  AM.MLME_BEACON_NOTIFY -> MAC;
  AM.MLME_SYNC_LOSS -> MAC;
  AM.MLME_GTS -> MAC;
  AM.Random -> RandomC;
  AM.BeaconFrame -> MAC;
  AM.GtsUtility -> MAC;
  AM.IsEndSuperframe -> MAC;
  AM.IsGtsOngoing -> MAC.IsGtsOngoing;	
  AM.SF -> MAC;
  AM.TC -> MAC;
  AM.CapOverAlarm -> EndAlarm;
  AM.MyTSBegins -> TSBeginAlarm;
  
  components new QueueC(message_t*, 10) as MessageQueueP;
  AM.MessageQueue -> MessageQueueP;
  

  components new TimerMilliC() as TimerPhase;
  AM.TimerPhase -> TimerPhase;

  components LocalTime62500hzC;
  AM.LocalTime -> LocalTime62500hzC;

  
  //MainC.Boot <- App;
  //MainC.SoftwareInit -> AM;
 
}
