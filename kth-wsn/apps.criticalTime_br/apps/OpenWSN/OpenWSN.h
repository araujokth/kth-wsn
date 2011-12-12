#ifndef OPENWSN_H 
#define OPENWSN_H

enum {
   HOPPING_ENABLED = FALSE,
   LENGTHCELLFRAME = 16,
   MAXNUMNEIGHBORS = 10,
   //delays
   DELAY_LOSING_NEIGHBOR_32KHZ     = 327680,  //32kHz ticks = 10s (32768 ticks = 1s)
   DELAY_LOSING_NEIGHBOR_1KHZ      =  10240,  // 1kHz ticks = 10s ( 1024 ticks = 1s)
   DELAY_LOST_NEIGHBOR_32KHZ       = 655360,  //32kHz ticks = 20s (32768 ticks = 1s)
   DELAY_LOST_NEIGHBOR_1KHZ        =  40000,  // 1kHz ticks = 20s ( 1024 ticks = 1s)
//   DELAY_LOST_NEIGHBOR_1KHZ        =  20480,  // 1kHz ticks = 20s ( 1024 ticks = 1s)
   TXRETRIES                       =      3,
   //state
   QUEUELENGTH                     =      20,
   SERIAL_OUTPUT_BUFFER_SIZE       =    300,
   SERIAL_INPUT_BUFFER_SIZE        =    200, //not more than 255 (length encoded in 1B)
   //misc
   GOODNEIGHBORMINPOWER            =    219, //-80dBm in 8-bit 2's compl. (-80 -> -35 -> 36 -> 219)
   BADNEIGHBORMAXPOWER             =    229, //-70dBm in 8-bit 2's compl. (-70 -> -25 -> 26 -> 0001 1010 -> 1110 0101-> 229)
   SWITCHSTABILITYTHRESHOLD        =      3,
   TX_POWER                        =     31, //1=-25dBm, 31=0dBm (max value)
   MAXPREFERENCE                   =      2,
   INVALID_TIMESTAMP               = 0x80000000L,
};

//Slot Durations (standards compliant)
enum {
  TsTxOffset       =   86, //32kHz ticks =  2.106ms
  TsRxOffset       =   44, //32kHz ticks =  1.129ms (<TsTxOffset)
  TsRxWaitTime     =   93, //32kHz ticks =  2.014ms (both TsPacketWaitTime and TsAckWaitTime)
  TsTxAckDelay     =   64, //32kHz ticks =  2.000ms
  TsRxAckDelay     =   1, //32kHz ticks =  2.000ms (has to happen beofre TsTxAckDelay expires)
  SLOT_TIME        =  320, //32kHz ticks = 10.000ms
  radio_delay      =   17, //measured
  };
//enum {
//  TsTxOffset       =   86, //32kHz ticks =  2.106ms
//  TsRxOffset       =   37, //32kHz ticks =  1.129ms (<TsTxOffset)
//  TsRxWaitTime     =   83, //32kHz ticks =  2.014ms (both TsPacketWaitTime and TsAckWaitTime)
//  TsTxAckDelay     =   64, //32kHz ticks =  2.000ms
//  TsRxAckDelay     =   64, //32kHz ticks =  2.000ms (has to happen beofre TsTxAckDelay expires)
//  SLOT_TIME        =  640, //32kHz ticks = 20.000ms
//  radio_delay      =   21, //measured
//  };


//enum {
//  TsTxOffset       =   86, //32kHz ticks =  2.106ms
//  TsRxOffset       =   37, //32kHz ticks =  1.129ms (<TsTxOffset)
//  TsRxWaitTime     =   66, //32kHz ticks =  2.014ms (both TsPacketWaitTime and TsAckWaitTime)
//  TsTxAckDelay     =   64, //32kHz ticks =  2.000ms
//  TsRxAckDelay     =   64, //32kHz ticks =  2.000ms (has to happen beofre TsTxAckDelay expires)
//  SLOT_TIME        =  640, //32kHz ticks = 20.000ms
//  radio_delay      =   21, //measured
//  };

//Slot Durations (video transmission)
//enum {
//   TsTxOffset       =   197, //32kHz ticks =  6ms
//   TsRxOffset       =   131, //32kHz ticks =  4ms (<TsTxOffset)
//   TsRxWaitTime     =   131, //32kHz ticks =  4ms (both TsPacketWaitTime and TsAckWaitTime)
//   TsTxAckDelay     =    66, //32kHz ticks =  2ms
//   TsRxAckDelay     =   164, //32kHz ticks =  5ms (has to happen beofre TsTxAckDelay expires)
//   SLOT_TIME        =   983, //32kHz ticks = 30ms
//   radio_delay      =    19, //measured
//};

typedef uint8_t   cellType_t;
typedef uint16_t  slotOffset_t;
typedef uint8_t   channelOffset_t;
typedef uint16_t  shortnodeid_t;
typedef uint64_t  longnodeid_t;
typedef uint32_t  timervalue_t;
typedef uint16_t  errorparameter_t;
typedef uint8_t   dagrank_t;
typedef uint16_t  asn_t;

typedef struct open_addr_t {   //always written big endian, i.e. MSB in addr[0]
   uint8_t type;
   union {
      uint8_t addr_16b[2];
      uint8_t addr_64b[8];
      uint8_t addr_128b[16];
      uint8_t panid[2];
      uint8_t prefix[8];
   };
} open_addr_t;



//cell types
enum {
   ADDR_NONE   = 0,
   ADDR_16B    = 1,
   ADDR_64B    = 2,
   ADDR_128B   = 3,
   ADDR_PANID  = 4,
   ADDR_PREFIX = 5,
};

//cell types
enum {
   CELLTYPE_OFF      =0,
   CELLTYPE_TXRX     =1,
   CELLTYPE_RXSERIAL =2
};

enum {
   LITTLE_ENDIAN = TRUE,
   BIG_ENDIAN    = FALSE,
};

enum {
   IS_ADV        = TRUE,
   IS_NOT_ADV    = FALSE,
};

enum {
   IANA_ICMPv6                        = 0x3a,
   IANA_UDP                           = 0x11,
   IANA_TCP                           = 0x06,
   IANA_UNDEFINED                     = 0x00,
   IANA_ICMPv6_ECHO_REQUEST           = 128,
   IANA_ICMPv6_ECHO_REPLY             = 129,
   IANA_ICMPv6_RS                     = 133,
   IANA_ICMPv6_RA                     = 134,
   IANA_ICMPv6_RA_PREFIX_INFORMATION  = 3,
   IANA_ICMPv6_RPL                    = 155,
};

enum {
   //TCP
   WKP_TCP_ECHO       =    7,
   WKP_TCP_INJECT     = 2188,
   WKP_TCP_HTTP       =   80,
   WKP_TCP_DISCARD    =    9,
   //UDP
   WKP_UDP_CHANNEL    = 2191,
   WKP_UDP_ECHO       =    7,
   WKP_UDP_GINA       = 2190,
   WKP_UDP_HELI       = 2192,
   WKP_UDP_INJECT     = 2188,
   WKP_UDP_DISCARD    =    9,
   WKP_UDP_SENSOR     = 2189,
};

//============================================ packet formats =====================================

//OpenQueue entry definition
typedef struct OpenQueueEntry_t {
   //admin
   uint8_t       creator;                    //the component which called getFreePacketBuffer()
   uint8_t       owner;                      //the component which currently owns the entry
   void*         payload;                    //pointer to the start of the payload within 'packet'
   uint8_t       length;                     //length in bytes of the payload
   //l4
   uint8_t       l4_protocol;                //l4 protocol to be used
   uint16_t      l4_sourcePortORicmpv6Type;  //l4 source port
   uint16_t      l4_destination_port;        //l4 destination port
   void*         l4_payload;                 //pointer to the start of the payload of l4 (used for retransmits)
   uint8_t       l4_length;                  //length of the payload of l4 (used for retransmits)
   //l3
   open_addr_t   l3_destinationORsource;     //128b IPv6 destination (down stack) or source address (up)
   //l2
   open_addr_t   l2_nextORpreviousHop;       //64b IEEE802.15.4 next (down stack) or previous (up) hop address
   uint8_t       l2_frameType;               //beacon, data, ack, cmd
   uint8_t       l2_retriesLeft;
   bool          l2_transmitInFirstSlot;     //for video transmission
   //l1 (drivers)
   uint8_t       l1_txPower;
   uint8_t       l1_channel;
   uint8_t       l1_rssi;
   uint8_t       l1_lqi;
   bool          l1_crc;
   uint32_t      l1_rxTimestamp;
   //the packet
   uint8_t       packet[128];
} OpenQueueEntry_t;

typedef struct slotChannel_t {
   bool exists;
   bool primary;
   slotOffset_t slotOffset;
   channelOffset_t channelOffset;
} slotChannel_t;

//component identifiers
enum {
   COMPONENT_NULL             =  0,
   //l7
   COMPONENT_APPTCPECHO       =  1,
   COMPONENT_APPTCPINJECT     =  2,
   COMPONENT_APPTCPOHLONE     =  3,
   COMPONENT_APPTCPPRINT      =  4,
   COMPONENT_APPUDPCHANNEL    =  5, //only exist on the GINA port
   COMPONENT_APPUDPECHO       =  6,   
   COMPONENT_APPUDPGINA       =  7, //only exist on the GINA port
   COMPONENT_APPUDPHELI       =  8, //only exist on the GINA port
   COMPONENT_APPUDPINJECT     =  9,
   COMPONENT_APPUDPLEDS       = 10, //only exist on the GINA port
   COMPONENT_APPUDPPRINT      = 11,
   COMPONENT_APPUDPSENSOR     = 12,
   COMPONENT_BLINKHOP	 	  = 34,
   COMPONENT_APPDATA	 	  = 35,
   //l4
   COMPONENT_TCP              = 13,
   COMPONENT_UDP              = 14,
   //l3b
   COMPONENT_FORWARDING       = 14,
   COMPONENT_ICMPv6           = 16,
   COMPONENT_ICMPv6ECHO       = 17,
   COMPONENT_ICMPv6ROUTER     = 18,
   COMPONENT_ICMPv6RPL        = 19,
   //l3a
   COMPONENT_OPENBRIDGE       = 20,
   COMPONENT_IPHC             = 21,
   //l2b
   COMPONENT_RES              = 22,
   COMPONENT_NEIGHBORS        = 23,
   //l2a
   COMPONENT_CELLUSAGE        = 24,
   COMPONENT_MAC              = 25,
   //phy
   COMPONENT_CC2420DRIVER     = 26,
   COMPONENT_CC2420TRANSMIT   = 27,
   COMPONENT_CC2420RECEIVE    = 28,
   COMPONENT_CC2420CONTROL    = 29,
   //cross-layer
   COMPONENT_IDMANAGER        = 30,
   COMPONENT_OPENQUEUE        = 31,
   COMPONENT_OPENSERIAL       = 32,
   COMPONENT_PACKETFUNCTIONS  = 33,
};

//============================================ debug ======================================

//status elements
enum {
   STATUS_ADVERTISEP_DAGRANK             =0,
   STATUS_CELLUSAGEP_CELLTABLE           =1,
   STATUS_NEIGHBORSP_NEIGHBORS           =2,
   STATUS_SERIALIOP_OUTPUTBUFFERINDEXES  =3,
   STATUS_OPENQUEUEP_QUEUE               =4,
   STATUS_IEEE802154EP_SYNCRAND          =5,
   STATUS_IDMANAGER_ID                   =6,
};

//error codes
enum {
   ERR_BOOTED                                    =0,   //main program booted                                       [AppP] 
   ERR_RES_READY                                 =1,   //reservation is ready                                      [AppP]        arg1=neighbor
   ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND           =2,   //send.sendDone for packet I didn't send                    [AppP,AdvertiseP,KeepAliveC,ReservationP]
   ERR_NO_NEXTHOP                                =3,   //no nextHop                                                [RPLP]
   ERR_NO_FREE_PACKET_BUFFER                     =4,   //no free Queuepkt Cell                                     [NeighborsP, NRESP, AppSensorP, IEEE802154EP] arg1=codeLocation
   ERR_OVER_255_CELLS_SAME_TYPE_NEIGHBOR         =5,   //over 255 cells of same type and same neighbor             [CellUsageP]   arg1=type,        arg2=neighbor
   ERR_WRONG_CELLTYPE                            =6,   //wrong celltype                                            [CellUsageP,IEEE802154EP,OpenQueueP] arg1=type
   ERR_RES_NOT_ON_RESSLOTOFFSET                  =7,   //CELLTYPE_RES not on SlotOffset RESSLOTOFFSET              [CellUsageP]   arg1=slotOffset,  arg2=channelOffset
   ERR_ILLEGAL_TYPE_TRANSITION                   =8,   //illegal type transition arg1->arg2                        [CellUsageP]   arg1=old_type,  arg2=old_type
   ERR_MODIFYING_OVERHEAD_SLOTS                  =9,   //modifying overhead slots                                  [CellUsageP]   arg1=slotOffset,  arg2=channelOffset
   ERR_SETUSAGE_NOT_FOR_RES_ADV                  =10,  //CellUsageSet.setUsage does not set RES and ADV slots      [CellUsageP]   arg1=slotOffset,  arg2=channelOffset
   ERR_BUSY_SENDING                              =11,  //busy sending a packet                                     [RPLP,TCPP] arg1=location
   ERR_LOST_SYNC                                 =12,  //lost synchronization                                      [GlobalTimeP]
   ERR_SENDDONE_WHILE_NOT_BUSY                   =13,  //SimpleSend.sendDone while busy==FALSE                     [KeepAliveC,AdvertiseP]
   ERR_MSG_UNKNOWN_TYPE                          =14,  //received message of unknown type                          [NRESC,OpenQueueP] arg1=type
   ERR_TXDATA_RESERVATION_TO_NEIGHBOR_FAILED     =15,  //ReservationUpdate.addSlot() to neighbor failed            [NeighborsP]   arg1=neighbor
   ERR_NEIGHBORS_FULL                            =16,  //neighbors table is full                                   [NeighborsP] arg1=MAXNUMNEIGHBORS
   ERR_NEIGHBOR_NOT_REACHABLE_YET                =17,  //neighbor not reachable (yet?)                             [ReservationP] arg1=neighbor
   ERR_CONFIRMRESERVATION_FAILED                 =18,  //confirmReservation failed   [ReservationP] arg1=type,  arg2=neighbor
   ERR_SETUSAGE_OFF_FAILED                       =19,  //CellUsageSet.setUsage OFF failed                          [ReservationP] arg1=slotOffset,  arg2=channelOffset
   ERR_RES_TYPE_UNKNOWN                          =20,  //RES type unknown                                          [ReservationP] arg1=type arg2=i
   ERR_LOCALRES_NOT_TXDATA_OR_OFF                =21,  //local RES which is not TXDATA or OFF                      [ReservationP] arg1=type
   ERR_LOSING_NEIGHBOR                           =22,  //losing neighbor==arg1                                     [NeighborsP] arg1=neighbor->addr_64b[7]
   ERR_TODO_UNKNOWN                              =23,  //unsupported todo code                                     [ReservationP] arg1=todo
   ERR_RXMSG_NOT_RES                             =24,  //received a message which is not RES                       [ReservationP] arg1=RXMESG_type
   ERR_UNEXPECTED_RES_ACTION_SUCCESS             =25,  //received a RES_ACTION_SUCCESS I wasn't waiting for        [ReservationP]
   ERR_UNSUPPORTED_TXCOMMAND                     =26,  //unsupported txCommand in received message                 [ReservationP] arg1=txCommand
   ERR_SENDDONE_WHILE_NOT_TODO_WAIT_FOR_SENT     =27,  //SimpleSend.sendDone with todo!=WAIT_FOR_SENT              [ReservationP]
   ERR_UNSUPPORTED_ACK_RESPONSE                  =28,  //unsupported ACK response                                  [ReservationP]
   ERR_WRONG_STATE_IN_STARTSLOTTASK              =29,  //wrong state in startSlotTask                              [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_TURN_OFF_RADIO_FAILED                     =30,  //could not turn off radio                                  [IEEE802154EP]  arg1=slotOffset
   ERR_WRONG_STATE_IN_NEWSLOT                    =31,  //wrong state in newSlot                                    [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_NO_TIME_TO_PREPARE_TX                     =32,  //no time to prepare TX                                     [IEEE802154EP]  arg1=slotOffset
   ERR_NO_TIME_TO_PREPARE_ACK                    =34,  //no time to prepare ACK                                    [IEEE802154EP]  arg1=slotOffset
   ERR_WRONG_STATE_IN_FASTTIMER_FIRED            =35,  //wrong state in FastTimer.fired                            [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_WRONG_STATE_IN_SUBCONTROL_STARTDONE       =36,  //wrong state in SubControl.startDone                       [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_WRONG_STATE_IN_SUBCONTROL_STOPDONE        =37,  //wrong state in SubControl.stopDone                        [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_START_RADIO_FOR_SYNCHRONIZER_FAILED       =38,  //could not start radio for synchronizer                    [IEEE802154EP]  arg1=slotOffset
   ERR_START_RADIO_AT_START_SLOT_FAILED          =39,  //could not start radio at beginning of slot                [IEEE802154EP]  arg1=slotOffset
   ERR_WRONG_STATE_IN_CC2420CONFIG_SYNCDONE      =40,  //wrong state in CC2420Config.syncDone                      [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_WRONG_STATE_IN_SUBSEND_SENDDONE           =41,  //wrong state in SubSend.sendDone                           [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_UNDEFINED_TYPE                            =42,  //undefined type                                            [OpenQueueC]   arg1=type
   ERR_NOT_DATA_MSG_THROUGH_SENDDATA             =43,  //should receive only DATA through SendDATA                 [OpenQueueC]   arg1=message type
   ERR_NOT_KA_MSG_THROUGH_SENDKA                 =44,  //should receive only DATA through KASend                   [OpenQueueC]   arg1=message type
   ERR_SEND_IMPOSSIBLE_PACKET_ADDRESS            =45,  //Send with an impossible packet address                    [OpenQueueP]   arg1=code_location arg2=requested_address
   ERR_NOT_RES_MSG_THROUGH_SENDRES               =46,  //should receive only RES through SendRES                   [OpenQueueC]   arg1=message type
   ERR_NOT_ADV_MSG_THROUGH_SENDADV               =47,  //should receive only ADV through SendADV                   [OpenQueueC]   arg1=message type
   ERR_DEQUEUEDONE_WHILE_BUSY_INFORMING_REQUESTER =48,  //Dequeue.done while busy==TRUE                            [OpenQueueC]
   ERR_DEQUEUEDONE_FOR_MSG_I_DID_NOT_SEND        =49,  //DeQueue.done for packet I didn't send                     [OpenQueueC]
   ERR_ACQUIRED_SYNC                             =50,  //Acquired synchronization                                  [GlobalTimeP]
   ERR_LOSING_SYNC                               =51,  //Losing synchronization                                    [GlobalTimeP]
   ERR_UNKNOWN_NEIGBOR                           =52,  //Unknown Neighbors                                         [IEEE802154EP]  arg1=neighbor
   ONGOINGRES_TABLE_FULL                         =53,  //ongoingRes table is full                                  [ReservationP] arg1=NUMONGOINGRES
   ERR_RES_SENT                                  =54,  //RES sent                                                  [ReservationP] arg1=slotOffset arg2=ack
   ERR_RES_RECEIVED                              =55,  //RES received                                              [ReservationP] arg1=txCommand arg2=rxCommand 
   ERR_RES_OUTCOME                               =56,  //whether RES has successed                                 [AppP]        arg1=neighbor arg2=error
   ERR_LOST_NEIGHBOR                             =57,  //lost neighbor==arg1                                       [NeighborsP] arg1=neighbor->addr_64b[7]
   ERR_GLOBALSLOTOFFSET_TOO_LARGE                =58,  //globalSlotOffset>SLOT_TIME                                [GlobalTimeP]  arg1=globalSlotOffset arg2=SLOT_TIME
   ERR_TOO_LONG_SINCE_RX                         =59,  //time since received message>SLOT_TIME                     [GlobalTimeP]  arg1=now-myReceptionLocalTime arg2=SLOT_TIME
   ERR_POIPOI                                    =60,  //pure debug                                                [temporary]    arg1=whatever arg2=whatever
   ERR_SENDDONE                                  =61,  //sendDone                                                  [AppP]    arg1=error arg2=ack
   ERR_REGISTERRES_TYPE_NOT_SUPPORTED            =63,  //registerRES type not supported                            [ReservationP] arg1=type
   ERR_UNSUPPORTED_RXCOMMAND                     =64,  //unsupported rxCommand in received message                 [ReservationP] arg1=txCommand
   ERR_INCONSISTENT_TXCOMMAND                    =65,  //inconsistent TXCOMMAND received from neighbor             [ReservationP] arg1=neighbor arg2=slotOffset
   ERR_INVALID_GLOBALSLOTOFFSET_TIMESTAMP        =66,  //invalid globalSlotOffset timestamp                        [GlobalTimeP,NeighborsP] arg1=source of message arg2=code locator
   ERR_INVALID_LOCALTIME_TIMESTAMP               =67,  //invalid localTime timestamp                               [GlobalTimeP,NeighborsP] arg1=source of message arg2=code locator
   ERR_REMOVE_ALL_CELLS                          =68,  //remove all celltype=arg1 with neighbor=arg2                               [CellUsageP] arg1=celltype arg2=neighbor
   ERR_INCONSISTENT_RXCOMMAND                    =69,  //inconsistent RXCOMMAND received from neighbor             [ReservationP] arg1=neighbor arg2=slotOffset
   ERR_FAILRESERVATION                           =70,  //reservation fails after todo=arg1                         [ReservationP] arg1=todo
   ERR_DETECTED_FROZEN                           =71,  //frozen reservation type=arg1 neighbor=arg2                [ReservationP] arg1=type arg2=neighbor
   ERR_UNSUPPORTED_COMMAND                       =72,  //unsupported command=arg1                                  [SerialIOP] arg1=command
   ERR_NO_TIME_TO_PREPARE_RX                     =73,  //no time to prepare RX                                     [IEEE802154EP]  arg1=slotOffset
   ERR_PREPARESENDDONE_FAILED                    =74,  //RadioSend.prepareSendDone returned with error             [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_WRONG_STATE_IN_RECEIVE                    =75,  //wrong state in receive                                    [IEEE802154EP] arg1=state arg2=slotOffset
   ERR_RECEIVENOW_FAILED                         =76,  //receiveNow failed                                         [IEEE802154EP] arg1=state arg2=slotOffset
   ERR_WRONG_STATE_IN_PREPARESEND                =77,  //RadioSend.prepareSend received but radio not S_STARTED    [CC2420driverP] arg1=state
   ERR_PREPARESEND_FAILED                        =78,  //RadioSend.prepareSend failed                              [IEEE802154EP]  arg1=slotOffset
   ERR_TXFIFOWRITEDONE_FAILED                    =79,  //TXFIFO.writeDone failed                                   [CC2420TransmitP]  arg1=error
   ERR_SETCHANNEL_FAILED                         =80,  //CC2420Config.setChannel failed                            [CC2420driverP]
   ERR_WRONG_STATE_IN_PREPARERECEIVE             =81,  //RadioSend.prepareReceive received but radio not S_STARTED [CC2420driverP] arg1=state
   ERR_PREPARERECEIVE_FAILED                     =82,  //RadioSend.prepareReceive failed                           [IEEE802154EP]  arg1=slotOffset
   ERR_PREPARERECEIVEDONE_FAILED                 =83,  //PrepareReceiveDone failed                                 [IEEE802154EP]  arg1=error arg2=slotOffset
   ERR_WRONG_STATE_LOADPACKET                    =84,  //loadpacket while state=arg1                               [CC2420TransmitP] arg1=state
   ERR_WRONG_STATE_IN_RECEIVEDNOTHING            =85,  //wrong state in receivedNothing                            [IEEE802154EP] arg1=state arg2=slotOffset
   ERR_BUFFER_NOT_READY                          =86,  //buffer not ready                                          [IEEE802154EP] arg1=slotOffset
   ERR_WRONG_STATE_SENDNOW                       =87,  //wrong state=arg1 in sendNow                               [CC2420TransmitP] arg1=slotOffset
   ERR_SYNC_RACE_CONDITION                       =88,  //race condition in resyncTask                              [IEEE802154EP] arg1=slotOffset arg2=dataGlobalSlotOffset
   ERR_PACKET_LARGER_BYTES_LEFT                  =89,  //packet larger than free_bytes_left_in_fifo                [CC2420ReceiveP] arg1=rxFrameLength + 1 arg2=free_bytes_left_in_fifo
   ERR_PACKET_TOO_LARGE                          =90,  //packet too large                                          [CC2420ReceiveP] arg1=rxFrameLength
   ERR_INCORRECT_CRC                             =91,  //incorrect CRC                                             [CC2420ReceiveP]
   ERR_PACKET_TOO_SHORT                          =92,  //packet too short                                          [CC2420ReceiveP] arg1=length
   ERR_SFD_DROPPED                               =93,  //sfd dropped                                               [CC2420TransmitP]
   ERR_WRONG_STATE_IN_PREPARESENDDONE            =94,  //wrong state in prepareSendDone                            [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_WRONG_STATE_IN_PREPARERECEIVEDONE         =95,  //wrong state in prepareReceiveDone                         [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_RFOFF_FAILED                              =96,  //RadioControl.rfOff failed                                 [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_SENDNOW_FAILED                            =97,  //RadioSend.sendNow failed                                  [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_SENDNOWDONE_FAILED                        =98,  //RadioSend.sendNowDone failed                              [IEEE802154EP]  arg1=state arg2=slotOffset
   ERR_WRONG_PANID                               =99,  //received msg with wrong PANid                             [IEEE802154EP]  arg1=msg_panid
   ERR_6LOWPAN_UNSUPPORTED                       =100, //unsupported 6LoWPAN parameter                             [IPHC] arg1=location arg2=param
   ERR_RCVD_ECHO_REQUEST                         =101, //received echo request                                     [RPLC]
   ERR_RCVD_ECHO_REPLY                           =102, //received echo reply                                       [RPLC]
   ERR_UNSUPPORTED_NEXT_HEADER                   =103, //unsupported next header                                   [RPLC] arg1=next_header
   ERR_UNSUPPORTED_ICMPV6_TYPE                   =104, //unsupported ICMPv6 type                                   [RPLC] arg1=icmpv6_type arg2=location
   ERR_PACKET_EMPTY                              =105, //packet empty                                              [CC2420ReceiveP] arg1=rxFrameLength
   ERR_WRONG_ADDR_TYPE                           =106, //wrong address type                                        [IEEE802154EP,IDManagerP,PacketFunctions] arg1=addressType arg2=codeLocation
   ERR_IEEE154_UNSUPPORTED                       =107, //unsupported 802154 parameter                              [IEEE802154EP] arg1=location arg2=param
   ERR_INVALID_NEXTHOP                           =108, //invalid nextHop                                           [NRESP]
   ERR_GETDATA_ASKS_TOO_FEW_BYTES                =109, //getData asks too few bytes                                [SerialIO] arg1=maxNumBytes arg2=input_buffer_fill_level
   ERR_INPUT_BUFFER_OVERFLOW                     =110, //input buffer overflow                                     [SerialIO]
   ERR_WRONG_CREATOR_COMPONENT                   =111, //wrong creator component                                   [IPHC] arg=creator_component
   ERR_WRONG_TRAN_PROTOCOL                       =112, //wrong transport protocol                                  [App] arg=tran_protocol
   ERR_WRONG_TCP_STATE                           =113, //wrong TCP state                                           [TCP] arg=state arg2=location
   ERR_RESET                                     =114, //TCP reset                                                 [TCP] arg=state arg2=location
   ERR_BRIDGE_MISMATCH                           =115, //isBridge mismatch                                         [NRES] arg1=code_location
   ERR_HEADER_TOO_LONG                           =116, //header too long                                           [PacketFunctions] arg1=code_location
   ERR_UNSUPPORTED_PORT_NUMBER                   =117, //unsupported port number                                   [all Apps and transport protocols] arg1=portNumber
   ERR_INPUTBUFFER_LENGTH                        =118, //input length problem                                      [openSerial, all components which get Triggered] arg1=input_buffer_length arg2=location
   ERR_DATAFRAMETOSEND_ERROR                     =119, //dataFrameToSend is/is not NULL when it should be          [stupidMAC] arg1=code_location
   ERR_BUSY_RECEIVING                            =120, //busy receiving                                            [stupidMAC]
   ERR_BUSY_SENDDONE                             =121, //busy in SenDone                                           [IEEE802154E] arg1=state arg2=slotOffset
};
typedef nx_struct TokenRequestMsg {
	nx_uint32_t criticalTime;
	nx_uint16_t tokenId;

}TokenRequestMsg;

typedef nx_struct TokenTransferredMsg {
	nx_uint8_t cmd;

	nx_uint16_t tokenId;
	nx_uint16_t destinationId;
	nx_uint16_t temperature;
}TokenTransferredMsg;

enum {
AM_TOKENREQUESTMSG = 14,
	AM_TOKENTRANSFERREDMSG = 13,

	// CMD OPTIONS

	CHANGE_COMMAND = 1,
	START_ADV = 2,
	ACK_COMMAND = 3,
	START_DATA = 6,
	SYNC_DONE=7,
};
#endif
