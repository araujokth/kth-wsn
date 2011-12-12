#ifndef OPENWSN_H 
#define OPENWSN_H


//============================================ CONSTANTS =============================================

//============================================ Layer 2 Constants =====================================

// NRESP State

enum {
	ScanWait = 0,
	ScanSend = 1,
	AdvWait = 2,
	ReqSend = 3,
	AdvSend = 4,
	Build = 5,
	BuildDone = 6,
};

// NRESP Messages

enum {
	SCAN_MSG = 0x09,
	ADV_MSG = 0x0a,
	REQ_MSG = 0x0b,
	CONF_MSG = 0x0c,
	SEED_MSG = 0x0d,
};

enum {
	BUILDING = 0, SCHEDULING = 1,
};

// Slot Types

enum {
	UPLINK = 0, DOWNLINK = 1,
};

// Misc Layer 2 Constants

enum {
	HOPPING_ENABLED = TRUE,
	MAXNUMNEIGHBORS = 10,
	NUMCHANNELS = 4,
	MAXSCANSAMPLE = 50,
	MAXADVSAMPLE = 50,
	TXRETRIES = 3,
	DEFAULT_SEED = 255,
};

//====================================================================================================


//============================================ Layer 1 Constants =====================================


// Slot Radio States

enum {
	S_SYNCHRONIZING = 0,
	S_TX_TXDATAPREPARE = 1,
	S_TX_TXDATAREADY = 2,
	S_TX_TXDATA = 3,
	S_TX_RXACKPREPARE = 4,
	S_TX_RXACKREADY = 5,
	S_TX_RXACK = 6,
	S_RX_RXDATAPREPARE = 7,
	S_RX_RXDATAREADY = 8,
	S_RX_RXDATA = 9,
	S_RX_TXACKPREPARE = 10,
	S_RX_TXACKREADY = 11,
	S_RX_TXACK = 12,
	S_SLEEP = 13,
};

// Slot Timing

enum {
	TsTxOffset = 86,
	TsRxOffset = 44,
	TsRxWaitTime = 93,
	TsTxAckDelay = 64,
	TsRxAckDelay = 1,
	SLOT_TIME = 320,
	radio_delay = 17,
};

// Misc Layer 1 Constants

enum {
	TX_POWER = 10, // 1=-25dBm, 31=0dBm (max value)
	INVALID_TIMESTAMP = 0x80000000L,
};

enum {
	FRAME_BASED_RESYNC = TRUE, ACK_BASED_RESYNC = FALSE,
};

//======================================================================================================

//============================================ Misc Constants ==========================================

// Component Identifiers

enum {
	COMPONENT_NULL = 0,
	// Layer 7
	COMPONENT_APPDATA = 35,
	// Layer 2b
	COMPONENT_RES = 22,
	// Layer 2a
	COMPONENT_MAC = 25,
	// Layer 1
	COMPONENT_CC2420DRIVER = 26,
	COMPONENT_CC2420TRANSMIT = 27,
	COMPONENT_CC2420RECEIVE = 28,
	COMPONENT_CC2420CONTROL = 29,
	// Cross-layer
	COMPONENT_IDMANAGER = 30,
	COMPONENT_OPENQUEUE = 31,
	COMPONENT_PACKETFUNCTIONS = 33,
};

enum {
	QUEUELENGTH = 20,
};

enum {
	ADDR_NONE = 0,
	ADDR_16B = 1,
	ADDR_64B = 2,
	ADDR_128B = 3,
	ADDR_PANID = 4,
	ADDR_PREFIX = 5,
};

enum {
	LITTLE_ENDIAN = TRUE, BIG_ENDIAN = FALSE,
};

//======================================================================================================

//============================================ TYPES =================================================

typedef nx_struct CMD_MSG_t {
	nxle_uint16_t timingInformation;
	nxle_uint8_t commandFrameId;
	nxle_uint8_t seedNumber;
	nxle_uint8_t numHops;
	nxle_uint8_t numChildren;
	nxle_uint8_t seedList[MAXNUMNEIGHBORS];
}CMD_MSG_t;

typedef struct open_addr_t { //always written big endian, i.e. MSB in addr[0]
	uint8_t type;
	union {
		uint8_t addr_16b[2];
		uint8_t addr_64b[8];
		uint8_t addr_128b[16];
		uint8_t panid[2];
		uint8_t prefix[8];
	};
} open_addr_t;

// OpenQueue Entry Definition

typedef struct OpenQueueEntry_t {
	// Admin
	uint8_t creator; //the component which called getFreePacketBuffer()
	uint8_t owner; //the component which currently owns the entry
	void* payload; //pointer to the start of the payload within 'packet'
	uint8_t length; //length in bytes of the payload
	// Layer 3
	open_addr_t l3_destinationORsource; //128b IPv6 destination (down stack) or source address (up)
	// Layer 2
	open_addr_t l2_nextORpreviousHop; //64b IEEE802.15.4 next (down stack) or previous (up) hop address
	uint8_t l2_seedNumber;
	uint8_t l2_frameType; //beacon, data, ack, cmd
	uint8_t l2_commandFrameId; // Type of command: scan_msg, adv_msg, req_msg, conf_msg, seed_msg
	uint8_t l2_retriesLeft;
	bool l2_transmitInFirstSlot; //for video transmission
	// Layer 1 (drivers)
	uint8_t l1_txPower;
	uint8_t l1_channel;
	int8_t l1_rssi;
	uint8_t l1_lqi;
	bool l1_crc;
	uint32_t l1_rxTimestamp;
	// The packet
	uint8_t packet[124];
} OpenQueueEntry_t;

typedef uint16_t asn_t;
typedef uint32_t timervalue_t;

typedef struct neighborEntry_t {
   bool             used;
   bool             isChild;
   uint8_t          seedNumber;
   int8_t          	rssi;
   uint8_t          numHops;
   open_addr_t      addr_16b;
   open_addr_t      addr_64b;
   open_addr_t      addr_128b;

} neighborEntry_t;

typedef struct neighborList_t {
	uint8_t			numNeighbors;
	uint8_t			old_numNeighbors;
	uint8_t			counterNoNewNeighbors;
	uint8_t			numChildren;
	uint8_t			numSibling;
	uint8_t			old_numChildren;
	uint8_t			counterNoNewChildren;
	uint8_t			counterSeedMsgSent;
	uint8_t			seedList [MAXNUMNEIGHBORS];
	uint8_t			childrenIndex [MAXNUMNEIGHBORS];
	uint8_t			siblingList [MAXNUMNEIGHBORS];
	uint8_t			parent;
	uint8_t			minHops;
	int8_t			maxRssi;
	neighborEntry_t neighbors[MAXNUMNEIGHBORS];
} neighborList_t;

#endif
