#ifndef __IEEE802154E_H__
#define __IEEE802154E_H__

/*----------------------------- IEEE802.15.4E ACK ---------------------------------------*/

typedef nx_struct IEEE802154E_ACK_ht {
   nxle_uint8_t     dhrAckNack;
   nxle_uint16_t    timeCorrection;
} IEEE802154E_ACK_ht;

enum IEEE802154E_ACK_dhrAckNack_enums {
   IEEE154E_ACK_dhrAckNack_DEFAULT = 0x82,
};

/*----------------------------- IEEE802.15.4E ADV ---------------------------------------*/

typedef nx_struct IEEE802154E_ADV_t {
   nxle_uint8_t     commandFrameId;
   nxle_uint32_t    timingInformation;   //needs to be 6 bytes long
   nxle_uint8_t     securityControlField;
   nxle_uint8_t     joinControl;
   nxle_uint8_t     timeslotHopping;
   nxle_uint8_t     channelPage;
   nxle_uint32_t    channelMap;
   nxle_uint8_t     numberSlotFrames;
   nxle_uint8_t     slotFrameID;
   nxle_uint16_t    slotFrameSize;
   nxle_uint8_t     numberLinks;
   nxle_uint32_t    linkInfo1;
   nxle_uint32_t    linkInfo2;
   nxle_uint8_t     DAGrank;
} IEEE802154E_ADV_t;

enum ieee154e_commandFrameId_enums {
   IEEE154E_ADV      = 0x0a,
   IEEE154E_JOIN     = 0x0b,
   IEEE154E_ACTIVATE = 0x0c,
};

enum ieee154e_ADV_defaults_enums {
   IEEE154E_ADV_SEC_DEFAULT           = 0x00,
   IEEE154E_ADV_JOINCONTROL_DEFAULT   = 0x00,
   IEEE154E_ADV_HOPPING_DEFAULT       = 0x00,
   IEEE154E_ADV_CHANNELPAGE_DEFAULT   = 0x04,
   IEEE154E_ADV_CHANNELMAP_DEFAULT    = 0x07FFF800,
   IEEE154E_ADV_NUMSLOTFRAMES_DEFAULT = 0x01,
   IEEE154E_ADV_SLOTFRAMEID_DEFAULT   = 0x00,
   IEEE154E_ADV_SLOTFRAMESIZE_DEFAULT = 0x001F,
   IEEE154E_ADV_NUMLINKS_DEFAULT      = 0x02,
   IEEE154E_ADV_LINKINFO1_DEFAULT     = 0x00000002,
   IEEE154E_ADV_LINKINFO2_DEFAULT     = 0x00010001,
};

//slot states
enum {
	//synchronizing
	S_SYNCHRONIZING = 0,
	//transmitter
	S_TX_TXDATAPREPARE = 1,
	S_TX_TXDATAREADY = 2,
	S_TX_TXDATA = 3,
	S_TX_RXACKPREPARE = 4,
	S_TX_RXACKREADY = 5,
	S_TX_RXACK = 6,
	//receiver
	S_RX_RXDATAPREPARE = 7,
	S_RX_RXDATAREADY = 8,
	S_RX_RXDATA = 9,
	S_RX_TXACKPREPARE = 10,
	S_RX_TXACKREADY = 11,
	S_RX_TXACK = 12,
	//cooldown
	S_SLEEP = 13,
};

enum {
	FRAME_BASED_RESYNC = TRUE, ACK_BASED_RESYNC = FALSE,
};

enum {
	WAS_ACKED = TRUE, WAS_NOT_ACKED = FALSE,
};

#endif
