#ifndef __IEEE802154_H__
#define __IEEE802154_H__

enum {
   TIME_LIMITED_RX       =   TRUE,
   INFITE_RX             =  FALSE,
};

/*----------------------------- IEEE802.15.4  -------------------------------------------*/

typedef struct ieee802154_header_iht { //iht for "internal header type"
   uint8_t     headerLength;    //including the length field
   uint8_t     frameType;
   bool        securityEnabled;
   bool        framePending;
   bool        ackRequested;
   bool        panIDCompression;
   uint8_t     dsn;
   //source and destination address modes contained in open_addr_t structs
   open_addr_t panid;
   open_addr_t dest;
   open_addr_t src;
} ieee802154_header_iht;

enum IEEE802154_fcf_enums {
   IEEE154_FCF_FRAME_TYPE              = 0,
   IEEE154_FCF_SECURITY_ENABLED        = 3,
   IEEE154_FCF_FRAME_PENDING           = 4,
   IEEE154_FCF_ACK_REQ                 = 5,
   IEEE154_FCF_INTRAPAN                = 6,
   IEEE154_FCF_DEST_ADDR_MODE          = 2,
   IEEE154_FCF_SRC_ADDR_MODE           = 6,
};

enum IEEE802154_fcf_type_enums {
   IEEE154_TYPE_BEACON                 = 0,
   IEEE154_TYPE_DATA                   = 1,
   IEEE154_TYPE_ACK                    = 2,
   IEEE154_TYPE_CMD                    = 3,
   IEEE154_TYPE_UNDEFINED              = 5,
};

enum IEEE802154_fcf_sec_enums {
   IEEE154_SEC_NO_SECURITY             = 0,
   IEEE154_SEC_YES_SECURITY            = 1,
};

enum IEEE802154_fcf_pending_enums {
   IEEE154_PENDING_NO_FRAMEPENDING     = 0,
   IEEE154_PENDING_YES_FRAMEPENDING    = 1,
};

enum IEEE802154_fcf_ack_enums {
   IEEE154_ACK_NO_ACK_REQ              = 0,
   IEEE154_ACK_YES_ACK_REQ             = 1,
};

enum IEEE802154_fcf_panid_enums {
   IEEE154_PANID_UNCOMPRESSED          = 0,
   IEEE154_PANID_COMPRESSED            = 1,
};

enum IEEE802154_fcf_addr_mode_enums {
   IEEE154_ADDR_NONE                   = 0,
   IEEE154_ADDR_SHORT                  = 2,
   IEEE154_ADDR_EXT                    = 3,
};

#endif
