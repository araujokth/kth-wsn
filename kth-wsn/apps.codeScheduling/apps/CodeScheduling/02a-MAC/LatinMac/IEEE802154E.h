#ifndef __IEEE802154E_H__
#define __IEEE802154E_H__

/*----------------------------- IEEE802.15.4E ACK ---------------------------------------*/

typedef nx_struct IEEE802154E_ACK_ht {
	nxle_uint8_t dhrAckNack;
	nxle_uint16_t timeCorrection;
}IEEE802154E_ACK_ht;

enum IEEE802154E_ACK_dhrAckNack_enums {
	IEEE154E_ACK_dhrAckNack_DEFAULT = 0x82,
};

#endif
