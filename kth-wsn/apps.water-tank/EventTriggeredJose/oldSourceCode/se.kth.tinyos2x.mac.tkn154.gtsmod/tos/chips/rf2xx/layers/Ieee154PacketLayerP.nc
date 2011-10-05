/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE VANDERBILT UNIVERSITY BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE VANDERBILT
 * UNIVERSITY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE VANDERBILT UNIVERSITY SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE VANDERBILT UNIVERSITY HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Author: Miklos Maroti
 */

#include <Ieee154PacketLayer.h>

module Ieee154PacketLayerP
{
	provides 
	{
		interface Ieee154PacketLayer;
		interface Ieee154Packet;
		interface RadioPacket;
	}

	uses
	{
		interface ActiveMessageAddress;
		interface RadioPacket as SubPacket;
	}
}

implementation
{
/*----------------- Ieee154Packet -----------------*/

	enum
	{
		IEEE154_DATA_FRAME_MASK = (IEEE154_TYPE_MASK << IEEE154_FCF_FRAME_TYPE) 
			| (1 << IEEE154_FCF_INTRAPAN) 
			| (IEEE154_ADDR_MASK << IEEE154_FCF_DEST_ADDR_MODE) 
			| (IEEE154_ADDR_MASK << IEEE154_FCF_SRC_ADDR_MODE),

		IEEE154_DATA_FRAME_VALUE = (IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) 
			| (1 << IEEE154_FCF_INTRAPAN) 
			| (IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE) 
			| (IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE),

		IEEE154_ACK_FRAME_LENGTH = 3,	// includes the FCF, DSN
		IEEE154_ACK_FRAME_MASK = (IEEE154_TYPE_MASK << IEEE154_FCF_FRAME_TYPE), 
		IEEE154_ACK_FRAME_VALUE = (IEEE154_TYPE_ACK << IEEE154_FCF_FRAME_TYPE),
	};

	ieee154_header_t* getHeader(message_t* msg)
	{
		return ((void*)msg) + call SubPacket.headerLength(msg);
	}

	void* getPayload(message_t* msg)
	{
		return ((void*)msg) + call RadioPacket.headerLength(msg);
	}

	async command uint16_t Ieee154PacketLayer.getFCF(message_t* msg)
	{
		return getHeader(msg)->fcf;
	}

	async command void Ieee154PacketLayer.setFCF(message_t* msg, uint16_t fcf)
	{
		getHeader(msg)->fcf = fcf;
	}

	async command bool Ieee154PacketLayer.isDataFrame(message_t* msg)
	{
		return (getHeader(msg)->fcf & IEEE154_DATA_FRAME_MASK) == IEEE154_DATA_FRAME_VALUE;
	}

	async command void Ieee154PacketLayer.createDataFrame(message_t* msg)
	{
		getHeader(msg)->fcf = IEEE154_DATA_FRAME_VALUE;
	}

	async command bool Ieee154PacketLayer.isAckFrame(message_t* msg)
	{
		return (getHeader(msg)->fcf & IEEE154_ACK_FRAME_MASK) == IEEE154_ACK_FRAME_VALUE;
	}

	async command void Ieee154PacketLayer.createAckFrame(message_t* msg)
	{
		call SubPacket.setPayloadLength(msg, IEEE154_ACK_FRAME_LENGTH);
		getHeader(msg)->fcf = IEEE154_ACK_FRAME_VALUE;
	}

	async command void Ieee154PacketLayer.createAckReply(message_t* data, message_t* ack)
	{
		ieee154_header_t* header = getHeader(ack);
		call SubPacket.setPayloadLength(ack, IEEE154_ACK_FRAME_LENGTH);

		header->fcf = IEEE154_ACK_FRAME_VALUE;
		header->dsn = getHeader(data)->dsn;
	}

	async command bool Ieee154PacketLayer.verifyAckReply(message_t* data, message_t* ack)
	{
		ieee154_header_t* header = getHeader(ack);

		return header->dsn == getHeader(data)->dsn
			&& (header->fcf & IEEE154_ACK_FRAME_MASK) == IEEE154_ACK_FRAME_VALUE;
	}

	async command bool Ieee154PacketLayer.getAckRequired(message_t* msg)
	{
		return getHeader(msg)->fcf & (1 << IEEE154_FCF_ACK_REQ);
	}

	async command void Ieee154PacketLayer.setAckRequired(message_t* msg, bool ack)
	{
		if( ack )
			getHeader(msg)->fcf |= (1 << IEEE154_FCF_ACK_REQ);
		else
			getHeader(msg)->fcf &= ~(uint16_t)(1 << IEEE154_FCF_ACK_REQ);
	}

	async command bool Ieee154PacketLayer.getFramePending(message_t* msg)
	{
		return getHeader(msg)->fcf & (1 << IEEE154_FCF_FRAME_PENDING);
	}

	async command void Ieee154PacketLayer.setFramePending(message_t* msg, bool pending)
	{
		if( pending )
			getHeader(msg)->fcf |= (1 << IEEE154_FCF_FRAME_PENDING);
		else
			getHeader(msg)->fcf &= ~(uint16_t)(1 << IEEE154_FCF_FRAME_PENDING);
	}

	async command uint8_t Ieee154PacketLayer.getDSN(message_t* msg)
	{
		return getHeader(msg)->dsn;
	}

	async command void Ieee154PacketLayer.setDSN(message_t* msg, uint8_t dsn)
	{
		getHeader(msg)->dsn = dsn;
	}

	async command uint16_t Ieee154PacketLayer.getDestPan(message_t* msg)
	{
		return getHeader(msg)->destpan;
	}

	async command void Ieee154PacketLayer.setDestPan(message_t* msg, uint16_t pan)
	{
		getHeader(msg)->destpan = pan;
	}

	async command uint16_t Ieee154PacketLayer.getDestAddr(message_t* msg)
	{
		return getHeader(msg)->dest;
	}

	async command void Ieee154PacketLayer.setDestAddr(message_t* msg, uint16_t addr)
	{
		getHeader(msg)->dest = addr;
	}

	async command uint16_t Ieee154PacketLayer.getSrcAddr(message_t* msg)
	{
		return getHeader(msg)->src;
	}

	async command void Ieee154PacketLayer.setSrcAddr(message_t* msg, uint16_t addr)
	{	
		getHeader(msg)->src = addr;
	}

	async command bool Ieee154PacketLayer.requiresAckWait(message_t* msg)
	{
		return call Ieee154PacketLayer.getAckRequired(msg)
			&& call Ieee154PacketLayer.isDataFrame(msg)
			&& call Ieee154PacketLayer.getDestAddr(msg) != 0xFFFF;
	}

	async command bool Ieee154PacketLayer.requiresAckReply(message_t* msg)
	{
		return call Ieee154PacketLayer.getAckRequired(msg)
			&& call Ieee154PacketLayer.isDataFrame(msg)
			&& call Ieee154PacketLayer.getDestAddr(msg) == call ActiveMessageAddress.amAddress();
	}

	async event void ActiveMessageAddress.changed()
	{
	}

/*----------------- Ieee154Packet -----------------*/

	command ieee154_saddr_t Ieee154Packet.address()
	{
		return call ActiveMessageAddress.amAddress();
	}

	command ieee154_saddr_t Ieee154Packet.destination(message_t* msg)
	{
		return call Ieee154PacketLayer.getDestAddr(msg);
	}
 
	command ieee154_saddr_t Ieee154Packet.source(message_t* msg)
	{
		return call Ieee154PacketLayer.getSrcAddr(msg);
	}

	command void Ieee154Packet.setDestination(message_t* msg, ieee154_saddr_t addr)
	{
		call Ieee154PacketLayer.setDestAddr(msg, addr);
	}

	command void Ieee154Packet.setSource(message_t* msg, ieee154_saddr_t addr)
	{
		call Ieee154PacketLayer.setSrcAddr(msg, addr);
	}

	command bool Ieee154Packet.isForMe(message_t* msg)
	{
		ieee154_saddr_t addr = call Ieee154Packet.destination(msg);
		return addr == call Ieee154Packet.address() || addr == IEEE154_BROADCAST_ADDR;
	}

	command ieee154_panid_t Ieee154Packet.pan(message_t* msg)
	{
		return call Ieee154PacketLayer.getDestPan(msg);
	}

	command void Ieee154Packet.setPan(message_t* msg, ieee154_panid_t grp)
	{
		call Ieee154PacketLayer.setDestPan(msg, grp);
	}

	command ieee154_panid_t Ieee154Packet.localPan()
	{
		return call ActiveMessageAddress.amGroup();
	}

/*----------------- RadioPacket -----------------*/

	async command uint8_t RadioPacket.headerLength(message_t* msg)
	{
		return call SubPacket.headerLength(msg) + sizeof(ieee154_header_t);
	}

	async command uint8_t RadioPacket.payloadLength(message_t* msg)
	{
		return call SubPacket.payloadLength(msg) - sizeof(ieee154_header_t);
	}

	async command void RadioPacket.setPayloadLength(message_t* msg, uint8_t length)
	{
		call SubPacket.setPayloadLength(msg, length + sizeof(ieee154_header_t));
	}

	async command uint8_t RadioPacket.maxPayloadLength()
	{
		return call SubPacket.maxPayloadLength() - sizeof(ieee154_header_t);
	}

	async command uint8_t RadioPacket.metadataLength(message_t* msg)
	{
		return call SubPacket.metadataLength(msg);
	}

	async command void RadioPacket.clear(message_t* msg)
	{
		call Ieee154PacketLayer.createDataFrame(msg);
		call SubPacket.clear(msg);
	}
}
