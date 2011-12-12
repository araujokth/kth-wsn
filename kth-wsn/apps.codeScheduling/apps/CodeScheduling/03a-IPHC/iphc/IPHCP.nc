#include "LATIN.h"
#include "IPHC.h"

module IPHCP {
   //down the stack
   provides interface OpenSend as OpenSendFromUpper;
   uses     interface OpenSend as OpenSendToLower;
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower;
   uses     interface OpenReceiveIp;
   //misc
   uses     interface OpenSerial;
   uses     interface IDManager;
   uses     interface PacketFunctions;
   uses     interface NeighborGet;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   /*-------------------------------- prototypes ----------------------------------------*/

   error_t prependIPv6Header(OpenQueueEntry_t* msg,
         uint8_t tf,
         uint32_t value_flowLabel,
         bool nh,
         uint8_t value_nextHeader,
         uint8_t hlim,
         uint8_t value_hopLimit,
         bool cid,
         bool sac,
         uint8_t sam,
         bool m,
         bool dac,
         uint8_t dam,
         open_addr_t* value_dest);

   ipv6_header_iht retrieveIPv6Header(OpenQueueEntry_t* msg);

   /*-------------------------------- interfaces ----------------------------------------*/

   //OpenSendFromUpper
   command error_t OpenSendFromUpper.send(OpenQueueEntry_t *msg) {
      open_addr_t  temp_dest_prefix;
      open_addr_t  temp_dest_mac64b;
      open_addr_t  temp_dest_mac16b;
      open_addr_t* p_dest;
      uint8_t sam;
      uint8_t dam;
      msg->owner = COMPONENT_IPHC;
      call PacketFunctions.ip128bToMac64b(&(msg->l3_destinationORsource),&temp_dest_prefix,&temp_dest_mac64b);
      if (call PacketFunctions.sameAddress(&temp_dest_prefix,call IDManager.getMyID(ADDR_PREFIX))) {
         //dest and me on same prefix
         if (call NeighborGet.isStableNeighbor(&(msg->l3_destinationORsource))) {
            //if direct neighbors, no need to put destination address
            sam = IPHC_SAM_ELIDED;
            dam = IPHC_DAM_ELIDED;
            p_dest = NULL;
         } else {
            //else, use 16B address
            sam = IPHC_SAM_16B;
            dam = IPHC_DAM_16B;
            call PacketFunctions.mac64bToMac16b(&temp_dest_mac64b,&temp_dest_mac16b);
            p_dest = &temp_dest_mac16b;
         }
      } else {
         sam = IPHC_SAM_128B;
         dam = IPHC_DAM_128B;
         p_dest = &(msg->l3_destinationORsource);
      }
      if (prependIPv6Header(msg,
               IPHC_TF_ELIDED,
               0,//poipoi should value_flowlabel be copied?
               IPHC_NH_INLINE,
               msg->l4_protocol,
               IPHC_HLIM_INLINE,
               64,
               IPHC_CID_NO,
               IPHC_SAC_STATELESS,
               sam,
               IPHC_M_NO,
               IPHC_DAC_STATELESS,
               dam,
               p_dest
               )==FAIL) {
         return FAIL;
      }
      return call OpenSendToLower.send(msg);
   }

   //OpenSendToLower
   event void OpenSendToLower.sendDone(OpenQueueEntry_t* msg, error_t error) {
      msg->owner    = COMPONENT_IPHC;
      signal OpenSendFromUpper.sendDone(msg,error);
   }

   //OpenReceiveFromLower
   command void OpenReceiveFromLower.receive(OpenQueueEntry_t* msg) {
      ipv6_header_iht ipv6_header;
      msg->owner  = COMPONENT_IPHC;
      ipv6_header = retrieveIPv6Header(msg);
      call OpenReceiveIp.receive(msg,ipv6_header);
   }

   /*-------------------------------- helper functions ----------------------------------*/

   error_t prependIPv6Header(OpenQueueEntry_t* msg,
         uint8_t tf,
         uint32_t value_flowLabel,
         bool nh,
         uint8_t value_nextHeader,
         uint8_t hlim,
         uint8_t value_hopLimit,
         bool cid,
         bool sac,
         uint8_t sam,
         bool m,
         bool dac,
         uint8_t dam,
         open_addr_t* value_dest) {
      uint8_t temp_8b;
      //destination address
      switch (dam) {
         case IPHC_DAM_ELIDED:
            break;
         case IPHC_DAM_16B:
            if (value_dest->type!=ADDR_16B) {
               call OpenSerial.printError(COMPONENT_IPHC,ERR_WRONG_ADDR_TYPE,
                     (errorparameter_t)value_dest->type,
                     (errorparameter_t)0);
               return FAIL;
            };
            call PacketFunctions.writeAddress(msg,value_dest,BIG_ENDIAN);
            break;
         case IPHC_DAM_64B:
            if (value_dest->type!=ADDR_64B) {
               call OpenSerial.printError(COMPONENT_IPHC,ERR_WRONG_ADDR_TYPE,
                     (errorparameter_t)value_dest->type,
                     (errorparameter_t)1);
               return FAIL;
            };
            call PacketFunctions.writeAddress(msg,value_dest,BIG_ENDIAN);
            break;
         case IPHC_DAM_128B:
            if (value_dest->type!=ADDR_128B) {
               call OpenSerial.printError(COMPONENT_IPHC,ERR_WRONG_ADDR_TYPE,
                     (errorparameter_t)value_dest->type,
                     (errorparameter_t)2);
               return FAIL;
            };
            call PacketFunctions.writeAddress(msg,value_dest,BIG_ENDIAN);
            break;
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)0,(errorparameter_t)dam);
            return FAIL;
            break;
      }
      //source address
      switch (sam) {
         case IPHC_SAM_ELIDED:
            break;
         case IPHC_SAM_16B:
            call PacketFunctions.writeAddress(msg, (call IDManager.getMyID(ADDR_16B)),BIG_ENDIAN);
            break;
         case IPHC_SAM_64B:
            call PacketFunctions.writeAddress(msg, (call IDManager.getMyID(ADDR_64B)),BIG_ENDIAN);
            break;
         case IPHC_SAM_128B:
            call PacketFunctions.writeAddress(msg, (call IDManager.getMyID(ADDR_64B)),BIG_ENDIAN);
            call PacketFunctions.writeAddress(msg, (call IDManager.getMyID(ADDR_PREFIX)),BIG_ENDIAN);
            break;
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)1,(errorparameter_t)sam);
            return FAIL;
            break;
      }
      //hop limit
      switch (hlim) {
         case IPHC_HLIM_INLINE:
            call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
            *((uint8_t*)(msg->payload)) = value_hopLimit;
            break;
         case IPHC_HLIM_1:
         case IPHC_HLIM_64:
         case IPHC_HLIM_255:
            break;
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)2,(errorparameter_t)hlim);
            return FAIL;
            break;
      }
      //next header
      switch (nh) {
         case IPHC_NH_INLINE:
            call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
            *((uint8_t*)(msg->payload)) = value_nextHeader;
            break;
         case IPHC_NH_COMPRESSED:
            //unsupported
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)3,(errorparameter_t)nh);
            return FAIL;
            break;
      }
      //flowlabel
      switch (tf) {
         case IPHC_TF_3B:
            call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
            *((uint8_t*)(msg->payload)) = ((uint32_t)(value_flowLabel & 0x000000ff) >> 0);
            call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
            *((uint8_t*)(msg->payload)) = ((uint32_t)(value_flowLabel & 0x0000ff00) >> 8);
            call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
            *((uint8_t*)(msg->payload)) = ((uint32_t)(value_flowLabel & 0x00ff0000) >> 16);
            break;            
         case IPHC_TF_ELIDED:
            break;
         case IPHC_TF_4B:
            //unsupported
         case IPHC_TF_1B:
            //unsupported
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)4,(errorparameter_t)tf);
            return FAIL;
            break;
      }
      //header
      temp_8b  = 0;
      temp_8b |= cid                 << IPHC_CID;
      temp_8b |= sac                 << IPHC_SAC;
      temp_8b |= sam                 << IPHC_SAM;
      temp_8b |= m                   << IPHC_M;
      temp_8b |= dac                 << IPHC_DAC;
      temp_8b |= dam                 << IPHC_DAM;
      call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
      *((uint8_t*)(msg->payload)) = temp_8b;
      temp_8b  = 0;
      temp_8b |= IPHC_DISPATCH_IPHC  << IPHC_DISPATCH;
      temp_8b |= tf                  << IPHC_TF;
      temp_8b |= nh                  << IPHC_NH;
      temp_8b |= hlim                << IPHC_HLIM;
      call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
      *((uint8_t*)(msg->payload)) = temp_8b;
      return SUCCESS;
   }

   ipv6_header_iht retrieveIPv6Header(OpenQueueEntry_t* msg) {
      uint16_t temp_8b;
      open_addr_t temp_addr_16b;
      open_addr_t temp_addr_64b;
      ipv6_header_iht ipv6_header;
      uint8_t dispatch;
      uint8_t tf;
      bool    nh;
      uint8_t hlim;
      bool    cid;
      bool    sac;
      uint8_t sam;
      bool    m;
      bool    dac;
      uint8_t dam;
      //header
      temp_8b   = *((uint8_t*)(msg->payload));
      dispatch  = (temp_8b >> IPHC_DISPATCH)  & 0x07;//3b
      tf        = (temp_8b >> IPHC_TF)        & 0x03;//2b
      nh        = (temp_8b >> IPHC_NH)        & 0x01;//1b
      hlim      = (temp_8b >> IPHC_HLIM)      & 0x03;//2b
      call PacketFunctions.tossHeader(msg,sizeof(uint8_t));
      temp_8b   = *((uint8_t*)(msg->payload));
      cid       = (temp_8b >> IPHC_CID)       & 0x01;//1b
      sac       = (temp_8b >> IPHC_SAC)       & 0x01;//1b
      sam       = (temp_8b >> IPHC_SAM)       & 0x03;//2b
      m         = (temp_8b >> IPHC_M)         & 0x01;//1b
      dac       = (temp_8b >> IPHC_DAC)       & 0x01;//1b
      dam       = (temp_8b >> IPHC_DAM)       & 0x03;//2b
      call PacketFunctions.tossHeader(msg,sizeof(uint8_t));
      //dispatch
      switch (dispatch) {
         case IPHC_DISPATCH_IPHC:
            break;            
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)5,(errorparameter_t)tf);
            break;
      }
      //flowlabel
      switch (tf) {
         case IPHC_TF_3B:
            ipv6_header.flow_label  = ((uint32_t) *((uint8_t*)(msg->payload)))<<0;
            call PacketFunctions.tossHeader(msg,sizeof(uint8_t));
            ipv6_header.flow_label |= ((uint32_t) *((uint8_t*)(msg->payload)))<<8;
            call PacketFunctions.tossHeader(msg,sizeof(uint8_t));
            ipv6_header.flow_label |= ((uint32_t) *((uint8_t*)(msg->payload)))<<16;
            call PacketFunctions.tossHeader(msg,sizeof(uint8_t));
            break;            
         case IPHC_TF_ELIDED:
            ipv6_header.flow_label  = 0;
            break;
         case IPHC_TF_4B:
            //unsupported
         case IPHC_TF_1B:
            //unsupported
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)6,(errorparameter_t)tf);
            break;
      }
      //next header
      switch (nh) {
         case IPHC_NH_INLINE:
            ipv6_header.next_header = *((uint8_t*)(msg->payload));
            call PacketFunctions.tossHeader(msg,sizeof(uint8_t));
            break;
         case IPHC_NH_COMPRESSED:
            //unsupported
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)7,(errorparameter_t)nh);
            break;
      }
      //hop limit
      switch (hlim) {
         case IPHC_HLIM_INLINE:
            ipv6_header.hop_limit = *((uint8_t*)(msg->payload));
            call PacketFunctions.tossHeader(msg,sizeof(uint8_t));
            break;
         case IPHC_HLIM_1:
            ipv6_header.hop_limit = 1;
         case IPHC_HLIM_64:
            ipv6_header.hop_limit = 64;
         case IPHC_HLIM_255:
            ipv6_header.hop_limit = 255;
            break;
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)8,(errorparameter_t)hlim);
            break;
      }
      //source address
      switch (sam) {
         case IPHC_SAM_ELIDED:
            call PacketFunctions.mac64bToIp128b(
                  call IDManager.getMyID(ADDR_PREFIX),
                  &(msg->l2_nextORpreviousHop),
                  &ipv6_header.src);
            break;
         case IPHC_SAM_16B:
            call PacketFunctions.readAddress(((uint8_t*)(msg->payload)),ADDR_16B,&temp_addr_16b,BIG_ENDIAN);
            call PacketFunctions.tossHeader(msg,2*sizeof(uint8_t));
            call PacketFunctions.mac16bToMac64b(&temp_addr_16b,&temp_addr_64b);
            call PacketFunctions.mac64bToIp128b(call IDManager.getMyID(ADDR_PREFIX),&temp_addr_64b,&ipv6_header.src);
            break;
         case IPHC_SAM_64B:
            call PacketFunctions.readAddress(((uint8_t*)(msg->payload)),ADDR_64B,&temp_addr_64b,BIG_ENDIAN);
            call PacketFunctions.tossHeader(msg,8*sizeof(uint8_t));
            call PacketFunctions.mac64bToIp128b(call IDManager.getMyID(ADDR_PREFIX),&temp_addr_64b,&ipv6_header.src);
            break;
         case IPHC_SAM_128B:
            call PacketFunctions.readAddress(((uint8_t*)(msg->payload)),ADDR_128B,&ipv6_header.src,BIG_ENDIAN);
            call PacketFunctions.tossHeader(msg,16*sizeof(uint8_t));
            break;
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)9,(errorparameter_t)sam);
            break;
      }
      //destination address
      switch (dam) {
         case IPHC_DAM_ELIDED:
            call PacketFunctions.mac64bToIp128b(
                  call IDManager.getMyID(ADDR_PREFIX),
                  call IDManager.getMyID(ADDR_64B),
                  &(ipv6_header.dest));
            break;
         case IPHC_DAM_16B:
            call PacketFunctions.readAddress(((uint8_t*)(msg->payload)),ADDR_16B,&temp_addr_16b,BIG_ENDIAN);
            call PacketFunctions.tossHeader(msg,2*sizeof(uint8_t));
            call PacketFunctions.mac16bToMac64b(&temp_addr_16b,&temp_addr_64b);
            call PacketFunctions.mac64bToIp128b(call IDManager.getMyID(ADDR_PREFIX),&temp_addr_64b,&ipv6_header.dest);
            break;
         case IPHC_DAM_64B:
            call PacketFunctions.readAddress(((uint8_t*)(msg->payload)),ADDR_64B,&temp_addr_64b,BIG_ENDIAN);
            call PacketFunctions.tossHeader(msg,8*sizeof(uint8_t));
            call PacketFunctions.mac64bToIp128b(call IDManager.getMyID(ADDR_PREFIX),&temp_addr_64b,&ipv6_header.dest);
            break;
         case IPHC_DAM_128B:
            call PacketFunctions.readAddress(((uint8_t*)(msg->payload)),ADDR_128B,&ipv6_header.dest,BIG_ENDIAN);
            call PacketFunctions.tossHeader(msg,16*sizeof(uint8_t));
            break;
         default:
            call OpenSerial.printError(COMPONENT_IPHC,ERR_6LOWPAN_UNSUPPORTED,(errorparameter_t)10,(errorparameter_t)sam);
            break;
      }
      return ipv6_header;
   }

}
