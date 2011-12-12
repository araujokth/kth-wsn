void prependIEEE802154header(OpenQueueEntry_t* msg,
      uint8_t      frameType,
      bool         securityEnabled,
      uint8_t      sequenceNumber,
      open_addr_t* nextHop) {
   uint8_t temp_8b;
   //previousHop address
   if (frameType!=IEEE154_TYPE_ACK) {
      switch (nextHop->type) {
         case ADDR_16B:
            call PacketFunctions.writeAddress(msg,call IDManager.getMyID(ADDR_16B),LITTLE_ENDIAN);
            break;
         case ADDR_64B:
            call PacketFunctions.writeAddress(msg,call IDManager.getMyID(ADDR_64B),LITTLE_ENDIAN);
            break;
         default:
//            call OpenSerial.printError(COMPONENT_MAC,ERR_WRONG_ADDR_TYPE,
//                  (errorparameter_t)nextHop->type,
//                  (errorparameter_t)0);
      }
   }
   //nextHop address
   if (frameType!=IEEE154_TYPE_ACK) {
      if (call PacketFunctions.isBroadcastMulticast(nextHop)) { //broadcast address is always 16-bit
         call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
         *((uint8_t*)(msg->payload)) = 0xFF;
         call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
         *((uint8_t*)(msg->payload)) = 0xFF;
      } else {
         switch (nextHop->type) {
            case ADDR_16B:
            case ADDR_64B:
               call PacketFunctions.writeAddress(msg,nextHop,LITTLE_ENDIAN);
               break;
            default:
//               call OpenSerial.printError(COMPONENT_MAC,ERR_WRONG_ADDR_TYPE,
//                     (errorparameter_t)nextHop->type,
//                     (errorparameter_t)1);
         }
      }
   }
   //destpan
   if (frameType!=IEEE154_TYPE_ACK) {
      call PacketFunctions.writeAddress(msg,call IDManager.getMyID(ADDR_PANID),LITTLE_ENDIAN);
   }
   //dsn
   call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
   *((uint8_t*)(msg->payload)) = sequenceNumber;
   //fcf (2nd byte)
   temp_8b              = 0;
   if (frameType==IEEE154_TYPE_ACK) {
      temp_8b          |= IEEE154_ADDR_NONE               << IEEE154_FCF_DEST_ADDR_MODE;
   } else if (call PacketFunctions.isBroadcastMulticast(nextHop)) {
      temp_8b          |= IEEE154_ADDR_SHORT              << IEEE154_FCF_DEST_ADDR_MODE;
   } else {
      switch (nextHop->type) {
         case ADDR_16B:
            temp_8b    |= IEEE154_ADDR_SHORT              << IEEE154_FCF_DEST_ADDR_MODE;
            break;
         case ADDR_64B:
            temp_8b    |= IEEE154_ADDR_EXT                << IEEE154_FCF_DEST_ADDR_MODE;
            break;
      }
   }
   if (frameType==IEEE154_TYPE_ACK) {
      temp_8b          |= IEEE154_ADDR_NONE               << IEEE154_FCF_SRC_ADDR_MODE;
   } else {
      switch (nextHop->type) {//normal: SRC_ADDR_MODE is the same as DEST_ADDR_MODE
         case ADDR_16B:
            temp_8b    |= IEEE154_ADDR_SHORT              << IEEE154_FCF_SRC_ADDR_MODE;
            break;
         case ADDR_64B:
            temp_8b    |= IEEE154_ADDR_EXT                << IEEE154_FCF_SRC_ADDR_MODE;
            break;
      }
   }
   call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
   *((uint8_t*)(msg->payload)) = temp_8b;
   temp_8b              = 0;
   temp_8b             |= frameType                       << IEEE154_FCF_FRAME_TYPE;
   temp_8b             |= securityEnabled                 << IEEE154_FCF_SECURITY_ENABLED;
   temp_8b             |= IEEE154_PENDING_NO_FRAMEPENDING << IEEE154_FCF_FRAME_PENDING;
   if (frameType==IEEE154_TYPE_ACK || (call PacketFunctions.isBroadcastMulticast(nextHop))) {
      temp_8b          |= IEEE154_ACK_NO_ACK_REQ          << IEEE154_FCF_ACK_REQ;
   } else {
      temp_8b          |= IEEE154_ACK_YES_ACK_REQ         << IEEE154_FCF_ACK_REQ;
   }
   temp_8b             |= IEEE154_PANID_COMPRESSED        << IEEE154_FCF_INTRAPAN;
   call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
   *((uint8_t*)(msg->payload)) = temp_8b;
   //2 bytes at the end for 15.4 footer (CRC to be filled in by radio)
   call PacketFunctions.reserveFooterSize(msg,2);
   //1 byte at the beginning for length field ()
   call PacketFunctions.reserveHeaderSize(msg,sizeof(uint8_t));
   *((uint8_t*)(msg->payload)) = msg->length-1;//the length of the length field excluded
}

ieee802154_header_iht retrieveIEEE802154header(OpenQueueEntry_t* msg) {
   ieee802154_header_iht ieee802514_header;
   uint8_t temp_8b;
   uint8_t offset = 0;
   //length (we don't use it)
   offset += 1;
   //fcf
   temp_8b = *((uint8_t*)(msg->payload)+offset);
   ieee802514_header.frameType         = (temp_8b >> IEEE154_FCF_FRAME_TYPE      ) & 0x07;//3b
   ieee802514_header.securityEnabled   = (temp_8b >> IEEE154_FCF_SECURITY_ENABLED) & 0x01;//1b
   ieee802514_header.framePending      = (temp_8b >> IEEE154_FCF_FRAME_PENDING   ) & 0x01;//1b
   ieee802514_header.ackRequested      = (temp_8b >> IEEE154_FCF_ACK_REQ         ) & 0x01;//1b
   ieee802514_header.panIDCompression  = (temp_8b >> IEEE154_FCF_INTRAPAN        ) & 0x01;//1b
   offset += 1;
   temp_8b = *((uint8_t*)(msg->payload)+offset);
   switch ( (temp_8b >> IEEE154_FCF_DEST_ADDR_MODE ) & 0x03 ) {
      case IEEE154_ADDR_NONE:
         ieee802514_header.dest.type = ADDR_NONE;
         break;
      case IEEE154_ADDR_SHORT:
         ieee802514_header.dest.type = ADDR_16B;
         break;
      case IEEE154_ADDR_EXT:
         ieee802514_header.dest.type = ADDR_64B;
         break;
      default:
//         call OpenSerial.printError(COMPONENT_MAC,ERR_IEEE154_UNSUPPORTED,
//               (errorparameter_t)1,
//               (errorparameter_t)(temp_8b >> IEEE154_FCF_DEST_ADDR_MODE ) & 0x03);
         break;
   }
   switch ( (temp_8b >> IEEE154_FCF_SRC_ADDR_MODE ) & 0x03 ) {
      case IEEE154_ADDR_NONE:
         ieee802514_header.src.type = ADDR_NONE;
         break;
      case IEEE154_ADDR_SHORT:
         ieee802514_header.src.type = ADDR_16B;
         break;
      case IEEE154_ADDR_EXT:
         ieee802514_header.src.type = ADDR_64B;
         break;
      default:
//         call OpenSerial.printError(COMPONENT_MAC,ERR_IEEE154_UNSUPPORTED,
//               (errorparameter_t)2,
//               (errorparameter_t)(temp_8b >> IEEE154_FCF_SRC_ADDR_MODE ) & 0x03);
         break;
   }
   offset += 1;
   //sequenceNumber
   ieee802514_header.dsn  = *((uint8_t*)(msg->payload)+offset);
   offset += 1;
   //panID
   if (ieee802514_header.frameType == IEEE154_TYPE_ACK) {
      ieee802514_header.panid.type = ADDR_NONE;
   } else {
      call PacketFunctions.readAddress(((uint8_t*)(msg->payload)+offset),ADDR_PANID,&ieee802514_header.panid,LITTLE_ENDIAN);
      offset += 2;
   }
   //dest
   switch (ieee802514_header.dest.type) {
      case ADDR_NONE:
         break;
      case ADDR_16B:
         call PacketFunctions.readAddress(((uint8_t*)(msg->payload)+offset),ADDR_16B,&ieee802514_header.dest,LITTLE_ENDIAN);
         offset += 2;
         break;
      case ADDR_64B:
         call PacketFunctions.readAddress(((uint8_t*)(msg->payload)+offset),ADDR_64B,&ieee802514_header.dest,LITTLE_ENDIAN);
         offset += 8;
         break;
   }
   //src
   switch (ieee802514_header.src.type) {
      case ADDR_NONE:
         break;
      case ADDR_16B:
         call PacketFunctions.readAddress(((uint8_t*)(msg->payload)+offset),ADDR_16B,&ieee802514_header.src,LITTLE_ENDIAN);
         offset += 2;
         break;
      case ADDR_64B:
         call PacketFunctions.readAddress(((uint8_t*)(msg->payload)+offset),ADDR_64B,&ieee802514_header.src,LITTLE_ENDIAN);
         offset += 8;
         break;
   }
   ieee802514_header.headerLength = offset;
   return ieee802514_header;
}
