#include "LATIN.h"
#include "UserButton.h"
#include "IPHC.h" //for IPv6 next header

module AppTcpOhloneP {
   //admin
   provides interface Init as SoftwareInit;
   //data flow
   uses interface TCPControl[uint16_t localPortNumber];
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   provides interface OpenReceive as OpenReceiveFromLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   uses interface PacketFunctions;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   OpenQueueEntry_t* pkt;
   bool              sending;
   uint8_t           httpReplyHeader[19];

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskHTTPReply();
   task void taskCloseTCPSession();

   /*-------------------------------- http receive sequence -----------------------------*/

   event bool TCPControl.shouldIlisten[uint16_t localPortNumber](){
      return TRUE;
   }

   command void OpenReceiveFromLower.receive[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      uint8_t payload_index;
      for (payload_index=0;payload_index<msg->length-3;payload_index++){
         if(   *((uint8_t*)(msg->payload+payload_index  ))=='\r' &&
               *((uint8_t*)(msg->payload+payload_index+1))=='\n' &&
               *((uint8_t*)(msg->payload+payload_index+2))=='\r' &&
               *((uint8_t*)(msg->payload+payload_index+3))=='\n') {
            post taskHTTPReply();
         }
      }
      call OpenSerial.printData((uint8_t*)(msg->payload),msg->length);
      call Malloc.freePacketBuffer(msg);
   }
   default command error_t OpenSendToLower.send[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_APPTCPOHLONE,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,0);
      return FAIL;
   }

   task void taskHTTPReply() {
      pkt = call Malloc.getFreePacketBuffer();
      if (pkt==NULL) {
         call OpenSerial.printError(COMPONENT_APPTCPOHLONE,ERR_NO_FREE_PACKET_BUFFER,(errorparameter_t)0,(errorparameter_t)0);
         return;
      }
      pkt->creator             = COMPONENT_APPTCPOHLONE;
      pkt->owner               = COMPONENT_APPTCPOHLONE;
      call PacketFunctions.reserveHeaderSize(pkt,sizeof(httpReplyHeader)+2);
      memcpy(pkt->payload,&httpReplyHeader,sizeof(httpReplyHeader));
      *((uint8_t*)(pkt->payload+sizeof(httpReplyHeader)+0)) = ':';
      *((uint8_t*)(pkt->payload+sizeof(httpReplyHeader)+1)) = ')';
      if ((call OpenSendToLower.send[WKP_TCP_HTTP](pkt))==FAIL) {
         call Malloc.freePacketBuffer(pkt);
      }
      return;
   }

   event void OpenSendToLower.sendDone[uint16_t localPortNumber](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_APPTCPOHLONE;
      if (msg->creator!=COMPONENT_APPTCPOHLONE) {
         call OpenSerial.printError(COMPONENT_APPTCPOHLONE,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      post taskCloseTCPSession();
      call Malloc.freePacketBuffer(msg);
   }

   task void taskCloseTCPSession() {
      //close TCP session, but keep listening
      call TCPControl.close[WKP_TCP_HTTP]();
   }

   default command error_t TCPControl.close[uint16_t localPortNumber]() {
      call OpenSerial.printError(COMPONENT_APPTCPOHLONE,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,1);
      return FAIL;
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      httpReplyHeader[0]  = 'H';
      httpReplyHeader[1]  = 'T';
      httpReplyHeader[2]  = 'T';
      httpReplyHeader[3]  = 'P';
      httpReplyHeader[4]  = '/';
      httpReplyHeader[5]  = '1';
      httpReplyHeader[6]  = '.';
      httpReplyHeader[7]  = '1';
      httpReplyHeader[8]  = ' ';
      httpReplyHeader[9]  = '2';
      httpReplyHeader[10] = '0';
      httpReplyHeader[11] = '0';
      httpReplyHeader[12] = ' ';
      httpReplyHeader[13] = 'O';
      httpReplyHeader[14] = 'K';
      httpReplyHeader[15] = '\r';
      httpReplyHeader[16] = '\n';
      httpReplyHeader[17] = '\r';
      httpReplyHeader[18] = '\n';
      return SUCCESS;
   }

   event void TCPControl.connectDone[uint16_t localPortNumber](error_t error) {
   }
}
