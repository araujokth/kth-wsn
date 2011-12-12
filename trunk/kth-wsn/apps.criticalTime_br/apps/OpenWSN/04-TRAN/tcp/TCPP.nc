#include "OpenWSN.h"
#include "IPHC.h"
#include "tcp.h"

module TCPP {
   //down the stack
   provides interface Init as SoftwareInit;
   provides interface TCPControl[uint16_t localPortNumber];
   provides interface OpenSend as OpenSendFromUpper[uint16_t localPortNumber];
   uses     interface OpenSend as OpenSendToLower[uint8_t iana_number];
   //up the stack
   provides interface OpenReceive as OpenReceiveFromLower[uint8_t iana_number];
   uses     interface OpenReceive as OpenReceiveToUpper[uint16_t localPortNumber];
   //misc
   uses     interface PacketFunctions;
   uses     interface IDManager;
   uses     interface OpenSerial;
   uses     interface Malloc;
}
implementation {

   //See OpenTcp for state machine and documentation.

   /*-------------------------------- variables -----------------------------------------*/

   uint8_t      state;
   uint32_t     mySeqNum;
   uint16_t     myPort;
   uint32_t     hisNextSeqNum;
   uint16_t     hisPort;
   open_addr_t  hisIPv6Address;

   OpenQueueEntry_t* dataToSend;
   OpenQueueEntry_t* dataReceived;

   /*-------------------------------- prototypes ----------------------------------------*/

   void prependTCPHeader(OpenQueueEntry_t* msg, bool ack, bool push, bool rst, bool syn, bool fin);
   bool containsControlBits(OpenQueueEntry_t* msg, uint8_t ack, uint8_t rst, uint8_t syn, uint8_t fin);
   void reset();

   /*-------------------------------- interfaces -------------------------------------*/

   command error_t SoftwareInit.init() {
      state          = TCP_STATE_CLOSED;
      mySeqNum       = TCP_INITIAL_SEQNUM;
      hisNextSeqNum  = 0;
      hisPort        = 0;
      dataToSend     = NULL;
      dataReceived   = NULL;
      return SUCCESS;
   }

   command error_t TCPControl.connect[uint16_t localPortNumber](open_addr_t* dest, uint16_t hisport) {
      //[command] establishment
      OpenQueueEntry_t* tempPkt;
      if (state!=TCP_STATE_CLOSED) {
         call OpenSerial.printError(COMPONENT_TCP,ERR_WRONG_TCP_STATE,(errorparameter_t)state,(errorparameter_t)0);
         return FAIL;
      }
      myPort  = localPortNumber;
      hisPort = hisport;
      memcpy(&hisIPv6Address,dest,sizeof(open_addr_t));
      //I receive command 'connect', I send SYNC
      tempPkt = call Malloc.getFreePacketBuffer();
      if (tempPkt==NULL) {
         call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
         return FAIL;
      }
      tempPkt->creator                = COMPONENT_TCP;
      tempPkt->owner                  = COMPONENT_TCP;
      memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
      mySeqNum = TCP_INITIAL_SEQNUM;
      prependTCPHeader(tempPkt,
            TCP_ACK_NO,
            TCP_PSH_NO,
            TCP_RST_NO,
            TCP_SYN_YES,
            TCP_FIN_NO);
      state = TCP_STATE_ALMOST_SYN_SENT;
      return call OpenSendToLower.send[IANA_TCP](tempPkt);
   }

   command error_t TCPControl.close[uint16_t localPortNumber]() {    //[command] teardown
      OpenQueueEntry_t* tempPkt;
      if (  state==TCP_STATE_ALMOST_CLOSE_WAIT ||
            state==TCP_STATE_CLOSE_WAIT        ||
            state==TCP_STATE_ALMOST_LAST_ACK   ||
            state==TCP_STATE_LAST_ACK          ||
            state==TCP_STATE_CLOSED) {
         //call OpenSerial.printError(COMPONENT_TCP,ERR_WRONG_TCP_STATE,(errorparameter_t)state,(errorparameter_t)1);
         //not an error, can happen when distant node has already started tearing down
         return EALREADY;
      }
      //I receive command 'close', I send FIN+ACK
      tempPkt = call Malloc.getFreePacketBuffer();
      if (tempPkt==NULL) {
         call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
         return FAIL;
      }
      tempPkt->creator       = COMPONENT_TCP;
      tempPkt->owner         = COMPONENT_TCP;
      memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
      prependTCPHeader(tempPkt,
            TCP_ACK_YES,
            TCP_PSH_NO,
            TCP_RST_NO,
            TCP_SYN_NO,
            TCP_FIN_YES);
      mySeqNum++;
      state = TCP_STATE_ALMOST_FIN_WAIT_1;
      return call OpenSendToLower.send[IANA_TCP](tempPkt);
   }

   command error_t OpenSendFromUpper.send[uint16_t localPortNumber](OpenQueueEntry_t* msg) {             //[command] data
      msg->owner                = COMPONENT_TCP;
      if (state!=TCP_STATE_ESTABLISHED) {
         call OpenSerial.printError(COMPONENT_TCP,ERR_WRONG_TCP_STATE,(errorparameter_t)state,(errorparameter_t)2);
         signal OpenSendFromUpper.sendDone[myPort](dataToSend,ERESERVE);
         return FAIL;
      }
      if (dataToSend!=NULL) {
         call OpenSerial.printError(COMPONENT_TCP,ERR_BUSY_SENDING,(errorparameter_t)0,(errorparameter_t)0);
         return FAIL;
      }
      //I receive command 'send', I send data
      msg->l4_protocol          = IANA_TCP;
      msg->l4_sourcePortORicmpv6Type       = myPort;
      msg->l4_destination_port  = hisPort;
      msg->l4_payload           = msg->payload;
      msg->l4_length            = msg->length;
      memcpy(&(msg->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
      dataToSend = msg;
      prependTCPHeader(dataToSend,
            TCP_ACK_YES,
            TCP_PSH_YES,
            TCP_RST_NO,
            TCP_SYN_NO,
            TCP_FIN_NO);
      mySeqNum += dataToSend->l4_length;
      state = TCP_STATE_ALMOST_DATA_SENT;
      return call OpenSendToLower.send[IANA_TCP](dataToSend);
   }

   event void OpenSendToLower.sendDone[uint8_t iana_number](OpenQueueEntry_t* msg, error_t error) {
      OpenQueueEntry_t* tempPkt;
      msg->owner = COMPONENT_TCP;
      switch (state) {
         case TCP_STATE_ALMOST_SYN_SENT:                             //[sendDone] establishement
            call Malloc.freePacketBuffer(msg);
            state =  TCP_STATE_SYN_SENT;
            break;

         case TCP_STATE_ALMOST_SYN_RECEIVED:                         //[sendDone] establishement
            call Malloc.freePacketBuffer(msg);
            state =  TCP_STATE_SYN_RECEIVED;
            break;

         case TCP_STATE_ALMOST_ESTABLISHED:                          //[sendDone] establishement
            call Malloc.freePacketBuffer(msg);
            state =  TCP_STATE_ESTABLISHED;
            signal TCPControl.connectDone[myPort](SUCCESS);
            break;

         case TCP_STATE_ALMOST_DATA_SENT:                            //[sendDone] data
            state =  TCP_STATE_DATA_SENT;
            break;

         case TCP_STATE_ALMOST_DATA_RECEIVED:                        //[sendDone] data
            call Malloc.freePacketBuffer(msg);
            state = TCP_STATE_ESTABLISHED;
            call OpenReceiveToUpper.receive[myPort](dataReceived);
            break;

         case TCP_STATE_ALMOST_FIN_WAIT_1:                           //[sendDone] teardown
            call Malloc.freePacketBuffer(msg);
            state =  TCP_STATE_FIN_WAIT_1;
            break;

         case TCP_STATE_ALMOST_CLOSING:                              //[sendDone] teardown
            call Malloc.freePacketBuffer(msg);
            state = TCP_STATE_CLOSING;
            break;

         case TCP_STATE_ALMOST_TIME_WAIT:                            //[sendDone] teardown
            call Malloc.freePacketBuffer(msg);
            state = TCP_STATE_TIME_WAIT;
            //TODO implement waiting timer
            reset();
            break;

         case TCP_STATE_ALMOST_CLOSE_WAIT:                           //[sendDone] teardown
            call Malloc.freePacketBuffer(msg);
            state = TCP_STATE_CLOSE_WAIT;
            //I send FIN+ACK
            tempPkt = call Malloc.getFreePacketBuffer();
            if (tempPkt==NULL) {
               call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
               call Malloc.freePacketBuffer(msg);
               return;
            }
            tempPkt->creator       = COMPONENT_TCP;
            tempPkt->owner         = COMPONENT_TCP;
            memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
            prependTCPHeader(tempPkt,
                  TCP_ACK_YES,
                  TCP_PSH_NO,
                  TCP_RST_NO,
                  TCP_SYN_NO,
                  TCP_FIN_YES);
            call OpenSendToLower.send[IANA_TCP](tempPkt);
            state = TCP_STATE_ALMOST_LAST_ACK;
            break;

         case TCP_STATE_ALMOST_LAST_ACK:                             //[sendDone] teardown
            call Malloc.freePacketBuffer(msg);
            state = TCP_STATE_LAST_ACK;
            break;

         default:
            call OpenSerial.printError(COMPONENT_TCP,ERR_WRONG_TCP_STATE,(errorparameter_t)state,(errorparameter_t)3);
            break;
      }
   }

   command void OpenReceiveFromLower.receive[uint8_t iana_number](OpenQueueEntry_t* msg) {
      OpenQueueEntry_t* tempPkt;
      msg->owner = COMPONENT_TCP;
      msg->l4_protocol         = IANA_TCP;
      msg->l4_payload          = msg->payload;
      msg->l4_length           = msg->length;
      msg->l4_sourcePortORicmpv6Type      = call PacketFunctions.ntohs((uint8_t*)&(((tcp_ht*)msg->payload)->source_port));
      msg->l4_destination_port = call PacketFunctions.ntohs((uint8_t*)&(((tcp_ht*)msg->payload)->destination_port));
      if ( 
            state!=TCP_STATE_CLOSED &&
            (
             msg->l4_destination_port != myPort  ||
             msg->l4_sourcePortORicmpv6Type      != hisPort ||
             call PacketFunctions.sameAddress(&(msg->l3_destinationORsource),&hisIPv6Address)==FALSE
            )
         ) {
         call Malloc.freePacketBuffer(msg);
         return;
      }
      if (containsControlBits(msg,TCP_ACK_WHATEVER,TCP_RST_YES,TCP_SYN_WHATEVER,TCP_FIN_WHATEVER)) {
         //I receive RST[+*], I reset
         reset();
         call Malloc.freePacketBuffer(msg);
      }
      switch (state) {

         case TCP_STATE_CLOSED:                                      //[receive] establishement
            if (  containsControlBits(msg,TCP_ACK_NO,TCP_RST_NO,TCP_SYN_YES,TCP_FIN_NO) &&
                  (signal TCPControl.shouldIlisten[msg->l4_destination_port]())==TRUE ) {
               myPort = msg->l4_destination_port;
               //I receive SYN, I send SYN+ACK
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+1;
               hisPort       = msg->l4_sourcePortORicmpv6Type;
               memcpy(&hisIPv6Address,&(msg->l3_destinationORsource),sizeof(open_addr_t));
               tempPkt       = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_YES,
                     TCP_FIN_NO);
               mySeqNum++;
               state = TCP_STATE_ALMOST_SYN_RECEIVED;
               call OpenSendToLower.send[IANA_TCP](tempPkt);
            } else {
               reset();
               call OpenSerial.printError(COMPONENT_TCP,ERR_RESET,(errorparameter_t)state,(errorparameter_t)0);
            }
            call Malloc.freePacketBuffer(msg);
            break;

         case TCP_STATE_SYN_SENT:                                    //[receive] establishement
            if (containsControlBits(msg,TCP_ACK_YES,TCP_RST_NO,TCP_SYN_YES,TCP_FIN_NO)) {
               //I receive SYN+ACK, I send ACK
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+1;
               tempPkt = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_NO,
                     TCP_FIN_NO);
               state = TCP_STATE_ALMOST_ESTABLISHED;
               call OpenSendToLower.send[IANA_TCP](tempPkt);
            } else if (containsControlBits(msg,TCP_ACK_NO,TCP_RST_NO,TCP_SYN_YES,TCP_FIN_NO)) {
               //I receive SYN, I send SYN+ACK
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+1;
               tempPkt       = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_YES,
                     TCP_FIN_NO);
               mySeqNum++;
               state = TCP_STATE_ALMOST_SYN_RECEIVED;
               call OpenSendToLower.send[IANA_TCP](tempPkt);
            } else {
               reset();
               call OpenSerial.printError(COMPONENT_TCP,ERR_RESET,(errorparameter_t)state,(errorparameter_t)1);
            }
            call Malloc.freePacketBuffer(msg);
            break;

         case TCP_STATE_SYN_RECEIVED:                                //[receive] establishement
            if (containsControlBits(msg,TCP_ACK_YES,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_NO)) {
               //I receive ACK, the virtual circuit is established
               state = TCP_STATE_ESTABLISHED;
            } else {
               reset();
               call OpenSerial.printError(COMPONENT_TCP,ERR_RESET,(errorparameter_t)state,(errorparameter_t)2);
            }
            call Malloc.freePacketBuffer(msg);
            break;

         case TCP_STATE_ESTABLISHED:                                 //[receive] data/teardown
            if (containsControlBits(msg,TCP_ACK_WHATEVER,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_YES)) {
               //I receive FIN[+ACK], I send ACK
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+msg->length-sizeof(tcp_ht)+1;
               tempPkt = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_NO,
                     TCP_FIN_NO);
               call OpenSendToLower.send[IANA_TCP](tempPkt);
               state = TCP_STATE_ALMOST_CLOSE_WAIT;
            } else if (containsControlBits(msg,TCP_ACK_WHATEVER,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_NO)) {
               //I receive data, I send ACK
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+msg->length-sizeof(tcp_ht);
               tempPkt = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_NO,
                     TCP_FIN_NO);
               call OpenSendToLower.send[IANA_TCP](tempPkt);
               call PacketFunctions.tossHeader(msg,sizeof(tcp_ht));
               dataReceived = msg;
               state = TCP_STATE_ALMOST_DATA_RECEIVED;
            } else {
               reset();
               call OpenSerial.printError(COMPONENT_TCP,ERR_RESET,(errorparameter_t)state,(errorparameter_t)3);
               call Malloc.freePacketBuffer(msg);
            }
            break;

         case TCP_STATE_DATA_SENT:                                   //[receive] data
            if (containsControlBits(msg,TCP_ACK_YES,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_NO)) {
               //I receive ACK, data message sent
               signal OpenSendFromUpper.sendDone[myPort](dataToSend,SUCCESS);
               dataToSend = NULL;
               state = TCP_STATE_ESTABLISHED;
            } else if (containsControlBits(msg,TCP_ACK_WHATEVER,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_YES)) {
               //I receive FIN[+ACK], I send ACK
               signal OpenSendFromUpper.sendDone[myPort](dataToSend,SUCCESS);
               dataToSend = NULL;
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+msg->length-sizeof(tcp_ht)+1;
               tempPkt = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_NO,
                     TCP_FIN_NO);
               call OpenSendToLower.send[IANA_TCP](tempPkt);
               state = TCP_STATE_ALMOST_CLOSE_WAIT;
            } else {
               reset();
               call OpenSerial.printError(COMPONENT_TCP,ERR_RESET,(errorparameter_t)state,(errorparameter_t)4);
            }
            call Malloc.freePacketBuffer(msg);
            break;

         case TCP_STATE_FIN_WAIT_1:                                  //[receive] teardown
            if (containsControlBits(msg,TCP_ACK_NO,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_YES)) {
               //I receive FIN, I send ACK
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+1;
               tempPkt = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_NO,
                     TCP_FIN_NO);
               call OpenSendToLower.send[IANA_TCP](tempPkt);
               state = TCP_STATE_ALMOST_CLOSING;
            } else if (containsControlBits(msg,TCP_ACK_YES,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_YES)) {
               //I receive FIN+ACK, I send ACK
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+1;
               tempPkt = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_NO,
                     TCP_FIN_NO);
               call OpenSendToLower.send[IANA_TCP](tempPkt);
               state = TCP_STATE_ALMOST_TIME_WAIT;
            } else if  (containsControlBits(msg,TCP_ACK_YES,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_NO)) {
               //I receive ACK, I will receive FIN later
               state = TCP_STATE_FIN_WAIT_2;
            } else {
               reset();
               call OpenSerial.printError(COMPONENT_TCP,ERR_RESET,(errorparameter_t)state,(errorparameter_t)5);
            }
            call Malloc.freePacketBuffer(msg);
            break;

         case TCP_STATE_FIN_WAIT_2:                                  //[receive] teardown
            if (containsControlBits(msg,TCP_ACK_WHATEVER,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_YES)) {
               //I receive FIN[+ACK], I send ACK
               hisNextSeqNum = (call PacketFunctions.ntohl((uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number)))+1;
               tempPkt = call Malloc.getFreePacketBuffer();
               if (tempPkt==NULL) {
                  call OpenSerial.printError(COMPONENT_TCP,ERR_NO_FREE_PACKET_BUFFER,0,0);
                  call Malloc.freePacketBuffer(msg);
                  return;
               }
               tempPkt->creator       = COMPONENT_TCP;
               tempPkt->owner         = COMPONENT_TCP;
               memcpy(&(tempPkt->l3_destinationORsource),&hisIPv6Address,sizeof(open_addr_t));
               prependTCPHeader(tempPkt,
                     TCP_ACK_YES,
                     TCP_PSH_NO,
                     TCP_RST_NO,
                     TCP_SYN_NO,
                     TCP_FIN_NO);
               call OpenSendToLower.send[IANA_TCP](tempPkt);
               state = TCP_STATE_ALMOST_TIME_WAIT;
            }
            call Malloc.freePacketBuffer(msg);
            break;

         case TCP_STATE_CLOSING:                                     //[receive] teardown
            if (containsControlBits(msg,TCP_ACK_YES,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_NO)) {
               //I receive ACK, I do nothing
               state = TCP_STATE_TIME_WAIT;
               //TODO implement waiting timer
               reset();
            }
            call Malloc.freePacketBuffer(msg);
            break;

         case TCP_STATE_LAST_ACK:                                    //[receive] teardown
            if (containsControlBits(msg,TCP_ACK_YES,TCP_RST_NO,TCP_SYN_NO,TCP_FIN_NO)) {
               //I receive ACK, I reset
               reset();
            }
            call Malloc.freePacketBuffer(msg);
            break;

         default:
            call OpenSerial.printError(COMPONENT_TCP,ERR_WRONG_TCP_STATE,(errorparameter_t)state,(errorparameter_t)4);
            break;
      }
   }   

   /*-------------------------------- helper functions ----------------------------------*/

   void prependTCPHeader(OpenQueueEntry_t* msg,
         bool ack,
         bool push,
         bool rst,
         bool syn,
         bool fin) {
      msg->l4_protocol = IANA_TCP;
      call PacketFunctions.reserveHeaderSize(msg,sizeof(tcp_ht));
      call PacketFunctions.htons(myPort        ,(uint8_t*)&(((tcp_ht*)msg->payload)->source_port));
      call PacketFunctions.htons(hisPort       ,(uint8_t*)&(((tcp_ht*)msg->payload)->destination_port));
      call PacketFunctions.htonl(mySeqNum      ,(uint8_t*)&(((tcp_ht*)msg->payload)->sequence_number));
      call PacketFunctions.htonl(hisNextSeqNum ,(uint8_t*)&(((tcp_ht*)msg->payload)->ack_number));
      ((tcp_ht*)msg->payload)->data_offset      = TCP_DEFAULT_DATA_OFFSET;
      ((tcp_ht*)msg->payload)->control_bits     = 0;
      if (ack==TCP_ACK_YES) {
         ((tcp_ht*)msg->payload)->control_bits |= 1 << TCP_ACK;
      } else {
         call PacketFunctions.htonl(0,(uint8_t*)&(((tcp_ht*)msg->payload)->ack_number));
      }
      if (push==TCP_PSH_YES) {
         ((tcp_ht*)msg->payload)->control_bits |= 1 << TCP_PSH;
      }
      if (rst==TCP_RST_YES) {
         ((tcp_ht*)msg->payload)->control_bits |= 1 << TCP_RST;
      }
      if (syn==TCP_SYN_YES) {
         ((tcp_ht*)msg->payload)->control_bits |= 1 << TCP_SYN;
      }
      if (fin==TCP_FIN_YES) {
         ((tcp_ht*)msg->payload)->control_bits |= 1 << TCP_FIN;
      }
      call PacketFunctions.htons(TCP_DEFAULT_WINDOW_SIZE    ,(uint8_t*)&(((tcp_ht*)msg->payload)->window_size));
      call PacketFunctions.htons(TCP_DEFAULT_URGENT_POINTER ,(uint8_t*)&(((tcp_ht*)msg->payload)->urgent_pointer));
      //calculate checksum last to take all header fields into account
      call PacketFunctions.calculateChecksum(msg,(uint8_t*)&(((tcp_ht*)msg->payload)->checksum));
   }

   bool containsControlBits(OpenQueueEntry_t* msg, uint8_t ack, uint8_t rst, uint8_t syn, uint8_t fin) {
      bool return_value = TRUE;
      if (ack!=TCP_ACK_WHATEVER){
         return_value = return_value && ((bool)( (((tcp_ht*)msg->payload)->control_bits >> TCP_ACK) & 0x01) == ack);
      }
      if (rst!=TCP_RST_WHATEVER){
         return_value = return_value && ((bool)( (((tcp_ht*)msg->payload)->control_bits >> TCP_RST) & 0x01) == rst);
      }
      if (syn!=TCP_SYN_WHATEVER){
         return_value = return_value && ((bool)( (((tcp_ht*)msg->payload)->control_bits >> TCP_SYN) & 0x01) == syn);
      }
      if (fin!=TCP_FIN_WHATEVER){
         return_value = return_value && ((bool)( (((tcp_ht*)msg->payload)->control_bits >> TCP_FIN) & 0x01) == fin);
      }
      return return_value;
   }

   void reset() {
      state               = TCP_STATE_CLOSED;
      mySeqNum            = TCP_INITIAL_SEQNUM; 
      hisNextSeqNum       = 0;
      hisPort             = 0;
      hisIPv6Address.type = ADDR_NONE;
      dataToSend          = NULL;
   }

   /*-------------------------------- defaults ------------------------------------------*/

   default event void OpenSendFromUpper.sendDone[uint16_t localPortNumber](OpenQueueEntry_t* msg, error_t error) {
      call OpenSerial.printError(COMPONENT_TCP,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,0);
   }

   default command void OpenReceiveToUpper.receive[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_TCP,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,1);
      call Malloc.freePacketBuffer(msg);
      dataReceived = NULL;
   }

   default event void TCPControl.connectDone[uint16_t localPortNumber](error_t error) {
      call OpenSerial.printError(COMPONENT_TCP,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,2);
   }

   default event bool TCPControl.shouldIlisten[uint16_t localPortNumber]() {
      call OpenSerial.printError(COMPONENT_TCP,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,3);
      return FALSE;
   }

   default command error_t OpenSendToLower.send[uint8_t iana_number](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_UDP,ERR_WRONG_TRAN_PROTOCOL,iana_number,0);
      return FAIL;
   }
}

