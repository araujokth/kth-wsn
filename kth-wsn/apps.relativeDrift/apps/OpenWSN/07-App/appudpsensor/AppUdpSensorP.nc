#include "OpenWSN.h"
#include "AppUdpSensor.h"

module AppUdpSensorP {
   //admin
   provides interface Init as SoftwareInit;
   //data flow
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   //sensors
   uses interface Read<uint16_t> as ReadLightVisible;
   uses interface Read<uint16_t> as ReadLightIR;
   uses interface Read<uint16_t> as ReadTemperature;
   uses interface Read<uint16_t> as ReadHumidity;
   //misc
   uses interface Random;
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   uses interface PacketFunctions;
   uses interface Timer<TMilli> as TimerMeasure;
   provides interface Trigger as TriggerUDPSensor;
}
implementation {
   /*-------------------------------- variables -----------------------------------------*/

   OpenQueueEntry_t* pkt;
   bool              sending;
   open_addr_t       destination_address;
   uint16_t          destination_port;
   uint8_t           sensor_counter;
   sensor_data_ht    current_tlv;
   uint8_t           lollipop;

   /*-------------------------------- prototypes ----------------------------------------*/

   task void taskRecordAddress();
   task void taskStartMeasurement();
   task void taskSend();

   /*-------------------------------- receive sequence ----------------------------------*/

   command void TriggerUDPSensor.trigger() {
      post taskRecordAddress();
   }

   task void taskRecordAddress() {
      uint8_t number_bytes_from_input_buffer;
      uint8_t input_buffer[18];
      //get command from OpenSerial (16B IPv6 destination address, 2B destination port)
      number_bytes_from_input_buffer = call OpenSerial.getInputBuffer(&(input_buffer[0]),sizeof(input_buffer));
      if (number_bytes_from_input_buffer!=sizeof(input_buffer)) {
         call OpenSerial.printError(COMPONENT_APPUDPSENSOR,ERR_INPUTBUFFER_LENGTH,
               (errorparameter_t)number_bytes_from_input_buffer,
               (errorparameter_t)0);
         return;
      };
      memcpy(&(destination_address.addr_128b[0]),&(input_buffer[0]),16);
      destination_port = call PacketFunctions.ntohs(&(input_buffer[16]));
      post taskStartMeasurement();
   }

   event void TimerMeasure.fired() {
      if (call IDManager.getIsBridge()==FALSE) {
         call TimerMeasure.startPeriodic(MEASUREMENT_PERIOD_MS/2+((call Random.rand16())%MEASUREMENT_PERIOD_MS));
         post taskStartMeasurement();
      }
   }

   task void taskStartMeasurement() {
      atomic P3OUT |= 0x10;
      if (call IDManager.getIsBridge()==FALSE) {
         sensor_counter = (sensor_counter+1)%5;
         switch(sensor_counter) {
            case 0:
               call ReadTemperature.read();
               break;
            case 1:
               //call ReadLightIR.read();
               break;
            case 2:
               //call ReadLightVisible.read();
               break;
            case 3:
               call ReadHumidity.read();
               break;
            case 4:
               lollipop++;
               if (lollipop==0) {
                  lollipop=128;
               }
               current_tlv.type   = 'L';
               current_tlv.length = 2;
               current_tlv.value  = lollipop;
               post taskSend();
               break;
         }
      }
   }

   event void ReadTemperature.readDone(error_t result, uint16_t val) {//takes 250ms
      current_tlv.type   = 'T';
      current_tlv.length = 2;
      current_tlv.value  = val;
      post taskSend();
   }
   event void ReadLightIR.readDone(error_t result, uint16_t val) {//takes 19.6ms
      current_tlv.type   = 'I';
      current_tlv.length = 2;
      current_tlv.value  = val;
      post taskSend();
   }
   event void ReadLightVisible.readDone(error_t result, uint16_t val) {//takes 1ms
      current_tlv.type   = 'V';
      current_tlv.length = 2;
      current_tlv.value  = val;
      post taskSend();
   }
   event void ReadHumidity.readDone(error_t result, uint16_t val) {//take 82ms
      current_tlv.type   = 'H';
      current_tlv.length = 2;
      current_tlv.value  = val;
      post taskSend();
   }

   task void taskSend() {
      pkt = call Malloc.getFreePacketBuffer();
      if (pkt==NULL) {
         call OpenSerial.printError(COMPONENT_APPUDPSENSOR,ERR_NO_FREE_PACKET_BUFFER,(errorparameter_t)0,(errorparameter_t)0);
         return;
      }
      //metadata
      pkt->creator                     = COMPONENT_APPUDPSENSOR;
      pkt->owner                       = COMPONENT_APPUDPSENSOR;
      pkt->l4_protocol                 = IANA_UDP;
      pkt->l4_sourcePortORicmpv6Type   = WKP_UDP_SENSOR;
      pkt->l4_destination_port         = destination_port;
      pkt->l3_destinationORsource.type = ADDR_128B;
      memcpy(&(pkt->l3_destinationORsource.addr_128b[0]),&(destination_address.addr_128b[0]),16);
      //payload
      call PacketFunctions.reserveHeaderSize(pkt,sizeof(sensor_data_ht));
      memcpy(pkt->payload,&current_tlv,sizeof(sensor_data_ht));
      //send
      if ((call OpenSendToLower.send[WKP_UDP_SENSOR](pkt))==FAIL) {
         call Malloc.freePacketBuffer(pkt);
      }
      return;
   }

   event void OpenSendToLower.sendDone[uint16_t localPortNumber](OpenQueueEntry_t* msg, error_t error) {
      msg->owner = COMPONENT_APPUDPSENSOR;
      if (msg->creator!=COMPONENT_APPUDPSENSOR) {
         call OpenSerial.printError(COMPONENT_APPUDPSENSOR,ERR_SENDDONE_FOR_MSG_I_DID_NOT_SEND,0,0);
      }
      call Malloc.freePacketBuffer(msg);
      atomic P3OUT &= ~0x10;
      post taskSend();
   }

   /*-------------------------------- interfaces ----------------------------------------*/

   command error_t SoftwareInit.init() {
      destination_address.type          = ADDR_128B;
      destination_address.addr_128b[0]  = 0x20;
      destination_address.addr_128b[1]  = 0x01;
      destination_address.addr_128b[2]  = 0x04;
      destination_address.addr_128b[3]  = 0x70;
      destination_address.addr_128b[4]  = 0x1f;
      destination_address.addr_128b[5]  = 0x04;
      destination_address.addr_128b[6]  = 0x0d;
      destination_address.addr_128b[7]  = 0xff;
      destination_address.addr_128b[8]  = 0x00;
      destination_address.addr_128b[9]  = 0x00;
      destination_address.addr_128b[10] = 0x00;
      destination_address.addr_128b[11] = 0x00;
      destination_address.addr_128b[12] = 0x00;
      destination_address.addr_128b[13] = 0x00;
      destination_address.addr_128b[14] = 0x00;
      destination_address.addr_128b[15] = 0x02;
      destination_port                  = 8080;
      sensor_counter = 0;
      lollipop = 0;
      call TimerMeasure.startPeriodic(MEASUREMENT_PERIOD_MS/2+((call Random.rand16())%MEASUREMENT_PERIOD_MS));
      return SUCCESS;
   }

   /*-------------------------------- defaults ------------------------------------------*/

   default command error_t OpenSendToLower.send[uint16_t localPortNumber](OpenQueueEntry_t* msg) {
      call OpenSerial.printError(COMPONENT_APPUDPSENSOR,ERR_UNSUPPORTED_PORT_NUMBER,localPortNumber,0);
      return FAIL;
   }
}
