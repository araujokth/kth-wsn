// $Id: BlinkToRadio.h,v 1.4 2006/12/12 18:22:52 vlahan Exp $

#ifndef APP_SENSORS_H
#define APP_SENSORS_H


#define LS7366R_DEF_SIZE 2
#define LS7366R_DEF_QUAD_MODE LS7366R_MDR0_X1_QUAD_MODE
#define LS7366R_DEF_RUNNING_MODE LS7366R_MDR0_FREE_RUNNING_MODE
#define LS7366R_DEF_INDEX LS7366R_MDR0_NO_INDEX
#define LS7366R_DEF_CLK_DIV LS7366R_MDR0_CLK_0_DIV
#define LS7366R_DEF_SYNC_INDEX LS7366R_MDR0_ASYNC_INDEX
#define LS7366R_DEF_MOD_N_LIMIT (1 << 12 )  //2^11 = 4096


enum {
  AM_ENCMSG = 7,
  AM_RATEMSG = 8,
  AM_MYMSG = 10, //Radio channel number
  
  //DEFAULT_RATE = 22,
  DEFAULT_RATE = 25,
  OFFSET_SENDER = 5,

  UART_QUEUE_LEN = 12,

//Not broadcast, just 2 try
  BROADCAST_DESTINATION = 0x01,

  RADIO_CHANNEL = 0x14,
  PAN_ID = 0x1234,

  BEACON_ORDER = 6,
  SUPERFRAME_ORDER = 5,

  COORDINATOR_ADDRESS = 0x00,
  BASE_STATION_ADDRESS = 0x02,

  TX_POWER = 0, // in dBm
  TX_POWER_COORDINATOR = 0, // in dBm
  
  TIMER_PREC = 32768U,
  
  BUFFER_SIZE=1,
  
};

//The message sent over the air.
//The default maximum payload is 28byte
typedef struct EncRS232Msg {
  uint16_t header;
  int16_t data; //data
  uint16_t footer;
} EncRS232Msg;

//The message sent over the air.
//The default maximum payload is 28byte 
typedef nx_struct EncMsg {
//  nx_int16_t counter; //data
  nx_int16_t data; //data
} EncMsg;

typedef nx_struct EncMsg2Sensors {
//  nx_int16_t counter; //data
  nx_int16_t data_theta; //data angle
  nx_int16_t data_xc; //data cart
} EncMsg2Sensors;

//The default maximum payload is 28byte
typedef nx_struct EncMsgBase {
  nx_int16_t counter; //data
  nx_int16_t data; //data
} EncMsgBase;

typedef nx_struct RateMsg {
  nx_uint16_t nodeid;  //NODE ID of the node, 2byte
  nx_uint16_t rate; //sampling rate
} RateMsg;

//structure of current meter message
typedef nx_struct MyMsg{// max 28 bytes
   nx_uint8_t other;
   nx_uint8_t srcId;
   nx_uint8_t trgtId;
   nx_uint16_t data[BUFFER_SIZE*2];
   nx_float integrator;
   nx_float next_time;
}MyMsg;

//structure of the message used by the BS to communicate with the PC in the WaterTanks system
typedef nx_struct EncMsgWT {
  nx_int16_t y11; //Tank system 1's upper tank level
  nx_int16_t y12; //Tank system 1's lower tank level
  nx_int16_t y21; //Tank system 2's upper tank level
  nx_int16_t y22; //Tank system 2's lower tank level
  nx_int16_t s1; //Sensor node 1
  nx_int16_t s2; //Sensor node 2
  nx_int16_t s3; //Sensor node 3
  nx_int16_t u1; //Voltage applied to pump on tank system 1
  nx_int16_t u2; //Voltage applied to pump on tank system 2
  nx_uint32_t time; //Time when CFP started
  nx_float i1; //Integral of y12 tracking error
  nx_float i2; //Integral of y22 tracking error
  //nx_float next_time1;
  //nx_float next_time2;
  nx_int16_t y11_initial; //Tank system 1's upper tank level
  nx_int16_t y12_initial; //Tank system 1's lower tank level
  nx_int16_t y21_initial; //Tank system 2's upper tank level
  nx_int16_t y22_initial; //Tank system 2's lower tank level
  nx_int16_t u1_initial; //Voltage applied to pump on tank system 1
  nx_int16_t u2_initial; //Voltage applied to pump on tank system 2
  nx_float i1_initial; //Integral of y12 tracking error
  nx_float i2_initial; //Integral of y22 tracking error

} EncMsgWT;

//structure of the message used by the PC to communicate with the BS in the WaterTanks system
typedef nx_struct RecMsgWT {
  nx_uint8_t BI; //Beacon interval
  nx_uint8_t T1; //
  nx_uint8_t T2; //
  nx_uint8_t T3; //
  nx_uint8_t T4; //Time slots for communication between motes and BS
  nx_uint8_t T5; //
  nx_uint8_t T6; //
  nx_uint8_t T7; //
} RecMsgWT;
#endif
