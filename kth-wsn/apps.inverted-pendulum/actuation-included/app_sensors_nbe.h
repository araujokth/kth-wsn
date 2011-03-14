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
  
  DEFAULT_RATE = 32,
  OFFSET_SENDER = 5,

  UART_QUEUE_LEN = 12,

//Not broadcast, just 2 try
  BROADCAST_DESTINATION = 0x01,

  RADIO_CHANNEL = 0x16,
  PAN_ID = 0x1234,

  BEACON_ORDER = 15,
  SUPERFRAME_ORDER = 15,

  CONTROLLER_ADDRESS = 0x00,
  CONTROLLER_ACTUATOR_ADDRESS = CONTROLLER_ADDRESS,
  ACTUATOR_ADDRESS = 0x03,
  XCART_ADDRESS = 0x01,
  TPENDULUM_ADDRESS = 0x02,

  TX_POWER = 0, // in dBm
  TX_POWER_COORDINATOR = 0, // in dBm
  
  TIMER_PREC = 32768U,
  
  BUFFER_SIZE=1,
  
};

//The message sent over the air.
//The default maximum payload is 28byte 
typedef nx_struct EncMsg {
  nx_int16_t data; //data
} EncMsg;

typedef nx_struct EncMsg2Sensors {
//  nx_int16_t counter; //data
  nx_int16_t data_theta; //data angle
  nx_int16_t data_xc; //data cart
} EncMsg2Sensors;

typedef nx_struct EncMsg2SensorsAct {
//  nx_int16_t counter; //data
  nx_int16_t data_theta; //data angle
  nx_int16_t data_xc; //data cart
  nx_int16_t u; //actuation
} EncMsg2SensorsAct;

//The default maximum payload is 28byte
typedef nx_struct EncMsgBase {
  nx_int16_t counter; //data
  nx_int16_t data; //data
} EncMsgBase;

typedef nx_struct RateMsg {
  nx_uint16_t nodeid;  //NODE ID of the node, 2byte
  nx_uint16_t rate; //sampling rate
} RateMsg;

#endif
