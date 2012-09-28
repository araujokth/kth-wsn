#ifndef __APP_PROFILE_H
#define __APP_PROFILE_H

enum {
  RADIO_CHANNEL =26,
  PAN_ID = 0x1234,
  COORDINATOR_ADDRESS = 0,
  DEVICE_ADDRESS=2,
  BEACON_ORDER = 6,
  SUPERFRAME_ORDER = 5,
  TX_POWER_BEACON = 0,
  TX_POWER = -20, // in dBm
  BUFFER_SIZE=3,
  AM_MYMSG = 10,
  AM_ACTUATORMSG= 11,
  
  ACTUATOR_ADDRESS1=1,
  ACTUATOR_ADDRESS2=2,
  ACTUATOR_ADDRESS3=3,
  ACTUATOR_ADDRESS4=4,
  ACTUATOR_ADDRESS5=5,
  ACTUATOR_ADDRESS6=6,
  ACTUATOR_ADDRESS7=7,
  ACTUATOR_ADDRESS8=8,
  
  SENSOR_ADDRESS1=9,
  SENSOR_ADDRESS2=10,
  SENSOR_ADDRESS3=11,
  SENSOR_ADDRESS4=12,
  SENSOR_ADDRESS5=13,
  SENSOR_ADDRESS6=14,
  SENSOR_ADDRESS7=15,
  SENSOR_ADDRESS8=16,
  
  ACTUATOR=2,
  SENSOR=ACTUATOR,

};

typedef nx_struct SensorValues {
   nx_uint8_t other;
   nx_uint8_t srcId;
   nx_uint16_t trgtId;
   nx_uint16_t data[BUFFER_SIZE]; 			// Values for the tk1 and tk2
} SensorValues;

typedef nx_struct ControllertoSF {

	nx_uint8_t sensorId[6];   // Id for Sensors
	nx_uint16_t dataWT[2*6];  // Values for the tk1 and tk2 //for one tank 4 bytes
	nx_uint8_t  Sensor[6];    // Shedular 
	nx_uint8_t  BSN;          // BSN;
 	nx_uint8_t  lock;         //if lock ==1 meaning   GTS Allocate/Dellocate request received

} ControllertoSF;

typedef nx_struct SFtoController{
	nx_uint16_t volt[6];                  //volt[0] =volta  , volt[1] =voltb  .....
    nx_uint8_t  Sensor  ; 
	nx_uint8_t   Actuator  ;  
	
} SFtoController;


typedef nx_struct ToActuator {
	nx_uint16_t trgId;
	nx_uint16_t volt; 
	nx_uint16_t counter; 			
} ToActuator;

typedef nx_struct GTSrequest{
 
 nx_uint8_t  gtspayload;
 nx_uint8_t  Id;	             

} GTSrequest;

#endif
