#include "OpenWSN.h"
#include "UserButton.h"

configuration AppUdpSensorC {
   //data flow
   uses interface OpenSend as OpenSendToLower[uint16_t localPortNumber];
   //misc
   uses interface Malloc;
   uses interface IDManager;
   uses interface OpenSerial;
   provides interface Trigger as TriggerUDPSensor;
}
implementation {
   components AppUdpSensorP;
   IDManager         = AppUdpSensorP.IDManager;
   OpenSendToLower   = AppUdpSensorP.OpenSendToLower;
   Malloc            = AppUdpSensorP.Malloc;
   OpenSerial        = AppUdpSensorP.OpenSerial;
   TriggerUDPSensor  = AppUdpSensorP.TriggerUDPSensor;

   components MainC;
   MainC.SoftwareInit->AppUdpSensorP;

   components new TimerMilliC() as TimerMeasureC;
   AppUdpSensorP.TimerMeasure->TimerMeasureC;

   components PacketFunctionsC;
   AppUdpSensorP.PacketFunctions->PacketFunctionsC;

   components RandomC;
   AppUdpSensorP.Random->RandomC;

   //components new HamamatsuS1087ParC() as SensorLightVisible;
   //AppUdpSensorP.ReadLightVisible->SensorLightVisible;

   //components new HamamatsuS10871TsrC() as SensorLightIR;
   //AppUdpSensorP.ReadLightIR->SensorLightIR;

   components new SensirionSht11C() as SensorTemperatureHumidity;
   AppUdpSensorP.ReadTemperature->SensorTemperatureHumidity.Temperature;
   AppUdpSensorP.ReadHumidity->SensorTemperatureHumidity.Humidity;
}

