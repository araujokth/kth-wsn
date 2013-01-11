#define NEW_PRINTF_SEMANTICS
#include "printf.h"
#include "EColl.h"
#include "TreeRouting.h"
#include "CC2420.h"

configuration SenseCollectionAppC {}
implementation {
  components SenseCollectionC;
  components PrintfC, SerialStartC; 
  components  MainC, LedsC, ActiveMessageC;
  components CollectionC as Collector;
  components new CollectionSenderC(0xee);
//  components new TimerMilliC();
  components CC2420PacketC as SetRadio;  
  
  SenseCollectionC.Boot -> MainC;
  SenseCollectionC.RadioControl -> ActiveMessageC;
  SenseCollectionC.RoutingControl -> Collector;
  SenseCollectionC.Leds -> LedsC;
//  SenseCollectionC.Timer -> TimerMilliC;
  SenseCollectionC.Send -> CollectionSenderC;
  SenseCollectionC.RootControl -> Collector;
  SenseCollectionC.Receive -> Collector.Receive[0xee];
  SenseCollectionC.Capisrunning -> ActiveMessageC;
  
  /* components new Msp430Adc12ClientAutoRVGC() as AutoAdc; */
  /* SenseCollectionC.Resource -> AutoAdc; */
  /* AutoAdc.AdcConfigure -> SenseCollectionC; */
  /* SenseCollectionC.MultiChannel -> AutoAdc.Msp430Adc12MultiChannel; */
  
  components new SensirionSht11C() as Sensor;
  SenseCollectionC.ReadSensor -> Sensor.Humidity;

  components UserButtonC;
  SenseCollectionC.UserButton -> UserButtonC; 
 }
