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
  /* components new TimerMilliC() as TimerPhase; */
  components new TimerMilliC() as TimerSPI;
  /* components new TimerMilliC() as TimerSensor; */
  components CC2420PacketC as SetRadio;  
  SenseCollectionC.Capisrunning -> ActiveMessageC;
  /* SenseCollectionC.TimerSensor -> TimerSensor;     */
  SenseCollectionC.Boot -> MainC;
  SenseCollectionC.RadioControl -> ActiveMessageC;
  SenseCollectionC.RoutingControl -> Collector;
  SenseCollectionC.Leds -> LedsC;
  SenseCollectionC.TimerSPI -> TimerSPI;
  /* SenseCollectionC.TimerWait -> TimerWait; */
  /* SenseCollectionC.TimerPhase -> TimerPhase; */
  SenseCollectionC.Send -> CollectionSenderC;
  SenseCollectionC.RootControl -> Collector;
  SenseCollectionC.Receive -> Collector.Receive[0xee];
  SenseCollectionC.GetCTPData -> Collector;
  SenseCollectionC.CollectionPacket -> Collector;
  SenseCollectionC.Cngstn -> Collector;
  SenseCollectionC.RadioInterface -> SetRadio;
 
  /* components HplMsp430GeneralIOC; */
  /* components new Msp430GpioC() as ADC0; */
  /* ADC0 -> HplMsp430GeneralIOC.Port60; */
  /* SenseCollectionC.ADC0 -> ADC0; */
  
  /* components new Msp430GpioC() as ADC1; */
  /* ADC1 -> HplMsp430GeneralIOC.Port61; */
  /* SenseCollectionC.ADC1 -> ADC1; */
  
  /* components new Msp430Adc12ClientAutoRVGC() as AutoAdc; */
  /* SenseCollectionC.Resource -> AutoAdc; */
  /* AutoAdc.AdcConfigure -> SenseCollectionC; */
  /* SenseCollectionC.MultiChannel -> AutoAdc.Msp430Adc12MultiChannel; */
  
  components UserButtonC;
  SenseCollectionC.UserButton -> UserButtonC;
  
  components  Ieee802154BeaconEnabledC as serialMAC;
  SenseCollectionC.Packet -> serialMAC;	
  
  components SerialActiveMessageC as Serial;
  SenseCollectionC.SerialControl -> Serial;
  SenseCollectionC.UartSend -> Serial.AMSend[AM_MYMSG];
  SenseCollectionC.UartAMPacket -> Serial;

  /* components LocalTime62500hzC; */
  /* SenseCollectionC.LocalTime -> LocalTime62500hzC; */
}
