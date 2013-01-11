#define NEW_PRINTF_SEMANTICS
#include "printf.h"
#include "EColl.h"
#include "TreeRouting.h"
#include "CC2420.h"

configuration EasyCollectionAppC {}
implementation {
  components EasyCollectionC;
  components PrintfC, SerialStartC; 
  components  MainC, LedsC, ActiveMessageC;
  components CollectionC as Collector;
  components new CollectionSenderC(0xee);
  components new TimerMilliC() as TimerWait, new TimerMilliC() as TimerPhase;
  components new TimerMilliC() as TimerSensor;
  components CC2420PacketC as SetRadio;  
  EasyCollectionC.Capisrunning -> ActiveMessageC;
  EasyCollectionC.TimerSensor -> TimerSensor;    
  EasyCollectionC.Boot -> MainC;
  EasyCollectionC.RadioControl -> ActiveMessageC;
  EasyCollectionC.RoutingControl -> Collector;
  EasyCollectionC.Leds -> LedsC;
  EasyCollectionC.TimerWait -> TimerWait;
  EasyCollectionC.TimerPhase -> TimerPhase;
  EasyCollectionC.Send -> CollectionSenderC;
  EasyCollectionC.RootControl -> Collector;
  EasyCollectionC.Receive -> Collector.Receive[0xee];
  EasyCollectionC.GetCTPData -> Collector;
  EasyCollectionC.CollectionPacket -> Collector;
  EasyCollectionC.Cngstn -> Collector;
  EasyCollectionC.RadioInterface -> SetRadio;
 
  components HplMsp430GeneralIOC;
  components new Msp430GpioC() as ADC0;
  ADC0 -> HplMsp430GeneralIOC.Port60;
  EasyCollectionC.ADC0 -> ADC0;
  
  components new Msp430GpioC() as ADC1;
  ADC1 -> HplMsp430GeneralIOC.Port61;
  EasyCollectionC.ADC1 -> ADC1;
  
  components new Msp430Adc12ClientAutoRVGC() as AutoAdc;
  EasyCollectionC.Resource -> AutoAdc;
  AutoAdc.AdcConfigure -> EasyCollectionC;
  EasyCollectionC.MultiChannel -> AutoAdc.Msp430Adc12MultiChannel;
  
  components UserButtonC;
  EasyCollectionC.UserButton -> UserButtonC;
  
  components  Ieee802154BeaconEnabledC as serialMAC;
  EasyCollectionC.Packet -> serialMAC;	
  
  components SerialActiveMessageC as Serial;
  EasyCollectionC.SerialControl -> Serial;
  EasyCollectionC.UartSend -> Serial.AMSend[AM_MYMSG];
  EasyCollectionC.UartAMPacket -> Serial; 
}