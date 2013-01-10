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
  components new TimerMilliC();
  components CC2420PacketC as SetRadio;  
  

      
  EasyCollectionC.Boot -> MainC;
  EasyCollectionC.RadioControl -> ActiveMessageC;
  EasyCollectionC.RoutingControl -> Collector;
  EasyCollectionC.Leds -> LedsC;
  EasyCollectionC.Timer -> TimerMilliC;
  EasyCollectionC.Send -> CollectionSenderC;
  EasyCollectionC.RootControl -> Collector;
  EasyCollectionC.Receive -> Collector.Receive[0xee];

  
  
    components new Msp430Adc12ClientAutoRVGC() as AutoAdc;
	EasyCollectionC.Resource -> AutoAdc;
	AutoAdc.AdcConfigure -> EasyCollectionC;
	EasyCollectionC.MultiChannel -> AutoAdc.Msp430Adc12MultiChannel;
	
	components UserButtonC;
	EasyCollectionC.UserButton -> UserButtonC;
   
 }
