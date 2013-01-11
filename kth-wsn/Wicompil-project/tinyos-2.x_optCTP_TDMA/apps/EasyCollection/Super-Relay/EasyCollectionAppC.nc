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
  
  //components LocalTimeMilliC;
  //components new  CounterToLocalTimeC(T32khz) as CurrSysTime;
  //components LinkEstimatorC as LSense;
      
  EasyCollectionC.Boot -> MainC;
  EasyCollectionC.RadioControl -> ActiveMessageC;
  EasyCollectionC.RoutingControl -> Collector;
  EasyCollectionC.Leds -> LedsC;
  EasyCollectionC.Timer -> TimerMilliC;
  EasyCollectionC.Send -> CollectionSenderC;
  EasyCollectionC.RootControl -> Collector;
  EasyCollectionC.Receive -> Collector.Receive[0xee];
  EasyCollectionC.Capisrunning -> ActiveMessageC;
  
  //EasyCollectionC.LinkEstimator -> LSense ;
  //EasyCollectionC.LocalTime -> LocalTimeMilliC;
  //EasyCollectionC.LocalTime -> CurrSysTime;
   
 }
