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
  components new TimerMilliC();
  components CC2420PacketC as SetRadio;  
  
  //components LocalTimeMilliC;
  //components new  CounterToLocalTimeC(T32khz) as CurrSysTime;
  //components LinkEstimatorC as LSense;
      
  SenseCollectionC.Boot -> MainC;
  SenseCollectionC.RadioControl -> ActiveMessageC;
  SenseCollectionC.RoutingControl -> Collector;
  SenseCollectionC.Leds -> LedsC;
  SenseCollectionC.Timer -> TimerMilliC;
  SenseCollectionC.Send -> CollectionSenderC;
  SenseCollectionC.RootControl -> Collector;
  SenseCollectionC.Receive -> Collector.Receive[0xee];
  SenseCollectionC.GetCTPData -> Collector;
  SenseCollectionC.CollectionPacket -> Collector;
  SenseCollectionC.Cngstn -> Collector;
  SenseCollectionC.RadioInterface -> SetRadio;
  SenseCollectionC.Capisrunning -> ActiveMessageC;
  
  //SenseCollectionC.LinkEstimator -> LSense ;
  //SenseCollectionC.LocalTime -> LocalTimeMilliC;
  //SenseCollectionC.LocalTime -> CurrSysTime;
   
 }
