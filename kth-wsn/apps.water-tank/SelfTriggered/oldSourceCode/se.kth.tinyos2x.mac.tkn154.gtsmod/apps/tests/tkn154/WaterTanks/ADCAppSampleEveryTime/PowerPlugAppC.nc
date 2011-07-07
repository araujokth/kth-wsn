/*******************
 @PowerPlugAppC.nc
********************/
#include "Msp430Adc12.h"
#include <Timer.h>
#include "PowerPlug.h"

#include "app_profile.h"


configuration PowerPlugAppC {
}
implementation
{
  //components implemented
  components MainC,
    LedsC,
    HplMsp430GeneralIOC;
 
  components PowerPlugC,
             new Msp430Adc12ClientAutoRVGC() as AutoAdc,  Ieee802154BeaconEnabledC as MAC;
  //components HplUserButtonC0,
  //           HplUserButtonC1;
  
  //connections
  //PowerPlugC.Relay0->HplUserButtonC0.GeneralIO;
  //PowerPlugC.Relay1->HplUserButtonC1.GeneralIO;
  PowerPlugC -> MainC.Boot;
  PowerPlugC.Leds -> LedsC;
  PowerPlugC.Resource -> AutoAdc;
  AutoAdc.AdcConfigure -> PowerPlugC;

  PowerPlugC.MultiChannel -> AutoAdc.Msp430Adc12MultiChannel;
  
  
  PowerPlugC.MLME_SCAN -> MAC;
  PowerPlugC.MLME_SYNC -> MAC;
  PowerPlugC.MLME_BEACON_NOTIFY -> MAC;
  PowerPlugC.MLME_SYNC_LOSS -> MAC;
  PowerPlugC.MCPS_DATA -> MAC;
  PowerPlugC.MLME_GTS -> MAC;
  PowerPlugC.Frame -> MAC;
  PowerPlugC.BeaconFrame -> MAC;
  PowerPlugC.GtsUtility -> MAC;
  PowerPlugC.Packet -> MAC;

  PowerPlugC.MLME_RESET -> MAC;
  PowerPlugC.MLME_SET -> MAC;
  PowerPlugC.MLME_GET -> MAC;
  
  PowerPlugC.GtsCoordinatorDb -> MAC;
  PowerPlugC.IsGtsOngoing -> MAC;
  

}
