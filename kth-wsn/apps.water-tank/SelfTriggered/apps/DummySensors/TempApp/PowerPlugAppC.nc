/*******************
 @PowerPlugAppC.nc
********************/
#include "Msp430Adc12.h"
#include <Timer.h>
#include "printf.h"
#include "PowerPlug.h"

#include "app_profile.h"


configuration PowerPlugAppC {
}
implementation
{
  //components implemented
  components MainC,
    new SensirionSht11C(),     
    LedsC;
 
  components PowerPlugC, Ieee802154BeaconEnabledC as MAC;
            
  PowerPlugC -> MainC.Boot;
  PowerPlugC.Leds -> LedsC;
  
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
  
  PowerPlugC.IsGtsOngoing -> MAC;
  
  PowerPlugC.ReadTemp -> SensirionSht11C.Temperature;
  

}
