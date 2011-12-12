configuration wiring {
}

implementation {
   components MainC;
   components fabMACC;
   components RF231driverC;
 //  components RF231SpiC;


   fabMACC.Boot -> MainC;
   fabMACC.RadioControl -> RF231driverC;
   fabMACC.RadioSend -> RF231driverC;
   fabMACC.RadioReceive -> RF231driverC;
//   RF231driverC.RF231SpiC -> RF231Spic;

   components HplMsp430GeneralIOC;

}
