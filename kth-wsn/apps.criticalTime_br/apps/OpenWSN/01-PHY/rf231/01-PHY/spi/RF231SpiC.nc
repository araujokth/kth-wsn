generic configuration RF231SpiC() {

  provides interface Resource;
  provides interface ChipSpiResource;

}

implementation {

  enum {
    CLIENT_ID = unique( "RF231Spi.Resource" ),
  };
  components RF231SpiP, LedsC;
  //components HplRF231PinsC as Pins;
  components RF231SpiWireC as Spi;
  components BusyWaitMicroC as spiWaitC;


  ChipSpiResource = Spi.ChipSpiResource;
  Resource = Spi.Resource[ CLIENT_ID ];
  RF231SpiP.Leds -> LedsC;
  RF231SpiP.spiWait -> spiWaitC;
}

