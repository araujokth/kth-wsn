generic configuration HplRF231SpiC() {
  
  provides interface Resource;
  provides interface SpiByte;
  provides interface SpiPacket;
  
}

implementation {

  components new Msp430SpiA0C() as SpiC;
  
  Resource = SpiC;
  SpiByte = SpiC;
  SpiPacket = SpiC;
  
}

