interface LatinMatrix {
   async command bool  childCanTransmit(uint8_t minLatinWindow,uint8_t* latinChannelOffset);
   async command bool  siblingCanTransmit (uint8_t minLatinWindow, uint8_t* latinChannelOffset);
   async command bool  parentCanTransmit (uint8_t minLatinWindow, uint8_t* latinChannelOffset);
   async command bool  canTransmit (uint8_t minLatinWindow, uint8_t* latinChannelOffset);
   async command uint8_t  getParentSeed();
}
