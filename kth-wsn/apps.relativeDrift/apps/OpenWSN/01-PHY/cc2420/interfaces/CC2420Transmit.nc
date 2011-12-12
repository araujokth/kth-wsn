interface CC2420Transmit {
  async command error_t loadPacket(OpenQueueEntry_t* msg);
  async event   void    loadPacketDone(error_t error);
  async command error_t sendNow(bool useCca);
  async event   void    sendNowDone(error_t error);
}
