interface GlobalTime {
  async command timervalue_t getGlobalSlotOffset();
  async command timervalue_t getLocalTime();
  async command asn_t getASN();
}
