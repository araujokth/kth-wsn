interface GlobalSync {
  async command bool getIsSync();
        event   void losingSync();
        event   void lostSync();
}
