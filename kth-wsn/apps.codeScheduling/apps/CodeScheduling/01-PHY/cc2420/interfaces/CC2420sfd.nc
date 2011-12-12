interface CC2420sfd {

  /**
   * Notification that an SFD capture has occured.
   *
   * @param time at which the capture happened.
   */
  async command void sfd( uint32_t time );

  /**
   * Notification that the packet has been dropped by the radio
   * (e.g. due to address rejection).
   */
  async command void sfd_dropped();

}
