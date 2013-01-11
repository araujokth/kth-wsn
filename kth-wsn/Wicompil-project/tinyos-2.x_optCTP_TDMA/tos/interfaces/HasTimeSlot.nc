/**
 *	This interface allows the CtpRoutngEngine, whether a node has a TS allocated or not
**/
 

interface HasTimeSlot{

  /**
   * Tells whether there is a timeslot for us
   *
   * @param void
   * @return 'bool' - whether we have timeslot for transmission or not .
   */
  
  command bool hasTS(void);  
  
  /**
   * Tells the AM layer the schedule
   *
   * @param   'uint8_t* ONE Schedule'   the schedule to AM Layer
   * @param    'uint8_t nodes'   Number of nodes in discovered in the network so far
   * @return          SUCCESS if the request was accepted and will issue
   *                  a sendDone event, EBUSY if the component cannot accept
   *                  the request now but will be able to later, FAIL
   *                  if the stack is in a state that cannot accept requests
   *                  (e.g., it's off).   */
  
  command error_t setSchedule(uint8_t* Schedule, uint8_t nodes);
  
  /**
   * Get the schedule from the AM layer
   *
   * @param   'uint8_t* ONE Schedule'   the schedule from AM Layer
   * @return          SUCCESS if the request was accepted and will issue
   *                  a sendDone event, EBUSY if the component cannot accept
   *                  the request now but will be able to later, FAIL
   *                  if the stack is in a state that cannot accept requests
   *                  (e.g., it's off).   */
  
  command uint8_t getSchedule(uint8_t* Schedule);
   
}