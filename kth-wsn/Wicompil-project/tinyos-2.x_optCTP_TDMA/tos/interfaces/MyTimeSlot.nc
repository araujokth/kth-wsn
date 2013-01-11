/**
 *	This interface allows the LinkEstimatorP to know, when it 
 * 	should it send the data 
**/
 

interface MyTimeSlot {

  /**
   * Tells that the Forwarding engine that our TS has started 
   *
   * @param void.
   * @return void.
   */
  
  async event void hasStarted(void);
  
   
}