/**
 *	This interface allows the LinkEstimatorP to know, when it 
 * 	should it send the data 
**/
 

interface Capisrunning {

  /**
   * Tells that the CAP has started 
   *
   * @param 'uint32_t t0'  - start time of the current Super Frame.
   * @param 'uint32_t dt'  - CAP duration less one time slot duration
   * @return void.
   */
  
  async event void Caphasstarted(uint32_t t0 , uint32_t dt);

  /**
   * Tells tha CAP has finished
   *
   * @param void.
   * @return void.
   */

  async event void Caphasfinished(void);
 
  /**
   * Tells that my TS has begun
   *
   * @param void.
   * @return void.
   */

  async event void MyTShasStarted(void);
  
   
}