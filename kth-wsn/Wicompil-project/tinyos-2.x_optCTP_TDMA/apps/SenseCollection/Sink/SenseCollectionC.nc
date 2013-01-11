#include <Timer.h>
#include "EColl.h"
#include "printf.h"
#include "TreeRouting.h"
#include "CC2420.h"
#include "AM.h"
#include "UserButton.h"

module SenseCollectionC {
  uses interface Boot;
  uses interface SplitControl as RadioControl;
  uses interface StdControl as RoutingControl;
  uses interface Send;
  uses interface Leds;
  
  /* uses interface Timer<TMilli> as TimerPhase; */
  /* uses interface Timer<TMilli> as TimerWait; */
  /* uses interface Timer<TMilli> as TimerSensor; */
  uses interface Timer<TMilli> as TimerSPI;
  //uses interface LocalTime<TMilli>;
  uses interface Capisrunning;
  uses interface RootControl;
  uses interface Receive;
  uses interface CtpInfo as GetCTPData;
  uses interface CollectionPacket;
  uses interface CtpCongestion as Cngstn;
  uses interface CC2420Packet as RadioInterface;
  
  /* uses interface GeneralIO as ADC0; */
  /* uses interface GeneralIO as ADC1; */
  uses  interface Notify<button_state_t> as UserButton;

  /* uses interface Resource; */
  /* uses interface Msp430Adc12MultiChannel as MultiControllertoSFChannel; */
  
  uses interface SplitControl as SerialControl;
  uses interface AMSend as UartSend;
  uses interface AMPacket as UartAMPacket;
  uses interface Packet;

  /* uses interface LocalTime<TSymbolIEEE802154>;   */
  /* provides interface AdcConfigure<const msp430adc12_channel_config_t*>; */
  //uses interface LinkEstimator;
}
implementation {
  message_t packet;
  bool sendBusy = FALSE;
  /* uint16_t seqno = 0; */
  /* uint16_t phasecounter=0;  */
  /*********************************************************************
   * 2) Variable definition
   *********************************************************************/
  message_t frametoSF;
  // LAB3 Variable definition 
  // NOTE: do not remove, just add more if needed
  /* uint16_t buffer[3]; */
  /* uint16_t u;	// control input */
  /* uint16_t u0;	// control input in the end of 1st phase */
  
  /* uint16_t kp;	// proportional gain */
  /* uint16_t ki;	// integral gain	 */
  /* uint16_t kd;	// derivative gain */
  
  /* uint16_t pc;	// proportional part of u */
  /* uint16_t ic;	// integral part of u	 */
  /* uint16_t dc;	// derivative part of u */
  
  /* nx_float x_ref; */
  //nx_float op_point;	// operating point for 1st phase
  /* nx_float outf;	// auxiliary variable */
  /* uint16_t e;	// output error */
  
  /* float x1;	// upper tank	 */
  /* float x2; 	// lower tank */
  
  /* float tank1;	// upper tank	 */
  /* float tank2; 	// lower tank */
  
  /* float x1_p;	// this is a printing variable */
  /* float x2_p;	// this is a printing variable */
  /* float u_p;	// this is a printing variable */
  /* float pc_p;	// this is a printing variable */
  /* float ic_p;	// this is a printing variable */
  /* float dc_p;	// this is a printing variable */
  
  /* uint16_t beta; //  1v in the pump is approx. 273 units in the DAC */
  
  // Other variables
  /* message_t pktToActuator; */
 
  /* nx_float x_int; */
  
  /* nx_float K[3]; */
  /* uint16_t i_p; */
  
  /* uint8_t m_state; */
  /* uint8_t m_phase; */
  
  /* bool relay; */
  /* bool start; */
  bool networkestablished;
  
  SerialMsg* data_to_SF;   // Send data from Controller to SF
  uint8_t m_payloadLentoSF = sizeof(SerialMsg);
  
  uint32_t prev_time = 0;

  /* bool NetEst = FALSE; */
  /* const uint32_t WaitTime[4] = {25000, 55000, 85000, 115000}; */
  /* uint8_t break_cnt = 0; */

  /* enum { */
  /*   STOPPED = 0x01, // Wait until pressing the user button */
  /*   RUNNING = 0x2, // Start of the experiment  */
  /* }; */
  
  /* enum { */
  /*   PHASE_0 = 0x0, */
  /*   PHASE_1 = 0x01, // In this phase there is a default controller for reaching the INITIAL_REFERENCE */
  /*   PHASE_2 = 0x02, // In this phase a new controller should be implemented for reaching the END_REFERENCE */
  /* }; */
  void printfFloat(float toBePrinted); // For printing out float numbers
  /* const msp430adc12_channel_config_t config = { */
  /*   INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5, */
  /*   SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES, */
  /*   SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1 */
  /* }; */
  
  event void Boot.booted() {		
    /* x1 = 0; */
    /* x2 = 0; */
    /* u = 0; */
    /* x_ref = OP_POINT; // defined as OP_POINT for 1st phase, and it will be changed to REFERNENCE in 2nd phase */
    /* beta = 273; */
    
    /* // printing variables */
    /* i_p = 0; */
    /* x1_p = 0; */
    /* x2_p = 0; */
    /* u_p = 0; */
    /* pc_p = 0; */
    /* ic_p = 0; */
    /* dc_p = 0; */
    
    /* m_state = STOPPED; */
    /* m_phase = PHASE_0; */
    /* Act_active=TRUE; */
    /* start = TRUE; */
    networkestablished=TRUE;
    
    /* K[0] =  -1.1437;  */
    /* K[1] = -3.3787 ; */
    /* K[2] = -0.3162; */
    /* atomic { */
    /*   ADC12CTL0 = REF2_5V +REFON; */
    /*   DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC; */
    /* } */
    data_to_SF =(SerialMsg*)(call Packet.getPayload(&frametoSF,sizeof(SerialMsg)));
    data_to_SF->data = 0;
    /* prev_time = call LocalTime.get(); */
    data_to_SF->tinterval = 0;
    
    /* call ADC0.makeInput(); */
    /* call ADC1.makeInput(); */
    call UserButton.enable();
  }
  
  event void RadioControl.startDone(error_t err) {
    if (err != SUCCESS){
      call RadioControl.start();
    }
    else {
      call RoutingControl.start();
      
      /* call TimerSensor.startPeriodic(SENSE_INTERVAL); */
      if (TOS_NODE_ID == COORDINATOR_ADDRESS) { call RootControl.setRoot();
	//  printf("This is root \n"); printfflush();
      }    
    }
  }
  
  event void RadioControl.stopDone(error_t err) {}
  
  event void Send.sendDone(message_t* m, error_t err) {
    //if (err != SUCCESS) { call Leds.led0On(); }
    sendBusy = FALSE;
  }
  
  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    uint32_t current_time = 0;
    /* current_time = call LocalTime.get(); */

    if(networkestablished)
      {
	/* call TimerPhase.startOneShot(PHASE1_DURATION);	 */
	networkestablished=FALSE;
      }
    
    if (len == sizeof(ECollMessage)) {
      ECollMessage* svpkt = (ECollMessage*) payload;
      call Leds.led1Toggle();
      
      atomic{
	data_to_SF->data = svpkt->seqno;
	data_to_SF->tinterval = current_time - prev_time;
      }
      prev_time = current_time;

    /*   atomic { */
    /* 	// Store sensor values */
    /* 	x1 = (svpkt->data[0])/(WT_CALIBRATION); */
    /* 	x2 = (svpkt->data[1])/(WT_CALIBRATION); */
	
    /* 	// Save the sensor values in global variables for printing them out later on */
    /* 	x1_p = x1; */
    /* 	x2_p = x2; */
	
    /* 	switch (m_phase) {	   */
    /* 	case PHASE_1: */
	  
    /* 	  x_int += ((nx_float) SAMPLING_PERIOD/1000 * (x2 - x_ref)); */
	  
    /* 	  outf = (273.0 *( (x1 - x_ref) * K[0] + (x2-x_ref) * K[1] + x_int * K[2] )); */
	  
    /* 	  if(outf < 0) outf = 0; */
    /* 	  else if(outf > 4095) outf = 4095; */
	  
    /* 	  u = (uint16_t) outf; */
    /* 	  u0 = u;	   */
    /* 	  break; */
	  
    /* 	  /\********************************************************************* */
    /* 	   * 4.2) MR: Controller Implementation */
    /* 	   *********************************************************************\/ */
	  
    /* 	case PHASE_2:	   */
    /* 	  x_int += ((nx_float) SAMPLING_PERIOD/1000 * (x2 - x_ref)); */
	  
    /* 	  outf = (273.0 *( (x1 - x_ref) * K[0] + (x2-x_ref) * K[1] + x_int * K[2] )); */
	  
    /* 	  if(outf < 0) outf = 0; */
    /* 	  else if(outf > 4095) outf = 4095; */
	  
    /* 	  u = (uint16_t) outf; */
    /* 	  u0 = u; */
    /* 	  //apkt->u = u; */
	  
    /* 	  // define variables to be printed */
	  
    /* 	  break; */
	  
    /* 	default: */
    /* 	  break; */
    /* 	} */
    /* 	/\********************************************************************* */
    /* 	 * END MR: Controller Implementation */
    /* 	 *********************************************************************\/ */
    /* 	u_p = ((float)u)/beta; */
    /* 	if(Act_active) */
    /* 	  { atomic { */
    /* 	      DAC12_0DAT = u; */
    /* 	    }} */
    /* 	data_to_SF->voltage=u; */
    /* 	//printf(" message received \n");printfflush(); */
    /*   } */
    }else{
      call Leds.led0Toggle();
    }
    return msg;
  }

  event void TimerSPI.fired(){
    /* if (call UartSend.send(AM_BROADCAST_ADDR, &frametoSF,m_payloadLentoSF) == SUCCESS) { */
    /*   call Leds.led2Toggle(); */
    /* } */
    call Leds.led2Toggle();
  }

  /* event void TimerSensor.fired(){ */
  /*   call MultiChannel.getData(); */
  /* } */

  /* event void TimerWait.fired(){ */
  /*   m_phase = PHASE_1; */
  /*   x_ref = OP_POINT; */
  /*   call TimerPhase.startOneShot(PHASE1_DURATION); */
  /* } */
  
  /* event void TimerPhase.fired() { */
  /*   phasecounter++; */
  /*   /\**	 if( (phasecounter%2)==0  ) */
  /* 	 {  */
  /* 	 m_phase = PHASE_1; */
  /* 	 x_ref = OP_POINT; */
  /* 	 } */
  /* 	 else{ */
  /* 	 m_phase = PHASE_2; */
  /* 	 x_ref = REFERENCE; */
  /* 	 }**\/ */
  /*   m_phase = PHASE_2; */
  /*   x_ref = REFERENCE; */
  /*   call TimerWait.startOneShot(PHASE2_DURATION); */
  /* } */

  /*********************************************************************
	 	8) * ADDITIONAL TOOLS (For printing out float values)
  **********************************************************************/
  void printfFloat(float toBePrinted) {
    uint32_t fi, f0, f1, f2;
    char c;
    float f = toBePrinted;
    
    if (f<0) {
      c = '-'; f = -f;
    } else {
      c = ' ';
    }
    
    // integer portion.
    fi = (uint32_t) f;
    
    // decimal portion...get index for up to 3 decimal places.
    f = f - ((float) fi);
    f0 = f*10; f0 %= 10;
    f1 = f*100; f1 %= 10;
    f2 = f*1000; f2 %= 10;
    printf("%c%ld.%d%d%d", c, fi, (uint8_t) f0, (uint8_t) f1,
	   (uint8_t) f2);
  }
  
  /* async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples) */
  /* { */
  /*   Act_active=TRUE; */
  /*   atomic{ */
  /*     tank1 = buf[1]; //buf[1]  data[0] Lower tank */
  /*     tank2 = buf[2]; //buf[2] data[1] Upper Tank tank */
  /*     x1_p = tank1/(WT_CALIBRATION); */
  /*     x2_p = tank2/(WT_CALIBRATION); */
  /*   } */
    
  /*   if(x1_p>=MAX_LEVEL) */
  /*     { */
  /* 	atomic{DAC12_0DAT = 0;} */
  /* 	Act_active=FALSE; */
  /*     } */
    
  /*   data_to_SF-> TankLevel[0]=tank1; */
  /*   data_to_SF-> TankLevel[1]=tank2; */
     
  /*   if (call UartSend.send(AM_BROADCAST_ADDR, &frametoSF,m_payloadLentoSF) == SUCCESS) { */
  /*     /\* call Leds.led0Toggle(); *\/ */
  /*   }  */
    
    /* if(PRINT) */
    /*   { printfFloat(x1_p);printf(" "); printfFloat(x2_p);printf(" ");printfFloat(u_p);printf(" \n "); */
    /* 	printfflush(); } */
    
  /* } */
  //when readtemp request has been processed
  /* async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration() */
  /* { */
  /*   return &config; */
  /* } */
  
  /***** Finish Reading Data from Sensors **************/
  /*********************************************************************
   *                            Resource
   **********************************************************************/
  /* event void Resource.granted() */
  /* {   */
  /*   atomic { */
  /*     adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}}; */
      
  /*     if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) != SUCCESS) { */
  /* 	//call Leds.led0On(); */
  /*     } */
  /*   } */
    
  /* } */
  
  /* event void TimerPhase.fired(){ */
  /*   printf("T ##### breaking Mote %u #####\n", break_cnt+2); */
  /*   printfflush(); */
  /*   break_cnt++; */
  /*   if(break_cnt < 4){ */
  /*     call TimerPhase.startOneShot(WaitTime[break_cnt]); */
  /*   } */
  /* } */

  async event void Capisrunning.Caphasstarted(uint32_t t0 , uint32_t dt){
    /* call MultiChannel.getData(); */
    /* if(!NetEst){ */
    /*   NetEst = TRUE; */
    /*   call TimerPhase.startOneShot(WaitTime[break_cnt]); */
    /* } */
  }
  
  async event void Capisrunning.Caphasfinished(){}
  
  async event void Capisrunning.MyTShasStarted(){ }
  
  /*********************************************************************
   * 7) User Button
   *********************************************************************/
  event void UserButton.notify( button_state_t state ) {
    if ( state == BUTTON_PRESSED ) {
    } else if ( state == BUTTON_RELEASED ) {
      call RadioControl.start();
      /* call Resource.request(); */
      /* call SerialControl.start(); */
      /* call TimerWait.startOneShot(PHASE0_DURATION); */
      call TimerSPI.startPeriodic(SENSE_INTERVAL);
    }
  }
  
  event void SerialControl.startDone(error_t error) {}
  
  event void SerialControl.stopDone(error_t error) {}
  
  event void UartSend.sendDone(message_t* msg, error_t error) {}
}  // Implementation ends
