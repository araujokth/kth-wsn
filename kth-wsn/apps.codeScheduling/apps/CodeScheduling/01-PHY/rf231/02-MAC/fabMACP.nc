module fabMACP @safe()
{
  uses interface Timer<TMilli> as Timer;
  uses interface Leds;
  uses interface Boot;
  uses interface RadioControl;
  uses interface RadioSend;
  uses interface RadioReceive;
}
implementation
{
  event void Boot.booted()
  {
//      call Leds.led0Toggle();
//      call Leds.led1Toggle();
//      call Leds.led2Toggle();
//      call Leds.led3Toggle();//all leds on
      call RadioControl.start();
  }
      
  event void RadioControl.startDone(error_t error) {
    uint8_t packet[10];
//    call Leds.led3Toggle();//3off
    packet[0] = 0x06;// length: deadbeef + length_byte + crc(2)
    packet[1] = 0x0E; //this is the channel
    packet[2] = 0xDE;
    packet[3] = 0xAD;
    packet[4] = 0xBE;
    packet[5] = 0xEF;
    call RadioSend.prepareSend(packet);
    //call RadioControl.prepareReceive(0x0E);

     
     
     
  }

   async event void RadioSend.prepareSendDone(error_t error){
//      call Leds.led2Toggle();//2,3off
      call RadioSend.sendNow();
   }
  
   event void RadioSend.sendNowDone(error_t error) {
      call Timer.startOneShot( 200 );
   }

  event void Timer.fired()
     {
//       call Leds.led0Toggle();//0,2,3off
       call RadioControl.stop();
     }

  event void RadioControl.stopDone( error_t error ){
//      call Leds.led1Toggle();//all off
      return;
  }


   ////////event void RadioReceive.receive(OpenQueueEntry_t* msg) {}
   event void RadioReceive.receive(uint8_t* msg) {
if (msg[0] == 6) //call Leds.led2Toggle();//2,3off
}
   
   async event void RadioControl.prepareReceiveDone(error_t error){
call RadioControl.receiveNow();
}
   async event void RadioControl.receivedNothing(){}

}
