configuration fabMACC
{
  uses interface Boot;
  uses interface RadioControl;
  uses interface RadioSend;
  uses interface RadioReceive;
}
implementation
{
  components fabMACP, LedsC;
  components new TimerMilliC() as TimerC;


  Boot =  fabMACP.Boot;
  RadioControl = fabMACP.RadioControl;
  RadioSend = fabMACP.RadioSend;
  RadioReceive = fabMACP.RadioReceive;
  fabMACP.Timer -> TimerC;
  fabMACP.Leds -> LedsC;
}
