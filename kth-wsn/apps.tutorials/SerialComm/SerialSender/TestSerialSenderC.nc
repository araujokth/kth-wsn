
configuration TestSerialSenderC {
}
implementation {
	enum {
		CLIENT_ID = unique( "RS232.Resource" ),
	};
	components MainC;

	components new TimerMilliC() as Timer;
	components TestSerialSenderP as Enc;

	Enc.TimerSamples -> Timer;
	Enc.Boot -> MainC.Boot;

	components LedsC;
	Enc.Leds -> LedsC;

	components new Msp430Uart0C() as UartC;
	Enc.UartStream -> UartC.UartStream;
	Enc.UartResource -> UartC.Resource;
	Enc.Msp430UartConfigure <- UartC.Msp430UartConfigure;
	
	components RandomC;
	Enc.Random -> RandomC;

}
