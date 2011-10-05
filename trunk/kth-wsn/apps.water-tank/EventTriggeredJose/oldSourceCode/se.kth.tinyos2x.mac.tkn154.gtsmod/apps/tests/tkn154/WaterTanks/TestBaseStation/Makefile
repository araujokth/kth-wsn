COMPONENT=BaseStation154C

CFLAGS += -DCC2420_NO_ACKNOWLEDGEMENTS
CFLAGS += -DCC2420_NO_ADDRESS_RECOGNITION
CFLAGS += -DTASKLET_IS_TASK
CFLAGS += -I$(shell pwd)/..

#CFLAGS += -I$(TOSDIR)/lib/printf
#PFLAGS += -DPRINTF_BUFFER_SIZE=1000
	
BUILD_EXTRA_DEPS=EncMsg.class RateMsg.class

EncMsg.class: EncMsg.java
	javac EncMsg.java
	
EncMsg.java:
	mig java -target=telosb -java-classname=EncMsg ../app_sensors.h EncMsg -o $@
	
RateMsg.class: RateMsg.java
	javac RateMsg.java
	
RateMsg.java:
	mig java -target=telosb -java-classname=RateMsg ../app_sensors.h RateMsg -o $@

include ../../Makefile.include
