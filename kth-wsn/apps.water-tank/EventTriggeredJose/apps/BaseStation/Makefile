COMPONENT=BaseStation154C
	CFLAGS += -I$(shell pwd)/..
			  CFLAGS += -I$(TOSDIR)/lib/printf
	
BUILD_EXTRA_DEPS += WTInfoMsg.class SuperframeConfigMsg.class WTSensorValuesMsg.class ControlMsg.class

WTInfoMsg.java:  
	mig java -target=telosb -java-classname=WTInfoMsg ../app_sensors.h WTInfoMsg -o $@

SuperframeConfigMsg.java:  
	mig java -target=telosb -java-classname=SuperframeConfigMsg ../app_sensors.h SuperframeConfigMsg -o $@
	
WTSensorValuesMsg.java:  
	mig java -target=telosb -java-classname=WTSensorValuesMsg ../app_sensors.h WTSensorValuesMsg -o $@

ControlMsg.java:  
	mig java -target=telosb -java-classname=ControlMsg ../app_sensors.h ControlMsg -o $@
	
		
WTInfoMsg.class: WTInfoMsg.java
	javac WTInfoMsg.java
	
WTSensorValuesMsg.class: WTSensorValuesMsg.java
	javac WTSensorValuesMsg.java

SuperframeConfigMsg.class: SuperframeConfigMsg.java
	javac SuperframeConfigMsg.java
	
ControlMsg.class: ControlMsg.java
	javac ControlMsg.java

	
include ../Makefile.include
