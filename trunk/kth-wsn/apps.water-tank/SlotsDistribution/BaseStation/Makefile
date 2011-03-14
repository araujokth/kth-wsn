COMPONENT=ControllerAppC
CFLAGS += -DWT_USERBUTTON

CFLAGS += -I$(TOSDIR)/lib/printf
PFLAGS += -DPRINTF_BUFFER_SIZE=1000
CFLAGS += -I$(shell pwd)/..
CFLAGS += -I$(shell pwd)/../..

BUILD_EXTRA_DEPS += SensorMatrixMsg.class ActuationMatrixMsg.class

ActuationMatrixMsg.java:  
	mig java -target=telosb -java-classname=ActuationMatrixMsg ../app_sensors_nbe.h ActuationMatrixMsg -o$@

ActuationMatrixMsg.class: ActuationMatrixMsg.java
	javac ActuationMatrixMsg.java

SensorMatrixMsg.class: SensorMatrixMsg.java
	javac SensorMatrixMsg.java

SensorMatrixMsg.java:  
	mig java -target=telosb -java-classname=SensorMatrixMsg ../app_sensors_nbe.h SensorMatrixMsg -o$@

include ../Makefile.include