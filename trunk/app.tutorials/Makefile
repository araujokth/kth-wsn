COMPONENT=TestSerialCommAppC

BUILD_EXTRA_DEPS += TestSerialCommMsg.class

TestSerialCommMsg.java:  
	mig java -target=telosb -java-classname=TestSerialCommMsg ../app_profile.h TestSerialCommMsg -o $@
	
TestSerialCommMsg.class: TestSerialCommMsg.java
	javac TestSerialCommMsg.java

CFLAGS += -I$(shell pwd)/..

include $(MAKERULES)

