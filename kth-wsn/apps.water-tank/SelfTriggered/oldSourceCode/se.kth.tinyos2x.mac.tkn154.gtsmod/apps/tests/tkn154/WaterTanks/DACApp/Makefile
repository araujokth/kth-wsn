COMPONENT=PowerPlugAppC
CFLAGS += -I$(shell pwd)/..
PFLAGS += -DIEEE154_BEACON_TX_DISABLED #-DTKN154_DEBUG

#enable print function
CFLAGS += -I$(TOSDIR)/lib/printf
PFLAGS += -DPRINTF_BUFFER_SIZE=1000


BUILD_EXTRA_DEPS += MyMsg.class
CLEAN_EXTRA = *.class *~ MyMsg.java

MyMsg.class: $(wildcard *.java) MyMsg.java
	javac *.java

MyMsg.java:  
	mig java -target=null -java-classname=MyMsg PowerPlug.h MyMsg -o$@



include ../Makefile.include
