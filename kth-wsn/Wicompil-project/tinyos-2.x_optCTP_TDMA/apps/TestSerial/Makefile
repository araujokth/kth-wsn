COMPONENT=TestSerialAppC
# BUILD_EXTRA_DEPS += TestSerial.class
BUILD_EXTRA_DEPS = PrintfMsg.class PrintfMsg.java
CLEAN_EXTRA = *.class PrintfMsg.java

# CFLAGS += -I$(TOSDIR)/lib/T2Hack

PRINTF_H = $(TOSDIR)/lib/printf/printf.h

PrintfMsg.class: $(wildcard *.java) PrintfMsg.java
	javac -target 1.4 -source 1.4 *.java

PrintfMsg.java: $(PRINTF_H) FORCE
	mig java $(CFLAGS) -java-classname=PrintfMsg $(PRINTF_H) printf_msg -o $@


include $(MAKERULES)

