COMPONENT=SnifferAppC
CFLAGS += -DIEEE154_SCAN_DISABLED \
		  -DIEEE154_BEACON_SYNC_DISABLED \
		  -DIEEE154_BEACON_TX_DISABLED \
		  -DIEEE154_RXENABLE_DISABLED \
		  -DIEEE154_ASSOCIATION_DISABLED \
		  -DIEEE154_DISASSOCIATION_DISABLED \
		  -DIEEE154_COORD_REALIGNMENT_DISABLED \
		  -DIEEE154_COORD_BROADCAST_DISABLED \
		  -DTSCH_RESYNC_DISABLED 
		  
		  CFLAGS += -I$(TOSDIR)/lib/printf
		  
#CFLAGS += -DSNIFFER_MODE
		  
BUILD_EXTRA_DEPS += TokenRequestMsg.class TokenTransferredMsg.class

TokenTransferredMsg.java:  
	mig java -target=telosb -java-classname=TokenTransferredMsg app_profile.h TokenTransferredMsg -o$@

TokenTransferredMsg.class: TokenTransferredMsg.java
	javac TokenTransferredMsg.java

TokenRequestMsg.class: TokenRequestMsg.java
	javac TokenRequestMsg.java

TokenRequestMsg.java:  
	mig java -target=telosb -java-classname=TokenRequestMsg app_profile.h TokenRequestMsg -o$@
	
include ../Makefile.include
