COMPONENT=OpenWSNWiring
EXTRA_TOS = /home/david/workspace/tsch-project/tos
PFLAGS += -DTSCHDEBUG_ENABLED
CFLAGS += -I$(EXTRA_TOS)/interfaces \
		  -I$(EXTRA_TOS)/lib/debug \
#CFLAGS += -fnesc-dump=wiring -fnesc-dumpfile=wiring.xml

PFLAGS += -DPRINTF_BUFFER_SIZE=1000
CFLAGS += -I $(TOSDIR)/lib/printf
CFLAGS += -I $(TOSDIR)/platforms/telosb
CFLAGS += -I $(TOSDIR)/interfaces

CFLAGS += -I $(TOSROOT)/apps/CodeScheduling
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/interfaces


CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/07-App/appDATA/Device

CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/02b-RES/LatinRes
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/02b-RES/ures
#CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/02b-RES/neighbors

CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/02a-MAC
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/02a-MAC/cellusage
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/02a-MAC/LatinMac


CFLAGS += -DENABLE_SPI0_DMA #Enable DMA for CC2420 Packet Transfer (for transfering large packets?)
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/01-PHY/cc2420/
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/01-PHY/cc2420/control/
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/01-PHY/cc2420/interfaces/
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/01-PHY/cc2420/packet/
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/01-PHY/cc2420/receive/
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/01-PHY/cc2420/spi/
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/01-PHY/cc2420/cc2420driver/
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/01-PHY/cc2420/transmit/


CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/cross-layers/idmanager
CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/cross-layers/openqueue

CFLAGS += -I $(TOSROOT)/apps/CodeScheduling/cross-layers/packetfunctions


BUILD_EXTRA_DEPS += TokenRequestMsg.class TokenTransferredMsg.class

TokenTransferredMsg.java:  
	mig java -target=telosb -java-classname=TokenTransferredMsg LATIN.h TokenTransferredMsg -o$@

TokenTransferredMsg.class: TokenTransferredMsg.java
	javac TokenTransferredMsg.java

TokenRequestMsg.class: TokenRequestMsg.java
	javac TokenRequestMsg.java

TokenRequestMsg.java:  
	mig java -target=telosb -java-classname=TokenRequestMsg LATIN.h TokenRequestMsg -o$@
	
include $(MAKERULES)
