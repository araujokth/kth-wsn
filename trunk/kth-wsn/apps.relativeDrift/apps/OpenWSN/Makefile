COMPONENT=OpenWSNWiring
EXTRA_TOS = /home/david/workspace/tsch-project/tos
PFLAGS += -DTSCHDEBUG_ENABLED
CFLAGS += -I$(EXTRA_TOS)/interfaces \
		  -I$(EXTRA_TOS)/lib/debug \
#CFLAGS += -fnesc-dump=wiring -fnesc-dumpfile=wiring.xml

PFLAGS += -DPRINTF_BUFFER_SIZE=1000
CFLAGS += -I $(TOSDIR)/lib/printf

CFLAGS += -I $(TOSROOT)/apps/OpenWSN
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/interfaces


CFLAGS += -I $(TOSROOT)/apps/OpenWSN/07-App/appDATA/Device

CFLAGS += -I $(TOSROOT)/apps/OpenWSN/02b-RES/nres
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/02b-RES/ures
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/02b-RES/neighbors

CFLAGS += -I $(TOSROOT)/apps/OpenWSN/02a-MAC
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/02a-MAC/cellusage
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/02a-MAC/IEEE802154E


CFLAGS += -DENABLE_SPI0_DMA #Enable DMA for CC2420 Packet Transfer (for transfering large packets?)
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/01-PHY/cc2420/
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/01-PHY/cc2420/control/
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/01-PHY/cc2420/interfaces/
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/01-PHY/cc2420/packet/
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/01-PHY/cc2420/receive/
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/01-PHY/cc2420/spi/
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/01-PHY/cc2420/cc2420driver/
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/01-PHY/cc2420/transmit/


CFLAGS += -I $(TOSROOT)/apps/OpenWSN/cross-layers/idmanager
CFLAGS += -I $(TOSROOT)/apps/OpenWSN/cross-layers/openqueue

CFLAGS += -I $(TOSROOT)/apps/OpenWSN/cross-layers/packetfunctions


BUILD_EXTRA_DEPS += TokenRequestMsg.class TokenTransferredMsg.class

TokenTransferredMsg.java:  
	mig java -target=telosb -java-classname=TokenTransferredMsg OpenWSN.h TokenTransferredMsg -o$@

TokenTransferredMsg.class: TokenTransferredMsg.java
	javac TokenTransferredMsg.java

TokenRequestMsg.class: TokenRequestMsg.java
	javac TokenRequestMsg.java

TokenRequestMsg.java:  
	mig java -target=telosb -java-classname=TokenRequestMsg OpenWSN.h TokenRequestMsg -o$@
	
include $(MAKERULES)
