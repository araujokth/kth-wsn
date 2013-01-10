COMPONENT=EasyCollectionAppC
#CFLAGS += -DCC2420_DEF_CHANNEL=16

#CFLAGS += -DCC2420_DEF_RFPOWER=31

CFLAGS += -I$(TOSDIR)/lib/printf
CFLAGS += -I$(TOSDIR)/lib/net \
          -I$(TOSDIR)/lib/net/le \
          -I$(TOSDIR)/lib/net/ctp

PFLAGS += -DPRINTF_BUFFER_SIZE=1024
         
include $(MAKERULES)
