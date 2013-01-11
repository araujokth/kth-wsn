COMPONENT=EasyCollectionAppC
PLATFORM=telosb
#CFLAGS += -DCC2420_DEF_CHANNEL=16
#CFLAGS += -DCC2420_DEF_RFPOWER=7

PFLAGS += -DIEEE154_SCAN_DISABLED \
		  -DIEEE154_BEACON_SYNC_DISABLED

CFLAGS += -I$(shell pwd)/..
CFLAGS += -I$(TOSDIR)/lib/printf
PFLAGS += -DPRINTF_BUFFER_SIZE=512

CFLAGS += -I$(TOSDIR)/lib/net \
          -I$(TOSDIR)/lib/net/le_tdma \
          -I$(TOSDIR)/lib/net/ctp_tdma


# To use the TKN15.4 MAC instead of a platform's default MAC protocol first
# include the TinyOS "Makerules" file as usual ...
include $(MAKERULES)

include $(TOSCONTRIB)/kth/tkn154-gts-mod-all/tos/lib/mac/tkn154-sink/Makefile.include

# ... and then include the TKN15.4 "Makefile.include" file. That's all.
# Hint: type "make <platform> verbose" to see the aggregate include path.
include $(TOSDIR)/lib/mac/tkn154/Makefile.include
