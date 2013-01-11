COMPONENT=RssiBaseAppC

INCLUDES= -I..               \
          -I../InterceptBase

CFLAGS += $(INCLUDES)
CFLAGS += -I$(TOSDIR)/lib/printf
ifneq ($(filter iris,$(MAKECMDGOALS)),) 
	CFLAGS += -DRF230_RSSI_ENERGY
endif

include $(MAKERULES)
