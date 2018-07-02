CONTIKI_PROJECT = coap-example-server coap-example-client
# use target "plugtest-server" explicitly when required

all: $(CONTIKI_PROJECT)

CONTIKI=../..

# Do not try to build on Sky because of code size limitation
PLATFORMS_EXCLUDE = sky

# build RESTful resources
include $(CONTIKI)/Makefile.identify-target
ifeq ($(TARGET),native)
	MODULES_REL += ./resources-plugtest
endif
MODULES_REL += ./resources
MODULES_REL += $(TARGET)

# Include the CoAP implementation
MODULES += os/net/app-layer/coap


# Include the TSCH implementation
MAKE_MAC = MAKE_MAC_TSCH
MODULES += os/services/shell


include $(CONTIKI)/Makefile.include


