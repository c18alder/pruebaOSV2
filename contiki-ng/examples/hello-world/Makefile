CONTIKI_PROJECT = hello-world
all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

# openmote-b platform makefile

ifndef CONTIKI
  $(error CONTIKI not defined! You must specify where CONTIKI resides!)
endif

### Configure the build for the board and pull in board-specific sources
CONTIKI_TARGET_DIRS += . dev
PLATFORM_ROOT_DIR = $(CONTIKI)/arch/platform/$(TARGET)

### Include
CONTIKI_TARGET_SOURCEFILES += platform.c board.c
CONTIKI_TARGET_SOURCEFILES += leds-arch.c board-buttons.c openmote-sensors.c
CONTIKI_TARGET_SOURCEFILES += antenna.c si70x.c

CONTIKI_SOURCEFILES += $(CONTIKI_TARGET_SOURCEFILES)

CLEAN += *.openmote-b

### Define the CPU directory
CONTIKI_CPU=$(CONTIKI)/arch/cpu/cc2538
include $(CONTIKI_CPU)/Makefile.cc2538

MODULES += os/storage/cfs

PYTHON = python
BSL_FLAGS += -e --bootloader-invert-lines -w -v -b 450000

ifdef PORT
  BSL_FLAGS += -p $(PORT)
endif

BSL = $(CONTIKI)/tools/cc2538-bsl/cc2538-bsl.py

%.upload: %.bin %.elf
ifeq ($(wildcard $(BSL)), )
	@echo "ERROR: Could not find the cc2538-bsl script. Did you run 'git submodule update --init' ?"
else
	$(eval BSL_ADDRESS_ARG := -a $(shell $(OBJDUMP) -h $*.elf | grep -B1 LOAD | \
	                               grep -Ev 'LOAD|\-\-' | awk '{print "0x" $$5}' | \
	                               sort -g | head -1))
	$(PYTHON) $(BSL) $(BSL_FLAGS) $(BSL_ADDRESS_ARG) $<
endif
