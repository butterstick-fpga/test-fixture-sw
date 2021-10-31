BUILD_DIR?=..

include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak


OBJECTS   = isr.o donut.o helloc.o main.o crt0.o
ifdef WITH_CXX
	OBJECTS += hellocpp.o
endif

vpath % $(TESTROM_DIRECTORY) 

all: demo.bin

# pull in dependency info for *existing* .o files
-include $(OBJECTS:.o=.d)

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	chmod -x $@
ifeq ($(CPUENDIANNESS),little)
	$(PYTHON) -m litex.soc.software.mkmscimg $@ --little
else
	$(PYTHON) -m litex.soc.software.mkmscimg $@
endif

demo.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) \
		-T $(TESTROM_DIRECTORY)/linker.ld \
		-N -o $@ \
		$(OBJECTS) \
		$(LIBS:%=-L $(BUILDINC_DIRECTORY)/../%) \
		$(LIBS:lib%=-l%)
	chmod -x $@

main.o: main.c
	$(compile)

crt0.o: $(CPU_DIRECTORY)/crt0.S
	$(assemble)

donut.o: CFLAGS += -w

helloc.o: CFLAGS += -w

hellocpp.o: CXXFLAGS += -w

%.o: %.cpp
	$(compilexx)

%.o: %.c
	$(compile)

%.o: %.S
	$(assemble)

clean:
	$(RM) $(OBJECTS) $(OBJECTS:.o=.d) demo.elf demo.bin .*~ *~

.PHONY: all main.o clean load
