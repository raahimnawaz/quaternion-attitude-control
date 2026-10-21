# make -C test            host SIL tests (real sketch.ino + real chip/chip.chip.c)
# make -C test avr-bench  CPU cycle counts on ATmega2560 under simavr
# make -C test before-after  same, plus the SIL timing table, for BASELINE vs now
# make -C test plot       results/unwinding-359.svg from SIL traces
#
# All of these use the Adafruit libraries PlatformIO fetched into
# ../.pio/libdeps, so run `pio run` once first.

ROOT     := $(abspath ..)
BUILD    := $(ROOT)/test/build
LIBDEPS  := $(ROOT)/.pio/libdeps/megaatmega2560
PIOPKG   := $(HOME)/.platformio/packages
BASELINE := 73c03f0

GFX     := $(LIBDEPS)/Adafruit GFX Library
SSD1306 := $(LIBDEPS)/Adafruit SSD1306
BUSIO   := $(LIBDEPS)/Adafruit BusIO

# --- host SIL ---------------------------------------------------------------
HOSTCXX  ?= c++
HOSTCC   ?= cc
HOSTFLAGS := -O2 -g -Wall -Wno-unused-function -Wno-unused-variable -DARDUINO=10808
HOSTINC   := -I$(ROOT)/test/sim -I"$(GFX)" -I"$(SSD1306)" -I"$(BUSIO)"
LIBSRC    := "$(GFX)/Adafruit_GFX.cpp" "$(SSD1306)/Adafruit_SSD1306.cpp"

.PHONY: test avr-bench before-after plot clean
test: $(BUILD)/sil
	$(BUILD)/sil

$(BUILD)/legacy_sketch.ino:
	@mkdir -p $(BUILD)
	git -C $(ROOT) show $(BASELINE):sketch.ino > $@

$(BUILD)/plant.o: sim/plant.c $(ROOT)/chip/chip.chip.c sim/wokwi-api.h
	@mkdir -p $(BUILD)
	$(HOSTCC) -O2 -g -std=c11 -I$(ROOT)/test/sim -c $< -o $@

$(BUILD)/sil: sim/sim.cpp sim/firmware.cpp sil.cpp sim/*.h $(BUILD)/plant.o \
              $(ROOT)/sketch.ino $(BUILD)/legacy_sketch.ino
	$(HOSTCXX) $(HOSTFLAGS) -std=c++17 $(HOSTINC) -DLEGACY_SKETCH='"$(BUILD)/legacy_sketch.ino"' \
	  sil.cpp sim/sim.cpp sim/firmware.cpp $(LIBSRC) $(BUILD)/plant.o -o $@

# --- AVR cycle counts under simavr -------------------------------------------
AVRCXX   := $(PIOPKG)/toolchain-atmelavr/bin/avr-g++
SIMAVR   := $(PIOPKG)/tool-simavr/bin/simavr
AVRCORE  := $(PIOPKG)/framework-arduino-avr
PIOBUILD := $(ROOT)/.pio/build/megaatmega2560
AVRFLAGS := -fno-exceptions -fno-threadsafe-statics -fpermissive -std=gnu++11 \
            -mmcu=atmega2560 -Os -Wall -ffunction-sections -fdata-sections -flto \
            -DF_CPU=16000000L -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR -DARDUINO=10808
AVRINC   := -I$(ROOT)/test/avr -I$(AVRCORE)/cores/arduino -I$(AVRCORE)/variants/mega \
            -I$(AVRCORE)/libraries/SPI/src -I"$(GFX)" -I"$(SSD1306)" -I"$(BUSIO)"
AVRLIBS  := $(PIOBUILD)/libFrameworkArduino.a $(wildcard $(PIOBUILD)/*/libSPI.a) -lm

$(BUILD)/bench_%.elf: avr/bench.cpp avr/Wire.h $(ROOT)/sketch.ino $(BUILD)/legacy_sketch.ino
	@mkdir -p $(BUILD)
	$(AVRCXX) $(AVRFLAGS) $(AVRINC) -DSKETCH='"$(if $(filter legacy,$*),$(BUILD)/legacy_sketch.ino,$(ROOT)/sketch.ino)"' \
	  avr/bench.cpp $(LIBSRC) -o $@ -Wl,--gc-sections -fuse-linker-plugin $(AVRLIBS)

# simavr echoes the UART in colour; keep the results, drop the telemetry.
RUNAVR = $(SIMAVR) -m atmega2560 -f 16000000 $(1) 2>&1 | sed -e 's/\x1b\[[0-9;]*m//g' -e 's/\.\.$$//' \
         | grep -v -E '^-?[01]\.[0-9]{4},|^Loaded'

avr-bench: $(BUILD)/bench_now.elf
	@$(call RUNAVR,$<)

before-after: $(BUILD)/bench_legacy.elf $(BUILD)/bench_now.elf $(BUILD)/sil
	@echo "== CPU cycles, $(BASELINE) =="; $(call RUNAVR,$(BUILD)/bench_legacy.elf)
	@echo "== CPU cycles, working tree =="; $(call RUNAVR,$(BUILD)/bench_now.elf)
	@$(BUILD)/sil --timing

clean:
	rm -rf $(BUILD)

plot: $(BUILD)/sil
	python3 plot_unwinding.py
