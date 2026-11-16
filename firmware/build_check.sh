#!/bin/sh
# Compile-check the sketch for Arduino Uno with plain avr-gcc against the Arduino AVR core.
# Usage: ARDUINO_CORE=/path/to/ArduinoCore-avr sh firmware/build_check.sh
set -e
CORE=${ARDUINO_CORE:-../ArduinoCore-avr}
OUT=${OUT:-/tmp/daq_build}
mkdir -p "$OUT"
FLAGS="-mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10819 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR -Os -w -ffunction-sections -fdata-sections"
INC="-I$CORE/cores/arduino -I$CORE/variants/standard"
for f in $CORE/cores/arduino/*.c;   do avr-gcc $FLAGS $INC -c "$f" -o "$OUT/$(basename $f).o"; done
for f in $CORE/cores/arduino/*.cpp; do avr-g++ $FLAGS -fno-exceptions $INC -c "$f" -o "$OUT/$(basename $f).o"; done
cp firmware/waveform_daq/waveform_daq.ino "$OUT/sketch.cpp"
avr-g++ $FLAGS -fno-exceptions $INC -c "$OUT/sketch.cpp" -o "$OUT/sketch.o"
avr-gcc -mmcu=atmega328p -Wl,--gc-sections -o "$OUT/waveform_daq.elf" "$OUT"/*.o -lm
avr-size -C --mcu=atmega328p "$OUT/waveform_daq.elf"
