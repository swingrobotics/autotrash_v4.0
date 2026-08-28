#!/bin/sh
set -eu

SKETCH_PATH=${1:-/home/gnss/camera-stream/arduino/motor_serial/motor_serial.ino}
BUILD_PATH=${2:-/tmp/motor-serial-uno-build}

mkdir -p "$BUILD_PATH"
arduino-builder \
  -compile \
  -logger=human \
  -hardware /usr/share/arduino/hardware \
  -tools /usr/bin \
  -built-in-libraries /usr/share/arduino/hardware/arduino/avr/libraries \
  -prefs=tools.ctags.path=/usr/bin \
  -prefs=tools.ctags.cmd.path=/usr/bin/arduino-ctags \
  -prefs='tools.ctags.pattern="{cmd.path}" -u --language-force=c++ -f - --c++-kinds=svpf --fields=KSTtzns --line-directives "{source_file}"' \
  -fqbn arduino:avr:uno \
  -build-path "$BUILD_PATH" \
  "$SKETCH_PATH"

avr-size --format=avr --mcu=atmega328p "$BUILD_PATH/motor_serial.ino.elf"
ls -lh "$BUILD_PATH/motor_serial.ino.hex"
