#!/bin/bash

set -e

device=$1

if [[ -z $device ]]; then
  echo "please provide device name. Usage \`install.sh [devicename]\`"
  exit 1
fi

pio run
scp .pio/build/ATtiny1616/firmware.hex $device:
scp decode_log.py $device:
ssh $device "pymcuprog -d attiny1616 -t uart -u /dev/serial0 erase"
ssh $device "systemctl restart battery-logger || true"
ssh $device "pymcuprog -d attiny1616 -t uart -u /dev/serial0 write -f firmware.hex"

echo "Installed on device."
