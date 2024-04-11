#!/bin/bash

pico_device=/dev/disk/by-id/usb-RPI_RP2_E0C9125B0D9B-0\:0-part1
pico_bin=$1
if [[ ! -f $pico_bin ]]
then
  echo "Usage ./flash.sh bin_name"
  exit 1
fi

if [[ ! -L "$pico_device" ]]
then
  echo "Device "$pico_device" doesn't exist"
  exit 1
fi

echo "Device "$pico_device" found! Flashing..."

sudo dd if=$pico_bin of="$pico_device" bs=512 oflag=sync
