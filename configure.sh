#!/bin/bash

if [[ -z "$PICO_SDK_PATH" ]]
then
  echo "PICO_SDK_PATH not set. export PICO_SDK_PATH with path to RP2040 SDK directory"
  exit 1
fi

if [[ ! -f wifi.conf ]]
then
cat > wifi.conf << EOF
WIFI_SSID=""
WIFI_PASSWORD=""
EOF
echo "Fill wifi.conf file with ssid and password"
fi

wifi_ssid="$(grep WIFI_SSID wifi.conf | awk  -F= '{ print $2 }' | sed 's/\"//g')"
wifi_pass="$(grep WIFI_PASS wifi.conf | awk  -F= '{ print $2 }' | sed 's/\"//g')"

if [[ -z $wifi_ssid ]] 
then
  echo "Modify wifi.conf script and set ssid before configuring project"
  exit 1
fi

if [[ -z $wifi_pass ]] 
then
  echo "Modify wifi.conf script and set pass before configuring project"
  exit 1
fi

echo "Current ssid" $wifi_ssid

cmake -B build -S . -DCMAKE_EXPORT_COMPILE_COMMANDS=Y -GNinja -DWIFI_SSID="$wifi_ssid" -DWIFI_PASSWORD="$wifi_pass" -DPICO_SDK_PATH="$PICO_SDK_PATH"
# end of access point check

if [[ ! -f ./compile_commands.json ]]
then
  ln -s ./build/compile_commands.json ./compile_commands.json
fi

cp ./flash.sh ./build
