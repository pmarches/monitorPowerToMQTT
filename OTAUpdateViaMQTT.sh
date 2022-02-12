#!/bin/bash
if [[ "$(mosquitto_sub --host vyachtwifi.lan -C 1 -t monitorPowerToMQTT/status)" == "offline" ]]; then
  echo "The vhfadapter is not online"
  exit 1
fi

cat build/monitorPowerToMQTT.bin | mosquitto_pub --host vyachtwifi.lan -t monitorPowerToMQTT/appUpdate/image -s
mosquitto_sub --host vyachtwifi.lan -C 3 -t monitorPowerToMQTT/status
echo "The monitorPowerToMQTT has been updated and has came back online"
