#!/bin/bash
# Started at boot (crontab @reboot): runs the operator site, restarting it if it ever stops.
# Drone link: a wired serial port if there is one, otherwise the Solo controller's Wi-Fi (SoloLink),
# which sends the drone's MAVLink to UDP 14550.
cd ~/litterbug
while true; do
  if [ -e /dev/ttyAMA0 ]; then
    ARGS="--https --mavlink /dev/ttyAMA0 --baud 57600"
  else
    ARGS="--https --mavlink udpin:0.0.0.0:14550"
  fi
  echo "$(date) starting mission_server $ARGS" >> ~/mission_server.log
  ~/yolo/venv/bin/python mission_server.py $ARGS >> ~/mission_server.log 2>&1
  sleep 3
done
