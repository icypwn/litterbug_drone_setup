#!/bin/bash
# Started at boot (crontab @reboot): records the camera from power-on until the power is cut. No network,
# site or YOLO needed; process the video afterwards.
#  - Full field of view (2304x1296 sensor mode) scaled to 1920x1080, 30 fps, a keyframe every second.
#  - MPEG-TS: a file cut off by pulling the power still plays up to that point.
#  - Everything is flushed to the SD card every 3 s, so at most a few seconds are lost.
#  - Files: ~/recordings/rec_<number>_<date>.ts (the number is there because without internet the clock can be wrong).
# Stop it for good: crontab -e and delete the record_on_boot line.
OUT=/home/litterbug/recordings
LOG="$OUT/recorder.log"
mkdir -p "$OUT"
sleep 5   # let the camera come up

( while true; do sleep 3; sync; done ) &

while true; do
  FREE_KB=$(df --output=avail -k "$OUT" | tail -1)
  if [ "$FREE_KB" -lt 2000000 ]; then
    echo "$(date) less than 2 GB free: not recording" >> "$LOG"; sleep 60; continue
  fi
  N=$(ls "$OUT" | grep -c '^rec_')
  F="$OUT/rec_$(printf %03d $((N + 1)))_$(date +%Y%m%d_%H%M%S).ts"
  echo "$(date) recording $F" >> "$LOG"
  rpicam-vid -t 0 -n --mode 2304:1296:10:P --width 1920 --height 1080 --framerate 30 \
    --codec libav --libav-format mpegts --libav-video-codec-opts "preset=ultrafast;tune=zerolatency" \
    --bitrate 10000000 --intra 30 -o "$F" >> "$LOG" 2>&1
  echo "$(date) camera stopped (code $?), starting a new file" >> "$LOG"
  sleep 2
done
