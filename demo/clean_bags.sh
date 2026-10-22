#!/bin/bash
# Keeps recordings from filling the disk. Anyone in the demo workspace can
# start one, and recording every topic of the simulation, camera included,
# writes tens of megabytes a second. A recording is stopped after 5 minutes or
# 1 GB, a finished one is deleted after an hour, and recordings are stopped
# and the oldest deleted while less than 2 GB is free.
BAGS=/root/ros2_bags
free_mb() { df -Pm / | awk 'NR == 2 {print $4}'; }
# SIGINT lets ros2 bag write its index before it exits; SIGTERM if it doesn't
# within 15 s.
stop_recording() {
  kill -INT "$1" 2>/dev/null
  for _ in $(seq 15); do kill -0 "$1" 2>/dev/null || return; sleep 1; done
  kill -TERM "$1" 2>/dev/null
}
stop_recordings() { for pid in $(pgrep -f "ros2 bag record"); do stop_recording "$pid"; done; }

while sleep 15; do
  for pid in $(pgrep -f "ros2 bag record"); do
    [ "$(ps -o etimes= -p "$pid" | tr -d ' ')" -gt 300 ] 2>/dev/null && stop_recording "$pid"
  done
  if [ -d "$BAGS" ]; then
    newest=$(ls -1td "$BAGS"/*/ 2>/dev/null | head -1)
    [ -n "$newest" ] && [ "$(du -sm "$newest" | cut -f1)" -gt 1024 ] && stop_recordings
    find "$BAGS" -mindepth 1 -maxdepth 1 -mmin +60 -exec rm -rf {} +
    while [ "$(free_mb)" -lt 2048 ] && [ -n "$(ls -A "$BAGS")" ]; do
      stop_recordings
      sleep 2
      rm -rf "$(ls -1td "$BAGS"/*/ | tail -1)"
    done
  fi
done
