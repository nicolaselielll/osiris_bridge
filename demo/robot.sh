#!/bin/bash
# Turns Osiris Cloud's demo robot on or off. While it's off, only its disk is
# billed.
#   ./robot.sh on | off | status
set -e
APP=osiris-demo-robot
machine=$(fly machine list -a "$APP" --quiet | awk 'NF {print $1; exit}')
case "${1:-status}" in
  on) fly machine start "$machine" -a "$APP" && echo "Starting: it shows up in the demo workspace in about 2 minutes." ;;
  off) fly machine stop "$machine" -a "$APP" ;;
  status) fly machine list -a "$APP" ;;
  *) echo "usage: $0 on|off|status"; exit 1 ;;
esac
