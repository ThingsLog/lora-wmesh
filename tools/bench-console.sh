#!/bin/sh
# W-Mesh bench console — one GNU screen session, one window per connected
# node (every /dev/cu.usbmodem*), all visible at once in stacked regions.
#
#   tools/bench-console.sh            # open the session
#   tools/bench-console.sh --print    # just show the generated screenrc
#
# Inside the session:
#   Ctrl-A Tab     jump to the next region (board)
#   Ctrl-A H       toggle logging of the focused window to screenlog.N
#   Ctrl-A K y     kill the focused window   |   Ctrl-A \  quit everything
#   Ctrl-A [       scrollback (q to leave)
#
# Flashing needs the port free: kill that window (or the whole session) first.
# SPDX-License-Identifier: MIT

PORTS=$(ls /dev/cu.usbmodem* 2>/dev/null)
if [ -z "$PORTS" ]; then
  echo "No /dev/cu.usbmodem* ports — connect the nodes (and check they are"
  echo "running the app, not sitting dark; see firmware/xiao-nrf52840/README)."
  exit 1
fi

RC=$(mktemp /tmp/wmesh-bench.XXXXXX)
{
  echo "startup_message off"
  echo "defscrollback 10000"
  echo 'hardstatus alwayslastline "%{= kw}W-Mesh bench  %-w%{= BW}%n %t%{-}%+w  %=%c"'
  i=0
  for p in $PORTS; do
    echo "screen -t ${p##*/} $i $p 115200"
    i=$((i+1))
  done
  # stack the regions and put one window in each
  echo "select 0"
  i=1
  for p in $PORTS; do
    [ "$i" -eq 1 ] && { i=2; continue; }   # first window is already placed
    echo "split"
    echo "focus down"
    echo "select $((i-1))"
    i=$((i+1))
  done
  echo "focus top"
} > "$RC"

if [ "$1" = "--print" ]; then
  cat "$RC"
  rm -f "$RC"
  exit 0
fi

exec screen -c "$RC"
