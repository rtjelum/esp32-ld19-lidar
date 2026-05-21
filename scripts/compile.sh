#!/bin/bash
# Wrap `arduino-cli compile` with a progress bar.
#
# Runs compile in the background with verbose output piped to a log file,
# polls the log for phase markers, and interpolates the bar by elapsed time
# within the current phase. Each phase has a (base_pct, next_pct, expected_dur);
# `displayed` grows linearly from base toward next over expected_dur, capped at
# next-1 so the bar can't claim to be in a phase that hasn't been detected.

set -o pipefail

BUILD_DIR="${BUILD_DIR:-build}"
LOG="$BUILD_DIR/compile.log"
mkdir -p "$BUILD_DIR"
: > "$LOG"

start=$(date +%s)
WIDTH=30

phase_pct=0       # base percentage of the current phase
phase_next=5      # ceiling percentage (next phase's base)
phase_dur=3       # expected seconds for current phase
phase_start=$start
displayed=0
step="Starting"

set_phase() {
  local new_pct=$1 new_next=$2 new_dur=$3 new_step=$4
  if [ "$new_pct" -gt "$phase_pct" ]; then
    phase_pct=$new_pct
    phase_next=$new_next
    phase_dur=$new_dur
    phase_start=$(date +%s)
    [ "$displayed" -lt "$new_pct" ] && displayed=$new_pct
    step=$new_step
  fi
}

draw() {
  local now elapsed_total in_phase target filled bar=""
  now=$(date +%s)
  elapsed_total=$(( now - start ))
  in_phase=$(( now - phase_start ))

  # Linear interpolation from phase_pct toward (phase_next - 1) over phase_dur.
  target=$(( phase_pct + in_phase * (phase_next - phase_pct - 1) / phase_dur ))
  [ "$target" -gt "$displayed" ] && displayed=$target
  [ "$displayed" -ge "$phase_next" ] && displayed=$(( phase_next - 1 ))

  filled=$(( displayed * WIDTH / 100 ))
  [ "$filled" -gt 0 ] && bar=$(printf '%*s' "$filled" '' | tr ' ' '#')
  [ "$filled" -lt "$WIDTH" ] && bar+=$(printf '%*s' $((WIDTH - filled)) '' | tr ' ' '-')
  printf '\r\033[2K[%3ds] [%s] %3d%%  %s' "$elapsed_total" "$bar" "$displayed" "$step"
}

arduino-cli compile "$@" --verbose >"$LOG" 2>&1 &
PID=$!
trap 'kill $PID 2>/dev/null; printf "\n"; exit 130' INT TERM

draw
while kill -0 $PID 2>/dev/null; do
  # Highest matching phase wins. Args: base, next, expected_seconds, label.
  if   grep -q 'Sketch uses'           "$LOG"; then set_phase 100 101 1  "Finalizing"
  elif grep -q 'merge-bin'             "$LOG"; then set_phase 95  100 3  "Merging image"
  elif grep -q 'elf2image'             "$LOG"; then set_phase 65  99  25 "Generating image"
  elif grep -q 'Linking everything'    "$LOG"; then set_phase 55  70  5  "Linking"
  elif grep -q 'Compiling core'        "$LOG"; then set_phase 35  60  20 "Compiling core"
  elif grep -q 'Compiling libraries'   "$LOG"; then set_phase 25  45  10 "Compiling libraries"
  elif grep -q 'Compiling sketch'      "$LOG"; then set_phase 15  30  5  "Compiling sketch"
  elif grep -q 'Generating function'   "$LOG"; then set_phase 10  20  3  "Generating prototypes"
  elif grep -q 'Detecting libraries'   "$LOG"; then set_phase 5   15  3  "Detecting libraries"
  fi
  draw
  sleep 0.3
done

wait $PID
RC=$?

if [ $RC -eq 0 ]; then
  displayed=100; phase_pct=100; step="Done"
else
  step="Failed (log: $LOG)"
fi
draw
printf '\n'

if [ $RC -eq 0 ]; then
  grep -E 'Sketch uses|Global variables' "$LOG"
else
  tail -20 "$LOG"
fi

exit $RC
