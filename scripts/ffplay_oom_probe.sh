#!/usr/bin/env bash
# Copyright 2026 Evan Flynn
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# ffplay_oom_probe.sh — host-side attribution matrix (no ROS / no undistort).
#
#   back camera 640x480   -> phase low+sidecars  : play with front/left/right
#   back camera 1280x1024 -> phase high-solo     : sidecars STOPPED, back only
#
# Switches every SWITCH_INTERVAL seconds. OOM in a given phase tells you
# whether the fault is high-resolution REQBUFS alone, or multi-camera load.

set -u

STOP_ON_BAD_SIZEIMAGE="${STOP_ON_BAD_SIZEIMAGE:-1}"
V4L2CTL="${V4L2CTL:-v4l2-ctl}"

# --- sizeimage sanity (embedded so each script is self-contained) --------
sizeimage_expected_bpp() {
  case "$1" in
    MJPG|JPEG) echo 3 ;;
    YUYV|UYVY|YU12|YV12) echo 2 ;;
    GREY|Y800) echo 1 ;;
    *) echo 3 ;;
  esac
}

# Echo an explanation when the driver's sizeimage is implausible; echo nothing
# when it looks healthy (or the query failed / the field is absent).
sizeimage_abnormal_reason() {
  local device="$1" output size width height fourcc bpp
  local expected bound factor cap

  output="$("${V4L2CTL:-v4l2-ctl}" -d "$device" --get-fmt-video 2>/dev/null)" || return 0
  [ -n "$output" ] || return 0

  size="$(printf '%s\n' "$output" |
    awk -F: '/Size Image/ {gsub(/[^0-9]/, "", $2); print $2; exit}')"
  [ -n "$size" ] || return 0

  width="$(printf '%s\n' "$output" |
    awk -F: '/Width\/Height/ {print $2; exit}' | tr -d ' ' | cut -d/ -f1)"
  height="$(printf '%s\n' "$output" |
    awk -F: '/Width\/Height/ {print $2; exit}' | tr -d ' ' | cut -d/ -f2)"
  fourcc="$(printf '%s\n' "$output" |
    awk -F"'" '/Pixel Format/ {print $2; exit}')"
  bpp="$(sizeimage_expected_bpp "$fourcc")"

  bound=0
  if [[ "$width" =~ ^[0-9]+$ && "$height" =~ ^[0-9]+$ ]] &&
     (( width > 0 && height > 0 )); then
    expected=$(( width * height * bpp ))
    factor="${SIZEIMAGE_FACTOR:-4}"
    cap="${SIZEIMAGE_ABS_CAP:-67108864}"
    bound=$(( expected * factor ))
    (( bound > cap )) && bound="$cap"
  fi

  if (( size > 4026531840 )); then
    echo "sizeimage ${size} looks like uint32 garbage (${width}x${height} '${fourcc}')"
  elif (( bound > 0 && size > bound )); then
    echo "sizeimage ${size} exceeds bound ${bound} (${width}x${height} '${fourcc}', factor ${factor})"
  fi
  return 0
}

usage() {
  cat <<'EOF'
Usage: ffplay_oom_probe.sh [--help]

Host-side probe against /dev/rgb_camera_* (not inside a container).

Phases (alternating every SWITCH_INTERVAL seconds):
  low+sidecars   back=640x480   WITH rgb_camera_(front|left|right) playing
  high-solo      back=1280x1024 sidecars stopped (back camera alone)

Environment:
  SWITCH_INTERVAL  seconds per phase (default: 5)
  DURATION         total runtime seconds (default: 300)
  FRAMERATE        fps (default: 10). Set empty to omit -framerate
  INPUT_FORMAT     FFmpeg input_format; empty = omit/auto (default: empty).
                   A wrong value yields "Failed to open file ... or
                   configure filtergraph". Try mjpeg/yuyv422 only if needed.
  FFPLAY_EXTRA     extra ffplay args (e.g. "-an")
  SIDECARS         sidecar names (default: "front left right")
  BACK_DEVICE      back device (default: /dev/rgb_camera_back)
  OUTDIR           logs (default: /tmp/ffplay_oom_probe_<ts>)
  FFPLAY           binary (default: ffplay)
  STOP_ON_BAD_SIZEIMAGE  default 1: after each back-camera close, query
                    `v4l2-ctl -d BACK_DEVICE --get-fmt-video`; stop at once
                    (exit 2) when Size Image is implausible, leaving the
                    driver state for the next usb_cam launch.
  V4L2CTL          v4l2-ctl binary (default: v4l2-ctl)

Example:
  ./scripts/ffplay_oom_probe.sh
  SWITCH_INTERVAL=5 DURATION=120 ./scripts/ffplay_oom_probe.sh

Interpretation:
  FAIL only in high-solo   -> high-res REQBUFS / sizeimage on back alone
  FAIL only in low+sidecars-> multi-camera USB / memory pressure
  FAIL in both             -> switching or leftover UVC state
  PASS both                -> not reproduced; tighten interval / duration
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

SWITCH_INTERVAL="${SWITCH_INTERVAL:-5}"
DURATION="${DURATION:-300}"
FRAMERATE="${FRAMERATE:-10}"
# Empty = do not pass -input_format (auto). The known-good host command
# is `ffplay -f v4l2 -video_size 640x480 /dev/rgb_camera_back` without it.
INPUT_FORMAT="${INPUT_FORMAT:-}"
SIDECARS="${SIDECARS:-front left right}"
BACK_DEVICE="${BACK_DEVICE:-/dev/rgb_camera_back}"
FFPLAY="${FFPLAY:-ffplay}"
TS="$(date +%Y%m%d-%H%M%S)"
OUTDIR="${OUTDIR:-/tmp/ffplay_oom_probe_${TS}}"

LOW_RES="${LOW_RES:-640x480}"
HIGH_RES="${HIGH_RES:-1280x1024}"

if ! command -v "$FFPLAY" >/dev/null 2>&1; then
  echo "ERROR: ffplay not found (FFPLAY=$FFPLAY)" >&2
  exit 1
fi

if [[ ! -c "$BACK_DEVICE" ]]; then
  echo "ERROR: back device $BACK_DEVICE missing or not a char device" >&2
  echo "Run this on the host where /dev/rgb_camera_* exist." >&2
  exit 1
fi
if [[ "$STOP_ON_BAD_SIZEIMAGE" -eq 1 ]] && ! command -v "$V4L2CTL" >/dev/null 2>&1; then
  echo "ERROR: $V4L2CTL not found (required when STOP_ON_BAD_SIZEIMAGE=1)" >&2
  exit 1
fi

mkdir -p "$OUTDIR"
SUMMARY="$OUTDIR/summary.txt"

pids=()

cleanup() {
  trap - EXIT INT TERM
  # Order: back first, then sidecars (same as the live switch path).
  if declare -F stop_back >/dev/null 2>&1; then
    stop_back
    stop_sidecars
  fi
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

mem_available_kb() {
  awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0
}

oom_since_lines() {
  # $1 = previous dmesg line count
  dmesg 2>/dev/null | tail -n +"$(($1 + 1))" 2>/dev/null | grep -E \
    'Out of memory|oom-kill|Killed process|uvcvideo|Failed to resubmit|UVC non compliance|vb2' \
    || true
}

start_ffplay() {
  # $1=device $2=W $3=H $4=log  -> echoes pid
  #
  # Match the known-good host form:
  #   ffplay -f v4l2 -video_size WxH /dev/rgb_camera_back
  # Do NOT pass -nodisp/-autoexit here: on a headless/no-GL setup those
  # make ffplay fail with "configure filtergraph" even when the device
  # opens fine. INPUT_FORMAT empty means auto (omit -input_format).
  local dev="$1" w="$2" h="$3" log="$4"
  local -a args=(
    -loglevel warning
    -f v4l2
    -video_size "${w}x${h}"
  )
  if [[ -n "$INPUT_FORMAT" ]]; then
    args+=(-input_format "$INPUT_FORMAT")
  fi
  if [[ -n "$FRAMERATE" ]]; then
    args+=(-framerate "$FRAMERATE")
  fi
  args+=(-i "$dev")
  # Extra args for experiments, e.g. FFPLAY_EXTRA="-an"
  if [[ -n "${FFPLAY_EXTRA:-}" ]]; then
    # shellcheck disable=SC2206
    args+=($FFPLAY_EXTRA)
  fi
  "$FFPLAY" "${args[@]}" >"$log" 2>&1 &
  echo $!
}

# --- camera process control -------------------------------------------------
# Order matters (operator requirement):
#   start:  sidecars (front/left/right) FIRST, then back
#   stop:   back FIRST, then sidecars

sidecar_pids=()
back_pid=""

stop_pid() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

drop_from_pids() {
  local dead="$1" pid keep=()
  for pid in "${pids[@]:-}"; do
    [[ "$pid" == "$dead" ]] || keep+=("$pid")
  done
  pids=("${keep[@]:-}")
}

stop_back() {
  if [[ -n "$back_pid" ]]; then
    stop_pid "$back_pid"
    drop_from_pids "$back_pid"
    echo "[back] $(date +%T) stopped pid=$back_pid" >>"$OUTDIR/cameras.log"
    back_pid=""
  fi
}

stop_sidecars() {
  local pid
  for pid in "${sidecar_pids[@]:-}"; do
    stop_pid "$pid"
    drop_from_pids "$pid"
  done
  if [[ ${#sidecar_pids[@]} -gt 0 ]]; then
    echo "[sidecars] $(date +%T) stopped" >>"$OUTDIR/cameras.log"
  fi
  sidecar_pids=()
}

start_sidecars() {
  # Idempotent: never stack a second set on top of a live one.
  if [[ ${#sidecar_pids[@]} -gt 0 ]]; then
    return 0
  fi
  local name dev log pid w h
  w="${LOW_RES%x*}"
  h="${LOW_RES#*x}"
  for name in $SIDECARS; do
    dev="/dev/rgb_camera_${name}"
    if [[ ! -c "$dev" ]]; then
      echo "WARN: sidecar $dev missing — skip" >>"$OUTDIR/cameras.log"
      continue
    fi
    log="$OUTDIR/sidecar_${name}.log"
    pid="$(start_ffplay "$dev" "$w" "$h" "$log")"
    sidecar_pids+=("$pid")
    pids+=("$pid")
    echo "[sidecars] $(date +%T) started $name pid=$pid ${w}x${h}" >>"$OUTDIR/cameras.log"
  done
}

start_back() {
  # $1=W $2=H $3=log
  stop_back
  back_pid="$(start_ffplay "$BACK_DEVICE" "$1" "$2" "$3")"
  pids+=("$back_pid")
  echo "[back] $(date +%T) started pid=$back_pid ${1}x${2}" >>"$OUTDIR/cameras.log"
}

maybe_stop_on_bad_sizeimage() {
  # $1 = context label; aborts with exit 2 when the driver reports garbage.
  [[ "$STOP_ON_BAD_SIZEIMAGE" -eq 1 ]] || return 0
  local reason
  reason="$(sizeimage_abnormal_reason "$BACK_DEVICE")"
  [[ -n "$reason" ]] || return 0
  {
    echo "  ABNORMAL SIZEIMAGE ($1): $reason"
    echo "  capture stopped on purpose; the driver state is left untouched."
    echo "  Start the real stack now to reproduce, e.g.:"
    echo "    ros2 launch usb_cam rgb_camera_back_640x480.launch.py"
  } | tee -a "$SUMMARY"
  exit 2
}

# --- banner -----------------------------------------------------------------

{
  echo "ffplay OOM attribution probe ($(date))"
  echo "  back=$BACK_DEVICE interval=${SWITCH_INTERVAL}s duration=${DURATION}s"
  echo "  low+sidecars : $LOW_RES with {$SIDECARS}"
  echo "  high-solo    : $HIGH_RES alone (sidecars stopped)"
  echo "  format=${INPUT_FORMAT:-auto} fps=${FRAMERATE:-auto} logs=$OUTDIR"
} | tee "$SUMMARY"

# --- phase loop -------------------------------------------------------------

declare -A PHASE_FAILS
declare -A PHASE_RUNS
PHASE_FAILS[low+sidecars]=0
PHASE_FAILS[high-solo]=0
PHASE_RUNS[low+sidecars]=0
PHASE_RUNS[high-solo]=0

DEADLINE=$(( $(date +%s) + DURATION ))
STEP=0
TOTAL_FAILS=0

# Phase order: start low+sidecars so we learn multi-camera first.
phase="low+sidecars"

while [[ $(date +%s) -lt $DEADLINE ]]; do
  STEP=$((STEP + 1))

  # Lifecycle order (operator requirement):
  #   into low+sidecars : sidecars first, then back @ low
  #   into high-solo    : stop back first, then sidecars, then back @ high
  if [[ "$phase" == "low+sidecars" ]]; then
    res="$LOW_RES"
    stop_back
    start_sidecars
  else
    res="$HIGH_RES"
    stop_back
    stop_sidecars
  fi

  maybe_stop_on_bad_sizeimage "before step ${STEP} phase=${phase}"

  W="${res%x*}"
  H="${res#*x}"
  LOG="$OUTDIR/back_${STEP}_${phase}_${W}x${H}.log"
  MEM_BEFORE="$(mem_available_kb)"
  MSG_BEFORE=$(dmesg 2>/dev/null | wc -l)

  echo "[step $STEP] $(date +%T) phase=$phase back ${W}x${H}" | tee -a "$SUMMARY"

  start_back "$W" "$H" "$LOG"
  FPID="$back_pid"

  # Always hold the phase for the full SWITCH_INTERVAL. An early ffplay
  # exit must NOT shorten the step (that made switches look <2s).
  PHASE_END=$(( $(date +%s) + SWITCH_INTERVAL ))
  DIED_EARLY=0
  while [[ $(date +%s) -lt $PHASE_END ]]; do
    if [[ $DIED_EARLY -eq 0 ]] && ! kill -0 "$FPID" 2>/dev/null; then
      DIED_EARLY=1
    fi
    # Sleep the remaining time (capped at 1s) so we overshoot less.
    now=$(date +%s)
    remain=$(( PHASE_END - now ))
    if [[ $remain -le 0 ]]; then
      break
    fi
    if [[ $remain -gt 1 ]]; then
      sleep 1
    else
      sleep "$remain"
    fi
  done

  RC=0
  if kill -0 "$FPID" 2>/dev/null; then
    kill "$FPID" 2>/dev/null || true
    wait "$FPID" 2>/dev/null || true
    # our kill after a full interval is expected
  else
    wait "$FPID" 2>/dev/null
    RC=$?
  fi
  drop_from_pids "$FPID"
  back_pid=""

  MEM_AFTER="$(mem_available_kb)"
  MARKERS="$(oom_since_lines "$MSG_BEFORE")"

  STATUS="PASS"
  if [[ $RC -ne 0 && $RC -ne 137 && $RC -ne 143 ]]; then
    STATUS="FAIL"
  fi
  if [[ $DIED_EARLY -eq 1 ]]; then
    # ffplay left before the phase finished — that is a failure even on rc=0.
    STATUS="FAIL"
  fi
  if [[ -n "$MARKERS" ]]; then
    STATUS="FAIL"
  fi

  PHASE_RUNS[$phase]=$(( ${PHASE_RUNS[$phase]} + 1 ))
  if [[ "$STATUS" == "FAIL" ]]; then
    PHASE_FAILS[$phase]=$(( ${PHASE_FAILS[$phase]} + 1 ))
    TOTAL_FAILS=$((TOTAL_FAILS + 1))
  fi

  {
    echo "  status=$STATUS rc=$RC MemAvailable ${MEM_BEFORE} -> ${MEM_AFTER} kB"
    if [[ -n "$MARKERS" ]]; then
      echo "  dmesg:"
      printf '%s\n' "$MARKERS" | sed 's/^/    /'
    fi
    if [[ "$STATUS" == "FAIL" && -s "$LOG" ]]; then
      echo "  ffplay stderr (tail):"
      tail -n 12 "$LOG" | sed 's/^/    /'
    fi
  } | tee -a "$SUMMARY"

  # alternate phase
  if [[ "$phase" == "low+sidecars" ]]; then
    phase="high-solo"
  else
    phase="low+sidecars"
  fi
done

# Teardown order: back first, then sidecars.
stop_back
stop_sidecars
maybe_stop_on_bad_sizeimage "after teardown"

echo | tee -a "$SUMMARY"
echo "Phase summary:" | tee -a "$SUMMARY"
for p in "low+sidecars" "high-solo"; do
  echo "  $p : ${PHASE_FAILS[$p]} FAIL / ${PHASE_RUNS[$p]} runs" | tee -a "$SUMMARY"
done

echo | tee -a "$SUMMARY"
if [[ $TOTAL_FAILS -eq 0 ]]; then
  echo "RESULT: PASS ($STEP steps, no OOM markers)" | tee -a "$SUMMARY"
  echo "Not reproduced — try longer DURATION or SWITCH_INTERVAL=2." | tee -a "$SUMMARY"
  exit 0
fi

echo "RESULT: FAIL ($TOTAL_FAILS steps) — see $OUTDIR" | tee -a "$SUMMARY"
if [[ ${PHASE_FAILS[high-solo]} -gt 0 && ${PHASE_FAILS[low+sidecars]} -eq 0 ]]; then
  echo "Hint: only high-solo failed -> high-res REQBUFS/sizeimage on back alone." | tee -a "$SUMMARY"
elif [[ ${PHASE_FAILS[low+sidecars]} -gt 0 && ${PHASE_FAILS[high-solo]} -eq 0 ]]; then
  echo "Hint: only low+sidecars failed -> multi-camera USB/memory pressure." | tee -a "$SUMMARY"
else
  echo "Hint: both phases failed -> resolution switch / leftover UVC state." | tee -a "$SUMMARY"
fi
exit 1
