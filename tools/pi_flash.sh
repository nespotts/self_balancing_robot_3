#!/usr/bin/env bash
# Flash the Teensy 4.1 that is plugged into the Orange Pi (issue #2).
#
#   pi_flash.sh [--dry-run] firmware.hex
#
# Run on the Pi. tools/wifi_upload.py copies this script and the .hex to ~/sbr-tools/ and runs it.
#   1. checks the hex, the loader and that a Teensy is on the Pi's USB
#   2. aborts if VirtualHere or evesec (Serial to Ethernet) is using the Teensy
#   3. asks the ground-station bridge to let go of the serial port (release file) and waits
#   4. flashes with teensy_loader_cli (soft reboot, no button press)      <- skipped by --dry-run
#   5. waits for the Teensy's serial port to come back, then lets the bridge reconnect
# Exit code 0 = flashed (or dry run passed), anything else = failed. No sudo needed.
#
# Env: SBR3_RELEASE_FILE (default /tmp/sbr3-bridge.release, same as station/bridge)
#      SBR3_RELEASE_TIMEOUT seconds to wait for the port to be free (default 10)
#      SBR3_FLASH_TIMEOUT   seconds for teensy_loader_cli (default 60)
#      TEENSY_LOADER        loader path (default: PlatformIO's tool-teensy, then $PATH)
set -u

DRY=0
HEX=""
for a in "$@"; do
  case "$a" in
    --dry-run|-n) DRY=1 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    -*) echo "pi_flash: unknown option $a" >&2; exit 2 ;;
    *) HEX="$a" ;;
  esac
done

REL=${SBR3_RELEASE_FILE:-/tmp/sbr3-bridge.release}
REL_TIMEOUT=${SBR3_RELEASE_TIMEOUT:-10}
FLASH_TIMEOUT=${SBR3_FLASH_TIMEOUT:-60}
VH_CONFIG=/usr/local/etc/virtualhere/config.ini

say()  { echo "[pi_flash] $*"; }
ok()   { echo "[pi_flash]   ok   $*"; }
warn() { echo "[pi_flash]   WARN $*"; }
fail() { echo "[pi_flash] FAILED: $*" >&2; exit 1; }

[ "$DRY" = 1 ] && say "DRY RUN - everything except the flash itself"

# ---------------------------------------------------------------- 1. hex + loader
[ -n "$HEX" ] || fail "usage: pi_flash.sh [--dry-run] firmware.hex"
[ -s "$HEX" ] || fail "hex file not found or empty: $HEX"
head -c 1 "$HEX" | grep -q ':' || fail "$HEX is not an Intel hex file"
tail -n 2 "$HEX" | tr -d '\r' | grep -q '^:00000001FF' || fail "$HEX has no end record (incomplete copy?)"
ok "hex $HEX ($(wc -c < "$HEX") bytes, $(wc -l < "$HEX") lines)"

LOADER=${TEENSY_LOADER:-}
if [ -z "$LOADER" ]; then
  if [ -x "$HOME/.platformio/packages/tool-teensy/teensy_loader_cli" ]; then
    LOADER="$HOME/.platformio/packages/tool-teensy/teensy_loader_cli"
  else
    LOADER=$(command -v teensy_loader_cli || true)
  fi
fi
[ -n "$LOADER" ] && [ -x "$LOADER" ] || fail "teensy_loader_cli not found (install PlatformIO tool-teensy or set TEENSY_LOADER)"
"$LOADER" --list-mcus 2>&1 | grep -q TEENSY41 || fail "$LOADER does not run or does not know TEENSY41"
ok "loader $LOADER"

# ---------------------------------------------------------------- 2. find the Teensy
# Prints "<sysfs dir> <pid>" for every PJRC (16c0) device whose product id is 04xx.
teensy_devices() {
  local d
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat "$d/idVendor")" = 16c0 ] || continue
    case "$(cat "$d/idProduct")" in 04??) echo "$d $(cat "$d/idProduct")" ;; esac
  done
}
# tty of the first running Teensy (e.g. /dev/ttyACM0), empty if none.
teensy_tty() {
  local d pid t
  while read -r d pid; do
    [ "$pid" = 0478 ] && continue
    for t in "$d"/*:*/tty/tty*; do [ -e "$t" ] && { echo "/dev/$(basename "$t")"; return; }; done
  done < <(teensy_devices)
}

DEVS=$(teensy_devices)
[ -n "$DEVS" ] || fail "no Teensy on the Pi's USB (16c0:04xx). Check the cable / power."
MODE=""
while read -r d pid; do
  if [ "$pid" = 0478 ]; then MODE=bootloader; ok "Teensy in bootloader (16c0:0478) at $(basename "$d")"
  else [ -z "$MODE" ] && MODE=running; ok "Teensy running (16c0:$pid) at $(basename "$d")"; fi
  # VirtualHere (or any user-space program) claims the interfaces -> driver "usbfs"
  for i in "$d"/*:*; do
    [ -e "$i/driver" ] || continue
    if [ "$(basename "$(readlink "$i/driver")")" = usbfs ]; then
      if pgrep -x vhusbdarm64 >/dev/null || pgrep -f vhusbd >/dev/null; then
        fail "VirtualHere is sharing the Teensy with a client right now ($(basename "$i") claimed).
  Stop using it in the VirtualHere client (or stop virtualhere.service), then try again."
      fi
      fail "a user-space program has claimed the Teensy's USB interface $(basename "$i") (usbfs). Close it and try again."
    fi
  done
done <<< "$DEVS"

PORT=$(teensy_tty)
if [ "$MODE" = running ]; then
  [ -n "$PORT" ] || fail "Teensy is running but has no serial port (not a USB-serial build?). Press its button to flash by hand."
  ok "serial port $PORT"
fi

if pgrep -f vhusbd >/dev/null; then
  if [ -r "$VH_CONFIG" ] && grep -qi '^IgnoredDevices=.*16c0/0478' "$VH_CONFIG"; then
    ok "VirtualHere runs but ignores the Teensy bootloader"
  else
    warn "VirtualHere runs and does not ignore the Teensy (no IgnoredDevices=16c0/0483,16c0/0478 in $VH_CONFIG)."
    warn "     A VirtualHere client set to auto-use could grab it mid-flash. No client is using it now."
  fi
fi

# evesec (Serial to Ethernet Connector) keeps the tty open when its connection is started.
if [ -n "$PORT" ] && command -v evesecli >/dev/null; then
  EV=$(evesecli list 2>/dev/null | grep -F " $PORT " || true)
  if [ -n "$EV" ] && ! echo "$EV" | grep -q 'stopped'; then
    fail "evesec (Serial to Ethernet Connector) is using $PORT:
  $EV
  Stop that connection (evesecli / its web UI), then try again."
  fi
  ok "evesec not using $PORT"
fi

# ---------------------------------------------------------------- one flash at a time
exec 9>/tmp/sbr3-flash.lock
command -v flock >/dev/null && { flock -n 9 || fail "another pi_flash.sh is running"; }

# ---------------------------------------------------------------- 3. free the serial port
holders() {   # "pid command" of every process we can see with the port open
  [ -n "$PORT" ] || return 0
  find /proc/[0-9]*/fd -lname "$PORT" 2>/dev/null | cut -d/ -f3 | sort -u | while read -r p; do
    echo "$p $(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-80)"
  done
}
bridge_running() { pgrep -f 'station[./]bridge' >/dev/null; }

CREATED_REL=0
cleanup() {
  if [ "$CREATED_REL" = 1 ]; then rm -f "$REL"; say "release file removed - the bridge reconnects by itself"; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

if [ -n "$PORT" ]; then
  if [ -e "$REL" ]; then
    ok "release file $REL already there (left in place)"
  else
    touch "$REL" || fail "cannot create $REL"
    CREATED_REL=1
    ok "release file $REL created"
  fi
  if bridge_running; then
    say "waiting for the bridge to let go of $PORT (max ${REL_TIMEOUT}s)"
    end=$((SECONDS + REL_TIMEOUT))
    while [ -n "$(holders)" ] && [ $SECONDS -lt $end ]; do sleep 0.2; done
  else
    ok "no bridge running"
  fi
  H=$(holders)
  [ -z "$H" ] || fail "$PORT is still open by:
$(echo "$H" | sed 's/^/    /')
  Close those programs and try again."
  ok "$PORT is free"
fi

# ---------------------------------------------------------------- 4. flash
CMD=("$LOADER" --mcu=TEENSY41 -w -s -v "$HEX")
if [ "$DRY" = 1 ]; then
  say "would run: timeout $FLASH_TIMEOUT ${CMD[*]}"
  say "DRY RUN OK - nothing was flashed"
  exit 0
fi

say "flashing (soft reboot, no button press needed)..."
T0=$SECONDS
timeout "$FLASH_TIMEOUT" "${CMD[@]}"
RC=$?
[ $RC = 124 ] && fail "teensy_loader_cli timed out after ${FLASH_TIMEOUT}s (Teensy did not enter the bootloader? press its button)"
[ $RC = 0 ] || fail "teensy_loader_cli exit code $RC"
ok "flashed in $((SECONDS - T0))s"

# ---------------------------------------------------------------- 5. wait for it to come back
say "waiting for the Teensy to boot (max 15s)"
end=$((SECONDS + 15))
NEW=""
while [ $SECONDS -lt $end ]; do NEW=$(teensy_tty); [ -n "$NEW" ] && break; sleep 0.3; done
if [ -z "$NEW" ]; then
  warn "no Teensy serial port after 15s - new firmware may not have USB serial, or it crashed at boot"
  exit 3
fi
ok "Teensy back on $NEW"
[ -n "$PORT" ] && [ "$NEW" != "$PORT" ] && warn "port changed $PORT -> $NEW (the bridge expects $PORT)"
say "DONE"
exit 0
