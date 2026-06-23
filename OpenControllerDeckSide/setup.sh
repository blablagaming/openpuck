#!/usr/bin/env bash
# Per-boot device-permission grant for SteamOS (read-only rootfs -> no persistent udev rule).
# Run ONCE PER BOOT from Desktop Mode:   sudo ./setup.sh
#
# /dev/input/event* is already granted to the active session user (logind ACL) -- that's how the evdev
# grab works without this. But /dev/ttyACM* (the nRF CDC link) and /dev/hidraw* are not, so this opens
# them. It does NOT persist across reboots (nothing on the read-only FS changes); re-run after a reboot.
set -e
shopt -s nullglob
for f in /dev/ttyACM*; do chmod a+rw "$f" && echo "opened $f"; done
for f in /dev/hidraw*; do chmod a+rw "$f"; done   # only needed for --hidraw gyro/trackpads
echo "done. /dev/input is already session-ACL'd; ttyACM*/hidraw* opened for this boot."
