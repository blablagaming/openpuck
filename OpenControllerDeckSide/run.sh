#!/usr/bin/env bash
# OpenController Steam Deck forwarder launcher (uv-based; nothing installed system-wide).
#
# Add as a NON-STEAM GAME:
#   Target:      /home/deck/…/deck/run.sh
#   Start In:    /home/deck/…/deck/         (the directory this script is in)
# Then, in that shortcut's gear -> Properties -> Controller, set "Disable Steam Input" so Steam doesn't
# fight us for the built-in pad while we grab it.
#
# Dependencies are pulled by uv on first run (pyserial + pygame -- both prebuilt wheels, no compiler).
# uv also provisions its own Python, so nothing touches the read-only SteamOS rootfs.
#
# If you get a serial permission error on /dev/ttyACM*, run `sudo ./setup.sh` once per boot (see README).
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

UV="$(command -v uv || true)"
[ -n "$UV" ] || UV="$HOME/.local/bin/uv"
[ -x "$UV" ] || { echo "uv not found. Install it: https://docs.astral.sh/uv/  (curl -LsSf https://astral.sh/uv/install.sh | sh)"; exit 1; }

exec "$UV" run --with pyserial --with pygame python openctrl.py "$@"
