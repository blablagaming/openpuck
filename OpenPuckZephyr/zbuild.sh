#!/bin/bash
# Convenience build wrapper for the OpenPuck Zephyr port.
# Sets up the workspace env (venv, ZEPHYR_BASE, SDK) and runs west build.
# Usage: ./zbuild.sh [extra west build args]   (run from anywhere)
set -euo pipefail
export PIP_USER=false
export ZEPHYR_BASE="$HOME/zephyrproject/zephyr"
export ZEPHYR_SDK_INSTALL_DIR="$HOME/zephyr-sdk-1.0.1"
# shellcheck disable=SC1091
source "$HOME/openpuck-zephyr-venv/bin/activate"

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The uf2 variant carries the Adafruit UF2 bootloader partition layout (app at
# 0x26000, a 32 kB storage partition for littlefs, UF2 output) + CDC-ACM console
# + LED0 on P1.15 — matching the real puck hardware and the user's DFU workflow.
BOARD="${BOARD:-adafruit_feather_nrf52840/nrf52840/uf2}"

cd "$APP_DIR"
exec west build -b "$BOARD" "$APP_DIR" "$@"
