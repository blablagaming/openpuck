#!/usr/bin/env python3
"""openctrl.py -- Steam Deck forwarder for the OpenController emulated Steam Controller 2.

Runs on the Steam Deck (Desktop Mode). Opens the nRF (Valve 28DE:1302) CDC port, reads its STATUS
stream (which bonded pucks are live), and shows a fullscreen touchscreen UI of tappable puck tiles.
Tap an *available* puck -> the Deck's controls are grabbed (detached) and streamed to the nRF, which
relays them over RF so the puck presents a Steam Controller 2 to its host. Tap again, or lose the RF
link, and control returns to the Deck.

    python3 openctrl.py                 # fullscreen touchscreen UI
    python3 openctrl.py --debug         # headless: print status, no UI (for bring-up)
    python3 openctrl.py --port /dev/ttyACM0   # override CDC port autodetect
    python3 openctrl.py --hidraw        # also read gyro/trackpads from the Deck hidraw (see input_source)

Deps: pyserial (required), evdev (for input), pygame (for the UI). See README.md.
"""
import argparse
import sys
import time

import frame
import input_source as isrc

VID = 0x28DE
PID = 0x1302  # the OpenController emulated controller
INPUT_HZ = 250.0


def find_port():
    try:
        from serial.tools import list_ports
    except Exception:
        return None
    for p in list_ports.comports():
        if (p.vid, p.pid) == (VID, PID):
            return p.device
    # fall back: any Valve CDC
    for p in list_ports.comports():
        if p.vid == VID:
            return p.device
    return None


class App:
    def __init__(self, args):
        self.args = args
        import serial  # imported here so --help works without pyserial
        port = args.port or find_port()
        if not port:
            raise SystemExit("OpenController CDC port not found (Valve 28DE:1302). "
                             "Plug in the nRF, or pass --port.")
        self.port_name = port
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.reader = frame.Reader()

        self.input = None
        self.hidraw = None
        self.forwarding = False
        self.fwd_slot = None
        self.last_input = 0.0

        # model surfaced to the UI
        self.status = {"link_up": False, "link_slot": None, "sess_ch": 0,
                       "bonds": [], "forwarding": False}
        self.note = "connected to %s" % port
        self.log = []  # firmware '#' diagnostic lines (the in-app serial monitor)

    # ---- serial I/O ----
    def pump_serial(self):
        try:
            data = self.ser.read(512)
        except Exception:
            data = b""
        for typ, payload in self.reader.feed(data):
            if typ == frame.T_STATUS:
                st = frame.parse_status(payload)
                if st:
                    self.status = st
            elif typ == frame.T_TEXT:
                line = payload.decode("latin1", "replace").rstrip()
                if line:
                    self.note = line
                    self.log.append(line)
                    self.log = self.log[-300:]
                    if self.args.debug:
                        print(line)  # stream the firmware log like a serial monitor

    def send(self, b):
        try:
            self.ser.write(b)
        except Exception:
            pass

    # ---- forwarding control ----
    def _ensure_input(self):
        if self.input is None:
            self.input = isrc.EvdevSource(path=self.args.input)
            if self.args.hidraw:
                try:
                    self.hidraw = isrc.HidrawImu()
                except Exception:
                    self.hidraw = None
        return self.input

    def start_forwarding(self, slot):
        try:
            self._ensure_input().grab()
        except Exception as ex:
            self.note = "grab failed: %s" % ex
            return
        self.forwarding = True
        self.fwd_slot = slot
        self.send(frame.build_set_forwarding(True))
        self.note = "forwarding -> slot %s (%s)" % (slot, self.input.name)

    def stop_forwarding(self):
        self.forwarding = False
        self.fwd_slot = None
        self.send(frame.build_set_forwarding(False))
        if self.input:
            self.input.ungrab()
        self.note = "control returned to Deck"

    def toggle(self, slot):
        """Tap handler: a tappable tile is one whose puck is live (alive)."""
        if self.forwarding:
            self.stop_forwarding()
        else:
            self.start_forwarding(slot)

    def pump_input(self):
        if not self.forwarding or not self.input:
            return
        st = self.input.pump()
        if self.hidraw:
            self.hidraw.read_into(st)
        now = time.monotonic()
        if now - self.last_input >= 1.0 / INPUT_HZ:
            self.last_input = now
            self.send(frame.build_input(st))

    def auto_release_on_drop(self):
        """If the RF link drops while forwarding, hand control back to the Deck."""
        if self.forwarding and not self.status.get("link_up"):
            # only release if the slot we're forwarding to is no longer alive
            slot = self.fwd_slot
            bonds = self.status.get("bonds", [])
            alive = slot is not None and slot < len(bonds) and bonds[slot]["alive"]
            if not alive:
                self.stop_forwarding()

    # ---- loops ----
    def run_debug(self):
        last = 0
        while True:
            self.pump_serial()
            self.pump_input()
            self.auto_release_on_drop()
            if time.monotonic() - last > 0.5:
                last = time.monotonic()
                s = self.status
                tiles = ", ".join(
                    "[%d %s%s]" % (i, b["serial"] or "?", " LIVE" if b["alive"] else "")
                    for i, b in enumerate(s["bonds"]) if b["used"]) or "(no bonds)"
                print("link=%s fwd=%s ch=%s %s | %s" %
                      (s["link_up"], self.forwarding, s["sess_ch"], tiles, self.note))
            time.sleep(0.002)

    def run_ui(self):
        import ui
        ui.run(self)


def main():
    ap = argparse.ArgumentParser(description="Steam Deck forwarder for OpenController")
    ap.add_argument("--port", help="CDC port (default: autodetect Valve 28DE:1302)")
    ap.add_argument("--input", help="force the Deck controller evdev node, e.g. /dev/input/event12 "
                    "(default: prefer the physical 28de:1205 controller)")
    ap.add_argument("--debug", action="store_true", help="headless status print, no UI")
    ap.add_argument("--hidraw", action="store_true", help="also read gyro/trackpads via hid-steam hidraw")
    args = ap.parse_args()
    app = App(args)
    try:
        if args.debug:
            app.run_debug()
        else:
            app.run_ui()
    except KeyboardInterrupt:
        pass
    finally:
        if app.forwarding:
            app.stop_forwarding()


if __name__ == "__main__":
    main()
