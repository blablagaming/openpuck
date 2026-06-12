# OpenPuck RF Sniffer

A **passive** 2.4 GHz sniffer for the link between a real Valve SC2 puck and its controller. It never
transmits, so it cannot disturb the pairing — it only listens, and streams every frame to a browser app in
real time with **Start Cap / End Cap**.

Use it to capture the actual puck↔controller traffic for any function (shutdown, LED, battery, …), isolate
the relevant frames, and export them.

## What you need
- A **second** nRF52840 board (e.g. the same Pro Micro / Feather you flash OpenPuck onto) — *not* the puck.
- The real Valve puck + controller, paired and working.
- Chrome or Edge (WebUSB).

## 1. Flash the sniffer firmware
```
arduino-cli compile -b adafruit:nrf52:feather52840 \
  --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb}" \
  --upload -p <PORT> puck_sniffer
```
(Same manual-DFU/double-tap-reset dance as OpenPuck if the port won't auto-reset.) It enumerates as
`28DE:534E "OpenPuck Sniffer"`.

## 2. Open the app
WebUSB needs a secure context, so **not** a `file://` path. Either:
- **Hosted:** https://safijari.github.io/OpenPuck/sniffer.html (once `docs/` is pushed to gh-pages), or
- **Local:** `cd docs && python3 -m http.server` → open http://localhost:8000/sniffer.html

Click **Connect sniffer**, pick *OpenPuck Sniffer*.

## 3. Capture
The link is Nordic **Gazell**: the on-air address is a **System Address random per session** (not derived from
the bond), and the active `E3`-poll / `F1`-reply exchange rides a small channel set `{primary, 2, 80}` derived
from that address. The puck's ~5 Hz `E1`/`E2` **keepalive** trickles in even when the controller isn't being
actively polled — so seeing only `E2` and **`C→P: 0`** means the controller isn't in an active session *yet*.

Recipe (mirrors `~/nrf-sniffer/capture_puck_session.py`):
1. Status shows **ACQUIRE** → it catches the puck's `E1` on ch2 and locks the session address.
2. It then **hunts the channel set** for the controller's `0xF_` replies. **Make the session active**: with the
   controller connected, **do heavy trackpad / move sticks** so the puck polls fast. Watch **`C→P`** climb —
   that means it parked on the live channel. (Bonded reconnect uses a *fresh* address → if it won't lock,
   **power-cycle the controller** during ACQUIRE so the new `E1` is heard.)
3. With `C→P` climbing, **● Start Cap**, trigger the action in Steam (turn off / LED / let battery move),
   **■ End Cap**, **Download**. Send me the file.

Direction: `P→C` (opcode `0xE_`, puck→controller — LED/shutoff commands ride here) vs `C→P` (`0xF_`,
controller→puck — battery/telemetry rides here). The `filter` box matches payload hex (e.g. `9f`).

> Bond info (which controller/slot, its serial + uuids) is read over USB, not the air — `scmd`/`pairtui.py`
> in `~/nrf-sniffer`. Useful for labelling a capture; it does **not** give the on-air address (random per session).

## How it works
After RX the radio buffer is `[LENGTH][S1][payload…]`; `payload[0]` is the opcode. The puck advertises its
per-session base/prefix/channel inside the `E1` host frame on the shared `ibex`/ch2 rendezvous, so the sniffer
reads those, retunes to the session, and receives both directions (same ESB base — and it now listens on **all
8 ESB pipes** of that base, so a controller reply on a different prefix is still caught; the **pipe** column /
status `last pipe` shows which `RXMATCH` each frame hit). A QoS channel-hop is followed via the `E1` keepalive.
When the session goes silent it **sweeps clean candidate channels keeping the learned address** (rather than
abandoning to ch2) and only falls back to a full discovery scan after two dry sweeps. Radio parameters are
OpenPuck's CRC-validated config (`radio.cpp`).

### If you only see `E2`/`E1` on ch2 and never the controller's `0xF_` replies
That means it never camps on the session. Read a real `E1` row in the table (a `P→C`, op `e1` frame): its
payload bytes are `e1 [proteus:4] [ibex:4] [CH] 00 00 00 [BASE:4] [PFX]` — i.e. **byte 9 = session channel,
bytes 13–16 = session base, byte 17 = prefix** (the "E1 advertises" stat shows the sniffer's own read of these).
Cross-check that against where traffic actually is, then type the six values into **pin session**
(`b0 b1 b2 b3 pfx ch`, address bytes hex) and click **Pin** to force the sniffer onto that exact session — this
bypasses auto-acquire entirely, which is the reliable path if the real puck's `E1` offsets differ from ours.

## Stream protocol (WebUSB bulk)
- packet: `C0 DE [N] [t_us:4 LE] [ch] [flags] [rssi] [match] [N raw bytes]`  (flags bit0 = CRC ok; match = RXMATCH pipe; raw = `[LEN][S1][payload]`)
- status: `C1 DE [state] [curCh] [base:4] [prefix] [cap] [hb:2] [advCh] [advBase:4] [advPfx] [lastMatch]`  (19 B; adv* = session parsed from the last `E1`)
- commands (bulk OUT): `01` start · `02` stop · `03` re-acquire · `04 <ch>` pin channel · `05 <b0 b1 b2 b3 pfx ch>` pin full session
