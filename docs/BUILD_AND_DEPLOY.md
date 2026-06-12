# Build And Deploy

These instructions cover firmware builds for macOS, Linux, and Windows, plus static hosting of the WebUSB app through GitHub Pages.

## 1. Prerequisites

Hardware:

- nRF52840 board supported by the Adafruit nRF52 Arduino core
- USB cable
- Steam Controller 2 controller

Software:

- `arduino-cli`
- Adafruit nRF52 Arduino core
- Chrome or Edge for the WebUSB app

## 2. Install Arduino CLI

### macOS

```bash
brew install arduino-cli
```

### Linux

Use your package manager if it ships a recent `arduino-cli`, or download the release archive from Arduino and place `arduino-cli` on `PATH`.

### Windows

Use [choco](https://chocolatey.org/) and do a `choco install arduino-cli`. If you don't have that then install the official [Arduino CLI zip or MSI](https://arduino.github.io/arduino-cli/1.5/installation/), then ensure `arduino-cli.exe` is on `PATH`.

## 3. Install the board core

Run once on any platform:

```bash
arduino-cli config init
arduino-cli core update-index
arduino-cli core install adafruit:nrf52 --additional-urls https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
```

## 4. Build the firmware

From the repository root:

```bash
arduino-cli compile -b adafruit:nrf52:feather52840 --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb} -DCFG_TUD_HID=4" OpenPuck
```

This sketch requires `CFG_TUD_HID=4` because Steam mode exposes four HID interfaces.

## 5. Upload the firmware

### macOS / Linux

Find the board port:

```bash
arduino-cli board list
```

Upload:

```bash
arduino-cli upload \
  -b adafruit:nrf52:feather52840 \
  -p /dev/ttyACM0 \
  OpenPuck
```

Replace `/dev/ttyACM0` with the actual port. On macOS it is usually `/dev/cu.usbmodem*`.

### Windows

Find the COM port:

```powershell
arduino-cli board list
```

Upload:

```powershell
arduino-cli upload `
  -b adafruit:nrf52:feather52840 `
  -p COM5 `
  OpenPuck
```

Replace `COM5` with the actual board port.

## 6. Factory reset (erase persistent storage)

Re-flashing firmware does **not** erase the board's internal LittleFS. The paired-controller bond (`bonds.bin`) and every saved setting (`cfg.bin`: USB mode, chord assignments, back-paddle map, mouse sensitivity) survive a fresh build and upload. To bring a board up in a truly clean state — a new unit, a hand-me-down with a stale bond, or a corrupted config — wipe the filesystem with one of:

- **Build flag (wipe on flash):** compile with `-DOPK_FACTORY_RESET=1` and upload. This build reformats all persistent storage on boot, so flashing it brings the board up on clean defaults — no console or panel needed:

  ```bash
  arduino-cli compile -b adafruit:nrf52:feather52840 --build-property "build.extra_flags=-DNRF52840_XXAA {build.flags.usb} -DCFG_TUD_HID=4 -DOPK_FACTORY_RESET=1" OpenPuck
  ```

  It wipes on **every** boot while the flag is compiled in, so this is a one-shot reset tool: flash the reset build once, let it boot (storage erased), then build and flash a **normal** firmware (without the flag) for regular use.
- **WebUSB panel (any mode):** open the panel (§8), and in the maintenance card click **⚠ Factory erase**. Confirm the two warning dialogs and type `ERASE` when prompted. The board reformats its filesystem and reboots to factory defaults. This works in every USB mode.
- **Serial console (CDC):** connect to the board's serial port at 115200 baud and send the line `ERASE-ALL` (exact, all caps). Same effect: reformat + reboot.

Both reformat the entire internal filesystem (`cfg.bin` + `bonds.bin` and anything else), so the action is irreversible and the controller must be **re-paired afterwards** (see §7).

Note on the serial method: puck (Steam/Lizard) mode drops the CDC console by default to free a USB endpoint for the wake-mouse interface, so the serial port may not be present in that mode. Either arm the one-shot debug CDC first (panel debug-CDC toggle / `D` console command, which keeps the console for the next boot), or just use the WebUSB **Factory erase** button, which is available in all modes.

## 7. Pair and verify

1. Flash the board.
2. Plug it into the host.
3. In Steam mode it enumerates as a puck-compatible device.
4. Pair the controller to one of the bond slots.
5. Verify that the slot returns `0xB4 = 0x02` when connected.

## 8. Run the WebUSB app locally

WebUSB requires a secure context. `http://localhost` qualifies.

### macOS / Linux

```bash
cd docs
python3 -m http.server 8008
```

Open:

```text
http://localhost:8008
```

### Windows

```powershell
cd docs
py -m http.server 8008
```

Open:

```text
http://localhost:8008
```

## 9. Known operational details

- Chrome or Edge is required for WebUSB.
- The board re-enumerates on USB mode switches.
- Poll interval tuning in the WebUSB app is session-only by design.
- Bonds persist in the board's internal filesystem.
