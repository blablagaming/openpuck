"""input_source.py -- read the Steam Deck's built-in controls and normalize to Triton fields.

Pure stdlib: talks the Linux **evdev** ioctl ABI on /dev/input/event* directly (no `evdev` package,
so `uv` needs no compiler -- only pyserial + pygame for the app). EVIOCGRAB *detaches* the controls
from everything else while forwarding; release to hand them back. Reliable in Desktop Mode.

Gyro + trackpads: richer than evdev and best read from the Steam Deck's `hid-steam` hidraw report --
that decode is hardware-specific and OFF by default (use_hidraw=True / --hidraw). Offsets in HidrawImu
are a documented starting point to verify on hardware.

Triton field layout matches triton.h (TB_* masks, int16 sticks/pads, 0..255 triggers, int16 IMU).
"""
from __future__ import annotations
import fcntl
import glob
import os
import struct

# ---- Triton button masks (mirror triton.h) ----
TB_A = 0x1
TB_B = 0x2
TB_X = 0x4
TB_Y = 0x8
TB_QAM = 0x10
TB_R3 = 0x20
TB_VIEW = 0x40
TB_R4 = 0x80
TB_R5 = 0x100
TB_RB = 0x200
TB_DDN = 0x400
TB_DRT = 0x800
TB_DLF = 0x1000
TB_DUP = 0x2000
TB_MENU = 0x4000
TB_L3 = 0x8000
TB_STEAM = 0x10000
TB_L4 = 0x20000
TB_L5 = 0x40000
TB_LB = 0x80000

NEUTRAL = {
    "buttons": 0, "lx": 0, "ly": 0, "rx": 0, "ry": 0, "lt": 0, "rt": 0,
    "lpx": 0, "lpy": 0, "rpx": 0, "rpy": 0,
    "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0,
}

# ---- Linux input ABI constants (stable kernel ABI) ----
EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03
BTN_SOUTH, BTN_EAST, BTN_NORTH, BTN_WEST = 0x130, 0x131, 0x133, 0x134
BTN_TL, BTN_TR = 0x136, 0x137
BTN_SELECT, BTN_START, BTN_MODE = 0x13A, 0x13B, 0x13C
BTN_THUMBL, BTN_THUMBR = 0x13D, 0x13E
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0, 1, 2, 3, 4, 5
ABS_HAT0X, ABS_HAT0Y = 0x10, 0x11

# evdev key code -> Triton button mask
KEYMAP = {
    BTN_SOUTH: TB_A, BTN_EAST: TB_B, BTN_WEST: TB_X, BTN_NORTH: TB_Y,
    BTN_TL: TB_LB, BTN_TR: TB_RB,
    BTN_SELECT: TB_VIEW, BTN_START: TB_MENU, BTN_MODE: TB_STEAM,
    BTN_THUMBL: TB_L3, BTN_THUMBR: TB_R3,
}

# struct input_event { __kernel_long tv_sec, tv_usec; __u16 type, code; __s32 value; } = 24 bytes (64-bit)
_EVENT = struct.Struct("qqHHi")
EVENT_SZ = _EVENT.size  # 24

# ---- ioctl number assembly (asm-generic _IOC) ----
_IOC_NONE, _IOC_WRITE, _IOC_READ = 0, 1, 2


def _IOC(d, t, nr, size):
    return (d << 30) | (size << 16) | (ord(t) << 8) | nr


EVIOCGID = _IOC(_IOC_READ, 'E', 0x02, 8)          # struct input_id (4x u16)


def EVIOCGNAME(length):
    return _IOC(_IOC_READ, 'E', 0x06, length)


def EVIOCGBIT(ev, length):
    return _IOC(_IOC_READ, 'E', 0x20 + ev, length)


def EVIOCGABS(abscode):
    return _IOC(_IOC_READ, 'E', 0x40 + abscode, 24)  # struct input_absinfo (6x s32)


EVIOCGRAB = _IOC(_IOC_WRITE, 'E', 0x90, 4)


def _test_bit(bitmask, bit):
    return bool(bitmask[bit // 8] & (1 << (bit % 8)))


class _RawDev:
    """Minimal pure-stdlib evdev device."""

    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass

    def ids(self):
        buf = bytearray(8)
        try:
            fcntl.ioctl(self.fd, EVIOCGID, buf, True)
            _bus, vid, pid, _ver = struct.unpack("<HHHH", buf)
            return vid, pid
        except Exception:
            return (0, 0)

    def name(self):
        buf = bytearray(256)
        try:
            fcntl.ioctl(self.fd, EVIOCGNAME(256), buf, True)
            return buf.split(b"\x00")[0].decode("latin1", "replace")
        except Exception:
            return ""

    def has_key(self, code):
        buf = bytearray(96)  # (KEY_MAX/8)+1
        try:
            fcntl.ioctl(self.fd, EVIOCGBIT(EV_KEY, len(buf)), buf, True)
            return _test_bit(buf, code)
        except Exception:
            return False

    def abs_range(self, code):
        buf = bytearray(24)
        try:
            fcntl.ioctl(self.fd, EVIOCGABS(code), buf, True)
            _val, mn, mx, _fz, _flat, _res = struct.unpack("<6i", buf)
            return (mn, mx)
        except Exception:
            return (-32768, 32767)

    def grab(self):
        fcntl.ioctl(self.fd, EVIOCGRAB, 1)

    def ungrab(self):
        try:
            fcntl.ioctl(self.fd, EVIOCGRAB, 0)
        except Exception:
            pass

    def read_events(self):
        """Yield (type, code, value); non-blocking."""
        try:
            data = os.read(self.fd, EVENT_SZ * 64)
        except (BlockingIOError, OSError):
            return
        for i in range(0, len(data) - EVENT_SZ + 1, EVENT_SZ):
            _s, _u, typ, code, val = _EVENT.unpack_from(data, i)
            yield typ, code, val


# Deck controller USB PIDs: 0x1205 = the PHYSICAL hid-steam controller (grabbing this DETACHES the pad,
# once Steam Input is off for the foreground app); 0x11FF = the VIRTUAL pad Steam Input synthesizes
# (only present while Steam Input is ON; grabbing it does NOT detach -- the physical pad keeps feeding
# Steam). So we prefer the physical node. Override with EvdevSource(path=...) / openctrl --input.
PHYS_PID = 0x1205
VIRT_PID = 0x11FF


def find_deck_controller():
    """Best gamepad event node to grab. Prefer the physical Deck controller (28de:1205); fall back to
    the virtual Steam pad (28de:11ff), then any other gamepad. Returns a path or None."""
    cands = []  # (priority, path) -- lower is better
    for path in sorted(glob.glob("/dev/input/event*")):
        try:
            d = _RawDev(path)
        except Exception:
            continue
        try:
            if not d.has_key(BTN_SOUTH):
                continue
            vid, pid = d.ids()
            nm = d.name().lower()
            valve = vid == 0x28DE or "steam" in nm or "deck" in nm
            if valve and pid == PHYS_PID:
                prio = 0
            elif valve and pid == VIRT_PID:
                prio = 2
            elif valve:
                prio = 1
            else:
                prio = 3  # generic gamepad, last resort
            cands.append((prio, path))
        finally:
            d.close()
    cands.sort()
    return cands[0][1] if cands else None


def _scale_stick(v, lo, hi):
    if hi <= lo:
        return 0
    mid = (lo + hi) / 2.0
    half = (hi - lo) / 2.0
    return int(max(-32768, min(32767, (v - mid) / half * 32767)))


def _scale_trig(v, lo, hi):
    if hi <= lo:
        return 0
    return int(max(0, min(255, (v - lo) / (hi - lo) * 255)))


class EvdevSource:
    """Buttons / sticks / triggers via raw evdev, with grab() / ungrab() for detach."""

    def __init__(self, path=None):
        self.path = path or find_deck_controller()
        if not self.path:
            raise RuntimeError("no Deck gamepad event device found under /dev/input")
        self.dev = _RawDev(self.path)
        self._name = self.dev.name() or self.path
        self._range = {c: self.dev.abs_range(c)
                       for c in (ABS_X, ABS_Y, ABS_RX, ABS_RY, ABS_Z, ABS_RZ)}
        self.state = dict(NEUTRAL)
        self._grabbed = False

    @property
    def name(self):
        return self._name

    def grab(self):
        if not self._grabbed:
            self.dev.grab()
            self._grabbed = True

    def ungrab(self):
        if self._grabbed:
            self.dev.ungrab()
            self._grabbed = False

    def pump(self):
        for typ, code, val in self.dev.read_events():
            self._apply(typ, code, val)
        return self.state

    def _apply(self, typ, code, val):
        s = self.state
        if typ == EV_KEY:
            mask = KEYMAP.get(code)
            if mask is not None:
                if val:
                    s["buttons"] |= mask
                else:
                    s["buttons"] &= ~mask
        elif typ == EV_ABS:
            if code == ABS_X:
                s["lx"] = _scale_stick(val, *self._range[ABS_X])
            elif code == ABS_Y:
                s["ly"] = -_scale_stick(val, *self._range[ABS_Y])  # evdev Y is down-positive
            elif code == ABS_RX:
                s["rx"] = _scale_stick(val, *self._range[ABS_RX])
            elif code == ABS_RY:
                s["ry"] = -_scale_stick(val, *self._range[ABS_RY])
            elif code == ABS_Z:
                s["lt"] = _scale_trig(val, *self._range[ABS_Z])
            elif code == ABS_RZ:
                s["rt"] = _scale_trig(val, *self._range[ABS_RZ])
            elif code == ABS_HAT0X:
                s["buttons"] &= ~(TB_DLF | TB_DRT)
                if val < 0:
                    s["buttons"] |= TB_DLF
                elif val > 0:
                    s["buttons"] |= TB_DRT
            elif code == ABS_HAT0Y:
                s["buttons"] &= ~(TB_DUP | TB_DDN)
                if val < 0:
                    s["buttons"] |= TB_DUP
                elif val > 0:
                    s["buttons"] |= TB_DDN


class HidrawImu:
    """Optional gyro + trackpad reader from the Deck's hid-steam hidraw node (PID 0x1205).

    DISABLED by default. The Steam Deck input report layout must be confirmed on hardware -- the offsets
    below are a documented starting point, not verified bytes. Until then forwarding works with evdev
    buttons/sticks/triggers and zero IMU.
    """

    PID = 0x1205

    def __init__(self):
        self.path = self._find()
        self.fd = open(self.path, "rb", buffering=0) if self.path else None

    def _find(self):
        for p in glob.glob("/dev/hidraw*"):
            try:
                ue = open("/sys/class/hidraw/%s/device/uevent" % p.split("/")[-1]).read().upper()
            except Exception:
                continue
            if "28DE" in ue and "1205" in ue:
                return p
        return None

    def read_into(self, state):
        if not self.fd:
            return False
        try:
            rpt = self.fd.read(64)
        except Exception:
            return False
        if not rpt or len(rpt) < 64:
            return False
        # Placeholder offsets per the SDL hid_steam Deck report; VERIFY on hardware.
        try:
            ax, ay, az, gx, gy, gz = struct.unpack_from("<6h", rpt, 28)
            state["ax"], state["ay"], state["az"] = ax, ay, az
            state["gx"], state["gy"], state["gz"] = gx, gy, gz
            lpx, lpy, rpx, rpy = struct.unpack_from("<4h", rpt, 16)
            state["lpx"], state["lpy"], state["rpx"], state["rpy"] = lpx, lpy, rpx, rpy
            return True
        except Exception:
            return False


def _list_inputs():
    """Diagnostic: print every /dev/input gamepad candidate and which one we'd grab.

        python3 input_source.py
    """
    pick = find_deck_controller()
    for path in sorted(glob.glob("/dev/input/event*")):
        try:
            d = _RawDev(path)
        except Exception as ex:
            print("%-22s (open failed: %s)" % (path, ex))
            continue
        try:
            vid, pid = d.ids()
            print("%-22s vid=%04x pid=%04x gamepad=%s  %s%s"
                  % (path, vid, pid, d.has_key(BTN_SOUTH), d.name(),
                     "   <== WILL GRAB" if path == pick else ""))
        finally:
            d.close()
    print("\nGrab target: %s" % (pick or
          "NONE found — is Steam holding the controller? try Desktop Mode"))


if __name__ == "__main__":
    _list_inputs()
