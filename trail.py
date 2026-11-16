#!/usr/bin/env python3
"""Trail Recorder - GPS trail logger for M5Stack CardputerZero + Cap LoRa-1262.

Reads NMEA from the Cap's GNSS receiver (/dev/ttyS0, 115200 8N1), records
your route as a standard GPX 1.1 file and shows live trip statistics.

Pages
  1 Dashboard   speed, distance, time, recording state
  2 Map         your path, start point, waypoints, scale bar
  3 Elevation   altitude profile, climb / descent, min / max
  4 Satellites  fix, signal per satellite, position, accuracy
  5 Trails      saved GPX files with distance and duration

Keys
  Enter / Space   start, pause, resume recording
  S               stop and save           W   mark a waypoint
  U               miles <-> kilometres    Left/Right, Tab, 1-5  pages
  Up/Down         select a saved trail    D   delete selected trail
  ESC / Q         quit (asks first while recording)

Run with --preview out.png PAGE --mock to render a frame without hardware.
"""

import fcntl
import glob
import math
import mmap
import os
import re
import select
import signal
import struct
import sys
import termios
import time
from collections import deque
from datetime import datetime, timezone

from PIL import Image, ImageDraw, ImageFont

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

VERSION = "1.0.3"
APP = "pocket-trail"

# --------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------
W, H = 320, 170
FB_DEVICE = os.environ.get("APPLAUNCH_LINUX_FBDEV_DEVICE", "/dev/fb0")
GPS_DEVICE = os.environ.get("TRAIL_GPS_DEV", "/dev/ttyS0")
EXT5V_PATH = os.environ.get("TRAIL_EXT5V_PATH", "/sys/class/leds/ext_5v_out/brightness")
FONT_DIR = "/usr/share/fonts/truetype/dejavu"
HOME = os.path.expanduser("~")
TRAIL_DIR = os.environ.get("TRAIL_DIR", os.path.join(HOME, "Trails"))
CONFIG_DIR = os.path.join(os.environ.get("XDG_CONFIG_HOME") or os.path.join(HOME, ".config"), APP)
CONFIG_FILE = os.path.join(CONFIG_DIR, "trail.conf")

MIN_STEP_M = 5.0         # ignore movement smaller than this (GPS jitter)
MAX_HDOP = 6.0           # ignore points worse than this
MIN_INTERVAL_S = 1.0     # at most one track point per second
MAX_INTERVAL_S = 30.0    # but at least one every 30 s while recording
CLIMB_HYST_M = 3.0       # elevation gain/loss hysteresis

PAGES = ["Dashboard", "Map", "Elevation", "Satellites", "Trails"]
P_DASH, P_MAP, P_ELEV, P_SATS, P_FILES = range(5)

FBIOGET_VSCREENINFO = 0x4600
FBIOGET_FSCREENINFO = 0x4602

EV_KEY = 0x01
KEY_ESC, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5 = 1, 2, 3, 4, 5, 6
KEY_TAB, KEY_Q, KEY_W, KEY_Y, KEY_U = 15, 16, 17, 21, 22
KEY_ENTER, KEY_S, KEY_D, KEY_N = 28, 31, 32, 49
KEY_SEMICOLON, KEY_COMMA, KEY_DOT, KEY_SLASH = 39, 51, 52, 53
KEY_SPACE = 57
KEY_UP, KEY_LEFT, KEY_RIGHT, KEY_DOWN = 103, 105, 106, 108
NUM_KEYS = {KEY_1: 0, KEY_2: 1, KEY_3: 2, KEY_4: 3, KEY_5: 4}

BG = (10, 12, 16)
PANEL = (24, 28, 36)
GRID = (40, 46, 58)
SEL = (34, 52, 74)
TEXT = (230, 234, 240)
DIM = (130, 138, 152)
GREEN = (60, 220, 130)
GREEN_FILL = (20, 70, 50)
CYAN = (60, 190, 255)
YELLOW = (255, 205, 60)
ORANGE = (255, 140, 50)
ORANGE_FILL = (80, 45, 20)
RED = (255, 75, 75)
PURPLE = (175, 125, 255)

_running = True


def _stop(_sig, _frm):
    global _running
    _running = False


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def load_font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    for path in (os.path.join(FONT_DIR, name), "/usr/share/fonts/TTF/" + name,
                 "/Library/Fonts/" + name):
        try:
            return ImageFont.truetype(path, size)
        except (OSError, IOError):
            pass
    return ImageFont.load_default()


def haversine(lat1, lon1, lat2, lon2):
    r = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def bearing(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def fmt_dur(sec):
    sec = int(max(0, sec))
    h, rem = divmod(sec, 3600)
    m, s = divmod(rem, 60)
    return "{}:{:02d}:{:02d}".format(h, m, s) if h else "{}:{:02d}".format(m, s)


def compass_point(deg):
    pts = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return pts[int((deg + 22.5) % 360 // 45)]


class Units:
    def __init__(self, imperial=True):
        self.imperial = imperial

    def dist(self, m):
        if self.imperial:
            mi = m / 1609.344
            return ("{:.2f}".format(mi) if mi < 100 else "{:.1f}".format(mi)), "mi"
        km = m / 1000.0
        return ("{:.2f}".format(km) if km < 100 else "{:.1f}".format(km)), "km"

    def speed(self, mps):
        if mps is None:
            return "--", ("mph" if self.imperial else "km/h")
        v = mps * (2.236936 if self.imperial else 3.6)
        return ("{:.1f}".format(v) if v < 100 else "{:.0f}".format(v)), ("mph" if self.imperial else "km/h")

    def alt(self, m):
        if m is None:
            return "--", ("ft" if self.imperial else "m")
        return "{:.0f}".format(m * 3.28084 if self.imperial else m), ("ft" if self.imperial else "m")

    def short(self, m):
        """Short distances for the map scale bar."""
        if self.imperial:
            ft = m * 3.28084
            return "{:.0f} ft".format(ft) if ft < 1000 else "{:.2g} mi".format(m / 1609.344)
        return "{:.0f} m".format(m) if m < 1000 else "{:.2g} km".format(m / 1000.0)

    def pace(self, mps):
        """Minutes per mile/km, or None when barely moving."""
        if not mps or mps < 0.3:
            return None
        sec = (1609.344 if self.imperial else 1000.0) / mps
        return "{}:{:02d} /{}".format(int(sec // 60), int(sec % 60), "mi" if self.imperial else "km")


def load_config():
    cfg = {"imperial": True}
    try:
        with open(CONFIG_FILE) as f:
            for line in f:
                k, _, v = line.partition("=")
                if k.strip() == "units":
                    cfg["imperial"] = v.strip() != "metric"
    except OSError:
        pass
    return cfg


def save_config(cfg):
    try:
        os.makedirs(CONFIG_DIR, exist_ok=True)
        with open(CONFIG_FILE, "w") as f:
            f.write("units={}\n".format("imperial" if cfg["imperial"] else "metric"))
    except OSError:
        pass


# --------------------------------------------------------------------------
# NMEA parsing
# --------------------------------------------------------------------------
def nmea_checksum_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def _f(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _latlon(v, hemi):
    if not v:
        return None
    try:
        dot = v.index(".")
        deg = float(v[:dot - 2])
        minutes = float(v[dot - 2:])
    except ValueError:
        return None
    val = deg + minutes / 60.0
    return -val if hemi in ("S", "W") else val


class GpsState:
    """Accumulates the latest fix from NMEA sentences."""

    def __init__(self):
        self.lat = self.lon = None
        self.alt = None
        self.speed = None        # m/s, as reported by the receiver (noisy when still)
        self.ground_speed = None # m/s, smoothed and 0 when actually standing still
        self.stationary = True
        self._speed_ema = None
        self._shown = None
        self._fast = 0
        self._slow = 0
        self._track = deque()    # (t, lat, lon) over the last few seconds
        self.course = None       # degrees true
        self.fix_quality = 0     # GGA: 0 none, 1 GPS, 2 DGPS ...
        self.fix_mode = 1        # GSA: 1 none, 2 2D, 3 3D
        self.valid = False       # RMC status A
        self.hdop = self.pdop = self.vdop = None
        self.sats_used = 0
        self.used_prns = set()
        self._used_pending = set()
        self.sats = {}           # (system, prn) -> dict(elev, az, snr, t)
        self.utc = None          # datetime
        self.last_sentence = 0.0
        self.last_fix_time = 0.0
        self.sentences = 0
        self.bad_checksums = 0
        self.first_fix_at = None
        self.started = time.monotonic()
        self._gsv_buf = {}

    @property
    def has_fix(self):
        return (self.valid and self.lat is not None and self.fix_quality != 0
                and time.monotonic() - self.last_fix_time < 5)

    def feed(self, line):
        line = line.strip()
        if not line.startswith("$"):
            return None
        if not nmea_checksum_ok(line):
            self.bad_checksums += 1
            return None
        self.sentences += 1
        self.last_sentence = time.monotonic()
        body = line[1:line.index("*")]
        parts = body.split(",")
        talker, kind = parts[0][:2], parts[0][2:]
        try:
            if kind == "RMC":
                return self._rmc(parts)
            if kind == "GGA":
                return self._gga(parts)
            if kind == "GSA":
                self._gsa(parts)
            elif kind == "GSV":
                self._gsv(talker, parts)
            elif kind == "VTG":
                sp = _f(parts[7]) if len(parts) > 7 else None
                if sp is not None:
                    self.speed = sp / 3.6
        except (IndexError, ValueError):
            pass
        return None

    def _rmc(self, p):
        # $xxRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,...
        self.valid = p[2] == "A"
        if p[1] and len(p) > 9 and len(p[9]) == 6:
            try:
                hh, mm, ss = int(p[1][0:2]), int(p[1][2:4]), float(p[1][4:])
                dd, mo, yy = int(p[9][0:2]), int(p[9][2:4]), int(p[9][4:6])
                self.utc = datetime(2000 + yy, mo, dd, hh, mm, int(ss),
                                    int((ss % 1) * 1e6), tzinfo=timezone.utc)
            except ValueError:
                pass
        if self.valid:
            lat, lon = _latlon(p[3], p[4]), _latlon(p[5], p[6])
            if lat is not None and lon is not None:
                self.lat, self.lon = lat, lon
                self.last_fix_time = time.monotonic()
                if self.first_fix_at is None:
                    self.first_fix_at = time.monotonic()
            kn = _f(p[7])
            if kn is not None:
                self.speed = kn * 0.514444
            crs = _f(p[8])
            self.course = crs
            self._motion()
            return "fix"
        return None

    def _motion(self):
        """Decide whether we're really moving.

        Receivers report a small non-zero speed (often 0.5-1 mph) while
        stationary because the position wanders. Show 0 unless the position has
        actually moved beyond GPS jitter over the last few seconds.
        """
        now = time.monotonic()
        self._track.append((now, self.lat, self.lon))
        while self._track and now - self._track[0][0] > 8.0:
            self._track.popleft()
        raw = self.speed or 0.0
        # Slow average: only used to decide moving / standing still
        self._speed_ema = raw if self._speed_ema is None else self._speed_ema + 0.4 * (raw - self._speed_ema)
        ema = self._speed_ema
        # Shown speed follows the receiver closely (its Doppler speed is accurate
        # when moving); light smoothing only at walking speeds where it wobbles.
        if self._shown is None or raw >= 2.0 or abs(raw - self._shown) > 0.8:
            self._shown = raw
        else:
            self._shown += 0.7 * (raw - self._shown)
        # Quick start / stop from consecutive raw readings (still-noise stays below 0.45 m/s)
        self._fast = self._fast + 1 if raw >= 1.0 else 0
        self._slow = self._slow + 1 if raw < 0.45 else 0
        t0, la0, lo0 = self._track[0]
        span = now - t0
        if self.stationary and (self._fast >= 3 or (raw >= 1.5 and self._fast >= 2)):
            self.stationary = False
        elif not self.stationary and self._slow >= 2:
            self.stationary = True
        elif span >= 3.0:
            disp = haversine(la0, lo0, self.lat, self.lon)
            jitter = max(4.0, (self.hdop or 1.0) * 3.0)
            pos_moving = disp >= jitter and disp / span >= 0.4
            if self.stationary:
                # need both a real speed and real displacement to start moving
                if ema >= 0.5 and pos_moving:
                    self.stationary = False
            elif raw < 0.6 and (ema < 0.35 or (not pos_moving and ema < 0.6)):
                self.stationary = True
        elif ema >= 2.0:
            self.stationary = False   # clearly moving (e.g. a car) even before the window fills
        self.ground_speed = 0.0 if self.stationary else self._shown

    def _gga(self, p):
        # $xxGGA,time,lat,N,lon,E,quality,numsats,hdop,alt,M,...
        # GGA starts a new reporting cycle: publish the satellites GSA listed last cycle
        if self._used_pending:
            self.used_prns, self._used_pending = self._used_pending, set()
        self.fix_quality = int(p[6] or 0)
        self.sats_used = int(p[7] or 0)
        h = _f(p[8])
        if h is not None:
            self.hdop = h
        a = _f(p[9])
        if a is not None and self.fix_quality:
            self.alt = a
        if self.fix_quality:
            lat, lon = _latlon(p[2], p[3]), _latlon(p[4], p[5])
            if lat is not None and lon is not None:
                self.lat, self.lon = lat, lon
                self.last_fix_time = time.monotonic()
        return None

    def _gsa(self, p):
        # $xxGSA,A,3,prn x12,pdop,hdop,vdop[,systemId]
        self.fix_mode = int(p[2] or 1)
        prns = {x for x in p[3:15] if x}
        self._used_pending |= prns
        if not self.used_prns:
            self.used_prns = set(prns)
        self.pdop, self.hdop, self.vdop = _f(p[15]), _f(p[16]) or self.hdop, _f(p[17])

    def _gsv(self, talker, p):
        # $xxGSV,total,msgnum,insight,(prn,elev,az,snr)x4[,signal]
        now = time.monotonic()
        fields = p[4:]
        if len(fields) % 4 == 1:  # NMEA 4.10 signal id on the end
            fields = fields[:-1]
        for i in range(0, len(fields) - 3, 4):
            prn = fields[i]
            if not prn:
                continue
            self.sats[(talker, prn)] = {
                "sys": talker, "prn": prn,
                "elev": _f(fields[i + 1]), "az": _f(fields[i + 2]),
                "snr": _f(fields[i + 3]), "t": now}
        # forget satellites not reported for 10 s
        for k in [k for k, v in self.sats.items() if now - v["t"] > 10]:
            del self.sats[k]


# --------------------------------------------------------------------------
# Hardware: Cap power + serial port
# --------------------------------------------------------------------------
class CapPower:
    """Switches the Cap's 5 V rail on through the ext_5v_out LED-class file,
    restoring the previous state on exit (same approach as M5Stack's GPS app)."""

    def __init__(self, path=EXT5V_PATH):
        self.path = path
        self.previous = None
        self.error = None

    def enable(self):
        if not os.path.exists(self.path):
            self.error = None  # not this hardware; assume always powered
            return True
        try:
            with open(self.path) as f:
                self.previous = int(f.read().strip() or 0)
        except (OSError, ValueError) as e:
            self.error = "read {}: {}".format(self.path, e)
            return False
        if self.previous > 0:
            return True
        try:
            with open(self.path, "w") as f:
                f.write("1\n")
            return True
        except OSError as e:
            self.error = ("Can't power the Cap ({}). Reboot once after installing, "
                          "so the new permission rule applies.".format(e.strerror))
            self.previous = None
            return False

    def restore(self):
        if self.previous is not None and self.previous <= 0:
            try:
                with open(self.path, "w") as f:
                    f.write("{}\n".format(self.previous))
            except OSError:
                pass
        self.previous = None


class GpsPort:
    def __init__(self, path=GPS_DEVICE):
        self.path = path
        self.fd = None
        self.buf = b""
        self.error = None
        self.saved = None

    def open(self):
        try:
            self.fd = os.open(self.path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            self.error = "{}: {}".format(self.path, e.strerror)
            if e.errno in (13,):
                self.error += " (reboot once after installing)"
            return False
        try:
            attrs = termios.tcgetattr(self.fd)
            self.saved = list(attrs)
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
            iflag = 0
            oflag = 0
            lflag = 0
            cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
            cc = list(cc)
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = 0
            baud = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW,
                              [iflag, oflag, cflag, lflag, baud, baud, cc])
        except termios.error:
            pass  # not a real tty (tests / pipes): just read
        return True

    def lines(self):
        if self.fd is None:
            return []
        try:
            chunk = os.read(self.fd, 4096)
        except (BlockingIOError, InterruptedError):
            return []
        except OSError as e:
            self.error = str(e)
            return []
        if not chunk:
            return []
        self.buf += chunk
        if len(self.buf) > 16384:
            self.buf = self.buf[-4096:]
        *done, self.buf = self.buf.split(b"\n")
        return [d.decode("ascii", "replace") for d in done]

    def close(self):
        if self.fd is not None:
            if self.saved:
                try:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
                except termios.error:
                    pass
            os.close(self.fd)
            self.fd = None


# --------------------------------------------------------------------------
# Recording to GPX
# --------------------------------------------------------------------------
def _iso(dt):
    return dt.strftime("%Y-%m-%dT%H:%M:%SZ")


def _xml(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            .replace('"', "&quot;"))


def gpx_header(name, when):
    return ('<?xml version="1.0" encoding="UTF-8"?>\n'
            '<gpx version="1.1" creator="Trail Recorder for CardputerZero" '
            'xmlns="http://www.topografix.com/GPX/1/1">\n'
            '  <metadata><name>{}</name><time>{}</time></metadata>\n').format(_xml(name), _iso(when))


def finalize_part(part_path):
    """Turn a .gpx.part (+ .wpt.part) stream into a valid .gpx file."""
    base = part_path[:-len(".gpx.part")]
    wpt_path = base + ".wpt.part"
    out_path = base + ".gpx"
    try:
        with open(part_path) as f:
            body = f.read()
    except OSError:
        return None
    header, _, track = body.partition("<!--TRACK-->\n")
    wpts = ""
    if os.path.exists(wpt_path):
        with open(wpt_path) as f:
            wpts = f.read()
    track = track.rstrip()
    if track.endswith("<trkseg>"):          # empty last segment
        track = track[:-len("<trkseg>")].rstrip()
    elif track and not track.endswith("</trkseg>"):
        track += "\n    </trkseg>"
    if "<trkseg>" in track:
        trk = "  <trk><name>{}</name>\n{}\n  </trk>\n".format(
            _xml(os.path.basename(base)), track)
    else:
        trk = ""
    with open(out_path + ".tmp", "w") as f:
        f.write(header + wpts + trk + "</gpx>\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(out_path + ".tmp", out_path)
    for p in (part_path, wpt_path):
        try:
            os.remove(p)
        except OSError:
            pass
    return out_path


def recover_unfinished():
    """Finish any recording interrupted by a crash or power loss."""
    done = []
    for part in glob.glob(os.path.join(TRAIL_DIR, "*.gpx.part")):
        try:
            p = finalize_part(part)
            if p:
                done.append(p)
        except OSError:
            pass
    return done


class Recorder:
    """Streams track points to disk as they arrive, so nothing is lost on a crash."""

    def __init__(self):
        self.state = "idle"      # idle | recording | paused
        self.reset()

    def reset(self):
        self.points = []         # (lat, lon, alt, t_monotonic) kept for stats and drawing
        self.waypoints = []      # (lat, lon, name)
        self.distance = 0.0
        self.moving_time = 0.0
        self.elapsed = 0.0
        self.max_speed = 0.0
        self.climb = 0.0
        self.descent = 0.0
        self._alt_ref = None
        self.elev = []           # (distance_m, alt_m)
        self.last_kept = None    # (lat, lon, t_monotonic)
        self.last_point_t = 0.0
        self.seg_start = None
        self.start_wall = None
        self.base = None
        self.f = None
        self.wf = None
        self.need_seg = True
        self.smooth = None       # smoothed (lat, lon)

    # -- file handling --
    def _open(self, utc):
        os.makedirs(TRAIL_DIR, exist_ok=True)
        when = utc or datetime.now(timezone.utc)
        local = datetime.now()
        name = local.strftime("%Y-%m-%d_%H%M")
        base = os.path.join(TRAIL_DIR, name)
        n = 2
        while os.path.exists(base + ".gpx") or os.path.exists(base + ".gpx.part"):
            base = os.path.join(TRAIL_DIR, "{}_{}".format(name, n))
            n += 1
        self.base = base
        self.f = open(base + ".gpx.part", "w", buffering=1)
        self.f.write(gpx_header("Trail " + local.strftime("%Y-%m-%d %H:%M"), when))
        self.f.write("<!--TRACK-->\n")
        self.f.flush()
        self.wf = open(base + ".wpt.part", "w", buffering=1)

    def start(self, gps):
        if self.state == "idle":
            self.reset()
            self._open(gps.utc)
            self.start_wall = time.monotonic()
        self.state = "recording"
        self.seg_start = time.monotonic()
        self.need_seg = True
        self.last_kept = None    # don't join a line across the pause
        self.smooth = None

    def pause(self):
        if self.state == "recording":
            self.elapsed += time.monotonic() - self.seg_start
            self.state = "paused"
            if self.f and not self.need_seg:
                self.f.write("    </trkseg>\n")
                self.f.flush()
            self.need_seg = True

    def stop(self):
        """Finish the file. Returns the saved path (or None if nothing recorded)."""
        if self.state == "idle":
            return None
        self.pause()
        self.state = "idle"
        path = None
        if self.f:
            self.f.close()
            self.wf.close()
            self.f = self.wf = None
            if self.points or self.waypoints:
                path = finalize_part(self.base + ".gpx.part")
            else:
                for ext in (".gpx.part", ".wpt.part"):
                    try:
                        os.remove(self.base + ext)
                    except OSError:
                        pass
        return path

    @property
    def total_time(self):
        t = self.elapsed
        if self.state == "recording" and self.seg_start:
            t += time.monotonic() - self.seg_start
        return t

    # -- data --
    def add_waypoint(self, gps):
        if self.state == "idle" or not gps.has_fix:
            return None
        name = "WP{}".format(len(self.waypoints) + 1)
        self.waypoints.append((gps.lat, gps.lon, name))
        ele = "<ele>{:.1f}</ele>".format(gps.alt) if gps.alt is not None else ""
        tm = "<time>{}</time>".format(_iso(gps.utc)) if gps.utc else ""
        self.wf.write('  <wpt lat="{:.7f}" lon="{:.7f}">{}{}<name>{}</name></wpt>\n'.format(
            gps.lat, gps.lon, ele, tm, name))
        self.wf.flush()
        return name

    def update(self, gps):
        """Call on every new fix while recording."""
        if self.state != "recording" or not gps.has_fix:
            return
        if gps.hdop is not None and gps.hdop > MAX_HDOP:
            return
        now = time.monotonic()
        if now - self.last_point_t < MIN_INTERVAL_S:
            return
        # Smooth the raw fixes: GPS jitter of a few metres per second would
        # otherwise be counted as distance. Smooth harder when slow (where
        # jitter dominates) and lightly when fast (so driving doesn't lag).
        spd = getattr(gps, "ground_speed", None)
        if spd is None:
            spd = gps.speed or 0.0
        k = min(0.9, 0.25 + spd / 8.0)
        if self.smooth is None:
            self.smooth = (gps.lat, gps.lon)
        else:
            self.smooth = (self.smooth[0] + k * (gps.lat - self.smooth[0]),
                           self.smooth[1] + k * (gps.lon - self.smooth[1]))
        lat, lon = self.smooth
        alt = gps.alt
        if self.last_kept is not None:
            step = haversine(self.last_kept[0], self.last_kept[1], lat, lon)
            jitter = max(MIN_STEP_M, (gps.hdop or 1.0) * 3.0)
            if spd < 0.4 and step < 3 * jitter:
                jitter = 3 * jitter   # standing still: be much stricter
            if step < jitter and now - self.last_point_t < MAX_INTERVAL_S:
                return
            dt = now - self.last_kept[2]
            if step >= jitter:
                self.distance += step
                self.moving_time += dt
        if 0.5 < spd < 70:   # ignore standing-still noise and glitches
            self.max_speed = max(self.max_speed, spd)
        self.last_kept = (lat, lon, now)
        self.last_point_t = now
        self.points.append((lat, lon, alt, now))
        if alt is not None:
            self.elev.append((self.distance, alt))
            if self._alt_ref is None:
                self._alt_ref = alt
            elif alt - self._alt_ref >= CLIMB_HYST_M:
                self.climb += alt - self._alt_ref
                self._alt_ref = alt
            elif self._alt_ref - alt >= CLIMB_HYST_M:
                self.descent += self._alt_ref - alt
                self._alt_ref = alt
        if self.need_seg:
            self.f.write("    <trkseg>\n")
            self.need_seg = False
        ele = "<ele>{:.1f}</ele>".format(alt) if alt is not None else ""
        tm = "<time>{}</time>".format(_iso(gps.utc)) if gps.utc else ""
        self.f.write('      <trkpt lat="{:.7f}" lon="{:.7f}">{}{}</trkpt>\n'.format(lat, lon, ele, tm))
        self.f.flush()


# --------------------------------------------------------------------------
# Saved trails
# --------------------------------------------------------------------------
_TRKPT = re.compile(r'<trkpt lat="([-0-9.]+)" lon="([-0-9.]+)">(?:<ele>([-0-9.]+)</ele>)?(?:<time>([^<]+)</time>)?')
_summary_cache = {}


def trail_summary(path):
    try:
        st = os.stat(path)
    except OSError:
        return None
    key = (path, st.st_mtime, st.st_size)
    if key in _summary_cache:
        return _summary_cache[key]
    dist, prev, times, pts, wpts = 0.0, None, [], 0, 0
    try:
        with open(path) as f:
            for line in f:
                if "<wpt" in line:
                    wpts += 1
                if "</trkseg>" in line:
                    prev = None
                m = _TRKPT.search(line)
                if not m:
                    continue
                lat, lon = float(m.group(1)), float(m.group(2))
                if prev:
                    dist += haversine(prev[0], prev[1], lat, lon)
                prev = (lat, lon)
                pts += 1
                if m.group(4):
                    times.append(m.group(4))
    except OSError:
        return None
    dur = 0
    if len(times) >= 2:
        try:
            t0 = datetime.strptime(times[0], "%Y-%m-%dT%H:%M:%SZ")
            t1 = datetime.strptime(times[-1], "%Y-%m-%dT%H:%M:%SZ")
            dur = (t1 - t0).total_seconds()
        except ValueError:
            pass
    s = {"path": path, "name": os.path.basename(path)[:-4], "dist": dist, "dur": dur,
         "points": pts, "wpts": wpts, "size": st.st_size}
    _summary_cache[key] = s
    return s


def list_trails():
    out = []
    for p in sorted(glob.glob(os.path.join(TRAIL_DIR, "*.gpx")), reverse=True):
        s = trail_summary(p)
        if s:
            out.append(s)
    return out


# --------------------------------------------------------------------------
# Drawing
# --------------------------------------------------------------------------
class Renderer:
    def __init__(self):
        self.f_title = load_font(13, bold=True)
        self.f_huge = load_font(34, bold=True)
        self.f_big = load_font(22, bold=True)
        self.f_med = load_font(13, bold=True)
        self.f_reg = load_font(12)
        self.f_sm = load_font(10)
        self.f_smb = load_font(10, bold=True)

    def panel(self, d, box):
        d.rounded_rectangle(box, radius=6, fill=PANEL)

    def tr(self, d, xr, y, s, font, fill):
        d.text((xr - d.textlength(s, font=font), y), s, font=font, fill=fill)

    def tc(self, d, xc, y, s, font, fill):
        d.text((xc - d.textlength(s, font=font) / 2, y), s, font=font, fill=fill)

    def value(self, d, x, y, label, val, unit, font, col=TEXT):
        d.text((x, y), label, font=self.f_sm, fill=DIM)
        d.text((x, y + 12), val, font=font, fill=col)
        if unit:
            vw = d.textlength(val, font=font)
            d.text((x + vw + 3, y + 12 + (font.size - 10) - 1), unit, font=self.f_sm, fill=DIM)

    def header(self, d, page, s):
        d.rectangle((0, 0, W, 18), fill=PANEL)
        d.text((6, 2), "Trail", font=self.f_title, fill=GREEN)
        for i in range(len(PAGES)):
            cx = 48 + i * 8
            d.ellipse((cx, 7, cx + 4, 11), fill=GREEN if i == page else GRID)
        d.text((92, 4), PAGES[page], font=self.f_smb, fill=TEXT)
        rec = s["rec"]
        if rec.state == "recording":
            on = int(time.monotonic() * 2) % 2 == 0
            d.ellipse((172, 5, 180, 13), fill=RED if on else (120, 30, 30))
            d.text((184, 4), "REC", font=self.f_smb, fill=RED)
        elif rec.state == "paused":
            d.rectangle((172, 5, 174, 13), fill=YELLOW)
            d.rectangle((177, 5, 179, 13), fill=YELLOW)
            d.text((184, 4), "PAUSED", font=self.f_smb, fill=YELLOW)
        gps = s["gps"]
        if gps.has_fix:
            txt, col = "{}D {}sat".format(3 if gps.fix_mode == 3 else 2, gps.sats_used), GREEN
        elif gps.sentences and time.monotonic() - gps.last_sentence < 3:
            txt, col = "no fix", YELLOW
        else:
            txt, col = "no GPS", RED
        self.tr(d, 270, 4, txt, self.f_smb, col)
        t = gps.utc.astimezone().strftime("%H:%M") if gps.utc else datetime.now().strftime("%H:%M")
        self.tr(d, 314, 3, t, self.f_reg, TEXT)

    def footer(self, d, text):
        d.text((6, H - 13), text, font=self.f_sm, fill=DIM)

    def no_gps_hint(self, d, s, box):
        gps, err = s["gps"], s["error"]
        x0, y0, x1, y1 = box
        cx = (x0 + x1) / 2
        if err:
            self.tc(d, cx, y0 + 6, "GPS unavailable", self.f_med, RED)
            import textwrap
            for i, line in enumerate(textwrap.wrap(err, 46)[:3]):
                self.tc(d, cx, y0 + 26 + i * 13, line, self.f_sm, DIM)
        elif not gps.sentences or time.monotonic() - gps.last_sentence > 3:
            self.tc(d, cx, y0 + 6, "Waiting for GPS data…", self.f_med, YELLOW)
            self.tc(d, cx, y0 + 26, "Is the Cap LoRa-1262 attached?", self.f_sm, DIM)
            self.tc(d, cx, y0 + 39, "(power off before attaching it)", self.f_sm, DIM)
        else:
            vis = len(s["gps"].sats)
            waited = time.monotonic() - gps.started
            self.tc(d, cx, y0 + 6, "Searching for satellites…", self.f_med, YELLOW)
            self.tc(d, cx, y0 + 26, "{} in view · {}".format(vis, fmt_dur(waited)), self.f_sm, DIM)
            self.tc(d, cx, y0 + 39, "Go outside with a clear view of the sky", self.f_sm, DIM)

    # -- page 1 --
    def page_dash(self, d, s):
        gps, rec, u = s["gps"], s["rec"], s["units"]
        self.panel(d, (4, 22, 164, 100))
        spd, unit = u.speed(gps.ground_speed if gps.has_fix else None)
        d.text((10, 25), "SPEED", font=self.f_sm, fill=DIM)
        d.text((10, 38), spd, font=self.f_huge, fill=CYAN if gps.has_fix else DIM)
        d.text((12 + d.textlength(spd, font=self.f_huge), 60), unit, font=self.f_reg, fill=DIM)
        if gps.has_fix and gps.course is not None and not gps.stationary:
            self.tr(d, 158, 26, "{} {:.0f}°".format(compass_point(gps.course), gps.course),
                    self.f_smb, TEXT)
        pace = u.pace(gps.ground_speed) if gps.has_fix else None
        if pace:
            d.text((10, 82), "pace " + pace, font=self.f_sm, fill=DIM)

        self.panel(d, (168, 22, 316, 100))
        dv, du = u.dist(rec.distance)
        self.value(d, 174, 25, "DISTANCE", dv, du, self.f_big, GREEN)
        self.value(d, 174, 61, "TIME", fmt_dur(rec.total_time), "", self.f_big,
                   TEXT if rec.state != "idle" else DIM)

        self.panel(d, (4, 104, 316, 152))
        av = rec.distance / rec.moving_time if rec.moving_time > 5 else None
        cols = [("AVG", u.speed(av)), ("MAX", u.speed(rec.max_speed if rec.max_speed else None)),
                ("ALT", u.alt(gps.alt if gps.has_fix else None)), ("CLIMB", u.alt(rec.climb))]
        for i, (lbl, (v, un)) in enumerate(cols):
            x = 10 + i * 77
            d.text((x, 107), lbl, font=self.f_sm, fill=DIM)
            d.text((x, 120), v, font=self.f_med, fill=TEXT)
            d.text((x + d.textlength(v, font=self.f_med) + 2, 123), un, font=self.f_sm, fill=DIM)
        if rec.waypoints:
            d.text((10, 137), "{} waypoint{}".format(len(rec.waypoints), "s" if len(rec.waypoints) > 1 else ""),
                   font=self.f_sm, fill=YELLOW)

        if not gps.has_fix and rec.state == "idle":
            box = (14, 28, 306, 104)
            d.rounded_rectangle(box, radius=8, fill=(14, 16, 22), outline=YELLOW)
            self.no_gps_hint(d, s, box)
        hint = {"idle": "Enter: start recording   ←→ page   ESC quit",
                "recording": "Enter: pause   S: stop & save   W: waypoint",
                "paused": "Enter: resume   S: stop & save   W: waypoint"}[rec.state]
        self.footer(d, hint)

    # -- page 2 --
    def page_map(self, d, s):
        gps, rec, u = s["gps"], s["rec"], s["units"]
        box = (4, 22, 316, 152)
        self.panel(d, box)
        pts = [(p[0], p[1]) for p in rec.points]
        here = (gps.lat, gps.lon) if gps.has_fix else None
        allp = pts + ([here] if here else []) + [(w[0], w[1]) for w in rec.waypoints]
        if not allp:
            self.no_gps_hint(d, s, (14, 44, 306, 120))
            self.footer(d, "Your path appears here while recording")
            return
        lat0 = sum(p[0] for p in allp) / len(allp)
        kx = 111320.0 * math.cos(math.radians(lat0))
        ky = 110540.0
        xs = [p[1] * kx for p in allp]
        ys = [p[0] * ky for p in allp]
        span = max(max(xs) - min(xs), max(ys) - min(ys), 60.0)  # at least 60 m across
        cxm, cym = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        x0, y0, x1, y1 = box[0] + 14, box[1] + 10, box[2] - 14, box[3] - 18
        scale = min(x1 - x0, y1 - y0) / span
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2

        def proj(lat, lon):
            return (mx + (lon * kx - cxm) * scale, my - (lat * ky - cym) * scale)

        # grid
        for gx in range(int(x0), int(x1), 30):
            d.line((gx, y0, gx, y1), fill=(30, 35, 45))
        for gy in range(int(y0), int(y1), 30):
            d.line((x0, gy, x1, gy), fill=(30, 35, 45))
        # path (split where recording was paused: big time gaps)
        if len(pts) >= 2:
            seg = [proj(*pts[0])]
            for a, b in zip(rec.points, rec.points[1:]):
                if b[3] - a[3] > MAX_INTERVAL_S * 2:
                    if len(seg) > 1:
                        d.line(seg, fill=GREEN, width=3, joint="curve")
                    seg = []
                seg.append(proj(b[0], b[1]))
            if len(seg) > 1:
                d.line(seg, fill=GREEN, width=3, joint="curve")
        if pts:
            sx, sy = proj(*pts[0])
            d.ellipse((sx - 5, sy - 5, sx + 5, sy + 5), fill=BG, outline=GREEN, width=2)
        for lat, lon, name in rec.waypoints:
            wx, wy = proj(lat, lon)
            d.polygon([(wx, wy - 7), (wx - 5, wy + 3), (wx + 5, wy + 3)], fill=YELLOW)
        if here:
            hx, hy = proj(*here)
            if gps.course is not None and not gps.stationary:
                a = math.radians(gps.course)
                tip = (hx + 9 * math.sin(a), hy - 9 * math.cos(a))
                l = (hx + 6 * math.sin(a + 2.5), hy - 6 * math.cos(a + 2.5))
                r = (hx + 6 * math.sin(a - 2.5), hy - 6 * math.cos(a - 2.5))
                d.polygon([tip, l, (hx, hy), r], fill=CYAN)
            else:
                d.ellipse((hx - 5, hy - 5, hx + 5, hy + 5), fill=CYAN, outline=TEXT)
        # north arrow
        d.polygon([(304, 30), (299, 42), (309, 42)], fill=TEXT)
        self.tc(d, 304, 43, "N", self.f_smb, TEXT)
        # scale bar: a round number near 1/4 of the width
        target = span * 0.25
        mag = 10 ** math.floor(math.log10(target))
        nice = min((m * mag for m in (1, 2, 5, 10)), key=lambda v: abs(v - target))
        if u.imperial:
            ft_target = target * 3.28084
            if ft_target < 1500:
                magf = 10 ** math.floor(math.log10(ft_target))
                nice_ft = min((m * magf for m in (1, 2, 5, 10)), key=lambda v: abs(v - ft_target))
                nice, label = nice_ft / 3.28084, "{:.0f} ft".format(nice_ft)
            else:
                mi_t = target / 1609.344
                magm = 10 ** math.floor(math.log10(mi_t))
                nice_mi = min((m * magm for m in (1, 2, 5, 10)), key=lambda v: abs(v - mi_t))
                nice, label = nice_mi * 1609.344, "{:g} mi".format(nice_mi)
        else:
            label = u.short(nice)
        bl = nice * scale
        d.line((12, 144, 12 + bl, 144), fill=TEXT, width=2)
        d.line((12, 140, 12, 147), fill=TEXT)
        d.line((12 + bl, 140, 12 + bl, 147), fill=TEXT)
        d.text((16 + bl, 137), label, font=self.f_sm, fill=TEXT)
        dv, du = u.dist(rec.distance)
        self.tr(d, 310, 137, "{} {}".format(dv, du), self.f_smb, GREEN)
        if pts and here:
            back = haversine(pts[0][0], pts[0][1], here[0], here[1])
            self.footer(d, "Start: {} {} away".format(*u.dist(back)) if back > 30 else
                        "○ start  ▲ waypoint  ▶ you")
        else:
            self.footer(d, "○ start  ▲ waypoint  ▶ you")

    # -- page 3 --
    def page_elev(self, d, s):
        gps, rec, u = s["gps"], s["rec"], s["units"]
        self.panel(d, (4, 22, 316, 108))
        d.text((10, 25), "ELEVATION", font=self.f_sm, fill=DIM)
        av, au = u.alt(gps.alt if gps.has_fix else None)
        self.tr(d, 310, 24, "{} {}".format(av, au), self.f_med, CYAN)
        prof = rec.elev
        gbox = (12, 42, 308, 102)
        if len(prof) >= 2:
            alts = [a for _, a in prof]
            lo, hi = min(alts), max(alts)
            if hi - lo < 10:
                mid = (hi + lo) / 2
                lo, hi = mid - 5, mid + 5
            dmax = max(prof[-1][0], 1.0)
            pts = [(gbox[0] + (gbox[2] - gbox[0]) * dd / dmax,
                    gbox[3] - (gbox[3] - gbox[1]) * (a - lo) / (hi - lo)) for dd, a in prof]
            for i in range(1, 3):
                gy = gbox[3] - (gbox[3] - gbox[1]) * i / 3
                d.line((gbox[0], gy, gbox[2], gy), fill=GRID)
            d.polygon([(gbox[0], gbox[3])] + pts + [(pts[-1][0], gbox[3])], fill=ORANGE_FILL)
            d.line(pts, fill=ORANGE, width=2)
            d.text((gbox[0] + 2, gbox[1]), "{} {}".format(*u.alt(hi)), font=self.f_sm, fill=DIM)
            d.text((gbox[0] + 2, gbox[3] - 11), "{} {}".format(*u.alt(lo)), font=self.f_sm, fill=DIM)
        else:
            self.tc(d, 160, 62, "Profile builds up while you record", self.f_reg, DIM)
        self.panel(d, (4, 112, 316, 152))
        alts = [a for _, a in prof] or [None]
        vals = [("CLIMB", u.alt(rec.climb), GREEN), ("DESCENT", u.alt(rec.descent), ORANGE),
                ("LOWEST", u.alt(min(alts) if prof else None), TEXT),
                ("HIGHEST", u.alt(max(alts) if prof else None), TEXT)]
        for i, (lbl, (v, un), col) in enumerate(vals):
            x = 10 + i * 77
            d.text((x, 115), lbl, font=self.f_sm, fill=DIM)
            d.text((x, 128), v, font=self.f_med, fill=col)
            d.text((x + d.textlength(v, font=self.f_med) + 2, 131), un, font=self.f_sm, fill=DIM)
        self.footer(d, "GPS altitude is ± ~10 m   ←→ page")

    # -- page 4 --
    def page_sats(self, d, s):
        gps = s["gps"]
        self.panel(d, (4, 22, 316, 84))
        sats = sorted(gps.sats.values(), key=lambda v: (-(v["snr"] or 0)))[:16]
        if sats:
            n = len(sats)
            bw = max(8, min(16, (300 - 4 * n) // max(1, n)))
            for i, sat in enumerate(sats):
                x = 12 + i * (bw + 3)
                snr = sat["snr"] or 0
                h = int(34 * min(snr, 50) / 50)
                used = sat["prn"] in gps.used_prns
                col = GREEN if used and snr else (CYAN if snr else GRID)
                d.rectangle((x, 72 - h, x + bw, 72), fill=col)
                d.rectangle((x, 72 - 34, x + bw, 72), outline=GRID)
                lbl = sat["prn"][-2:]
                self.tc(d, x + bw / 2, 73, lbl, self.f_sm, DIM)
        else:
            self.tc(d, 160, 44, "No satellites reported yet", self.f_reg, DIM)
        d.text((10, 24), "SIGNAL", font=self.f_sm, fill=DIM)
        lx = 310 - d.textlength("seen", font=self.f_sm)
        d.text((lx, 24), "seen", font=self.f_sm, fill=DIM)
        d.rectangle((lx - 10, 27, lx - 4, 33), fill=CYAN)
        lx2 = lx - 16 - d.textlength("used", font=self.f_sm)
        d.text((lx2, 24), "used", font=self.f_sm, fill=DIM)
        d.rectangle((lx2 - 10, 27, lx2 - 4, 33), fill=GREEN)

        self.panel(d, (4, 88, 316, 152))
        fix = {1: "no fix", 2: "2D fix", 3: "3D fix"}.get(gps.fix_mode, "?") if gps.has_fix else "no fix"
        d.text((10, 91), fix.upper(), font=self.f_med, fill=GREEN if gps.has_fix else YELLOW)
        d.text((80, 93), "{} used / {} seen".format(gps.sats_used, len(gps.sats)), font=self.f_sm, fill=TEXT)
        acc = "±{:.0f} m".format(gps.hdop * 2.5) if gps.hdop else "--"
        self.tr(d, 310, 93, "HDOP {}  {}".format("{:.1f}".format(gps.hdop) if gps.hdop else "--", acc),
                self.f_sm, TEXT)
        if gps.lat is not None:
            ns, ew = "N" if gps.lat >= 0 else "S", "E" if gps.lon >= 0 else "W"
            d.text((10, 110), "{:.6f}°{}  {:.6f}°{}".format(abs(gps.lat), ns, abs(gps.lon), ew),
                   font=self.f_reg, fill=TEXT)
        else:
            d.text((10, 110), "position unknown", font=self.f_reg, fill=DIM)
        if gps.first_fix_at:
            d.text((10, 129), "first fix in {}".format(fmt_dur(gps.first_fix_at - gps.started)),
                   font=self.f_sm, fill=DIM)
        utc = gps.utc.strftime("%H:%M:%S UTC") if gps.utc else ""
        self.tr(d, 310, 129, utc, self.f_sm, DIM)
        if not gps.sentences or s["error"]:
            box = (14, 28, 306, 104)
            d.rounded_rectangle(box, radius=8, fill=(14, 16, 22), outline=YELLOW)
            self.no_gps_hint(d, s, box)
        self.footer(d, "{} sentences  {} bad   ←→ page".format(gps.sentences, gps.bad_checksums))

    # -- page 5 --
    def page_files(self, d, s, ui):
        u = s["units"]
        trails = s["trails"]
        self.panel(d, (4, 22, 316, 152))
        d.text((10, 25), "SAVED TRAILS", font=self.f_smb, fill=DIM)
        self.tr(d, 310, 25, "~/Trails  ({})".format(len(trails)), self.f_sm, DIM)
        if not trails:
            self.tc(d, 160, 70, "No trails yet", self.f_med, DIM)
            self.tc(d, 160, 90, "Press Enter on the Dashboard to record", self.f_sm, DIM)
        else:
            sel = min(ui["sel"], len(trails) - 1)
            top = max(0, min(sel - 2, len(trails) - 5))
            y = 39
            for i in range(top, min(top + 5, len(trails))):
                t = trails[i]
                if i == sel:
                    d.rounded_rectangle((7, y - 1, 313, y + 20), radius=4, fill=SEL)
                d.text((12, y + 3), t["name"][:20], font=self.f_reg, fill=TEXT)
                dv, du = u.dist(t["dist"])
                self.tr(d, 250, y + 3, "{} {}".format(dv, du), self.f_reg, GREEN)
                self.tr(d, 310, y + 4, fmt_dur(t["dur"]) if t["dur"] else "--", self.f_sm, DIM)
                y += 22
        self.footer(d, "↑↓ select   D delete   files are GPX in ~/Trails")
        if ui.get("confirm") == "delete" and trails:
            t = trails[min(ui["sel"], len(trails) - 1)]
            self.dialog(d, "Delete this trail?", t["name"], "Y = delete     N / ESC = keep")

    def dialog(self, d, title, line, keys, color=RED):
        box = (36, 50, 284, 122)
        d.rounded_rectangle(box, radius=8, fill=(14, 16, 22), outline=color, width=2)
        self.tc(d, 160, 58, title, self.f_med, TEXT)
        self.tc(d, 160, 80, line, self.f_reg, YELLOW)
        self.tc(d, 160, 101, keys, self.f_sm, DIM)

    def render(self, page, s, ui):
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        self.header(d, page, s)
        if page == P_DASH:
            self.page_dash(d, s)
        elif page == P_MAP:
            self.page_map(d, s)
        elif page == P_ELEV:
            self.page_elev(d, s)
        elif page == P_SATS:
            self.page_sats(d, s)
        else:
            self.page_files(d, s, ui)
        if ui.get("confirm") == "stop":
            dv, du = s["units"].dist(s["rec"].distance)
            self.dialog(d, "Stop and save this trail?", "{} {} · {}".format(
                dv, du, fmt_dur(s["rec"].total_time)), "Y = save     N / ESC = keep recording", GREEN)
        elif ui.get("confirm") == "quit":
            self.dialog(d, "Save trail and quit?", "Recording is still running",
                        "Y = save & quit     N / ESC = keep going", YELLOW)
        if ui.get("toast") and time.monotonic() < ui.get("toast_until", 0):
            msg, col = ui["toast"]
            w = d.textlength(msg, font=self.f_reg) + 20
            d.rounded_rectangle((160 - w / 2, 128, 160 + w / 2, 150), radius=6,
                                fill=(14, 16, 22), outline=col)
            self.tc(d, 160, 132, msg, self.f_reg, col)
        return img


# --------------------------------------------------------------------------
# Framebuffer + keyboard (same approach as SysMon)
# --------------------------------------------------------------------------
class Framebuffer:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR)
        vbuf = bytearray(160)
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vbuf)
        self.xres, self.yres = struct.unpack_from("2I", vbuf, 0)
        self.bpp = struct.unpack_from("I", vbuf, 24)[0]
        fix_fmt = "16sL" + "I" * 6 + "HHHI"
        fbuf = bytearray(struct.calcsize(fix_fmt))
        fcntl.ioctl(self.fd, FBIOGET_FSCREENINFO, fbuf)
        fields = struct.unpack(fix_fmt, fbuf)
        self.smem_len, self.line_length = fields[2], fields[8]
        if self.bpp not in (16, 32):
            os.close(self.fd)
            raise RuntimeError("unsupported framebuffer depth: {} bpp".format(self.bpp))
        self.mm = mmap.mmap(self.fd, self.smem_len, mmap.MAP_SHARED,
                            mmap.PROT_READ | mmap.PROT_WRITE)

    def _convert(self, img):
        if self.bpp == 32:
            return img.convert("RGBA").tobytes("raw", "BGRA")
        if HAS_NUMPY:
            a = np.asarray(img, dtype=np.uint16)
            v = ((a[..., 0] >> 3) << 11) | ((a[..., 1] >> 2) << 5) | (a[..., 2] >> 3)
            return v.astype("<u2").tobytes()
        out = bytearray()
        for r, g, b in img.getdata():
            v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            out += bytes((v & 0xFF, v >> 8))
        return bytes(out)

    def show(self, img):
        if (self.xres, self.yres) != img.size:
            if (self.yres, self.xres) == img.size:
                img = img.rotate(90, expand=True)
            else:
                img = img.resize((self.xres, self.yres))
        data = self._convert(img)
        row = self.xres * self.bpp // 8
        if row == self.line_length:
            self.mm.seek(0)
            self.mm.write(data)
        else:
            for y in range(self.yres):
                self.mm.seek(y * self.line_length)
                self.mm.write(data[y * row:(y + 1) * row])

    def clear(self):
        self.mm.seek(0)
        self.mm.write(b"\x00" * min(self.smem_len, self.line_length * self.yres))

    def close(self):
        try:
            self.mm.close()
        finally:
            os.close(self.fd)


EVENT_FMT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FMT)


class Keys:
    def __init__(self):
        self.fds = []
        for path in sorted(glob.glob("/dev/input/event*")):
            try:
                self.fds.append(os.open(path, os.O_RDONLY | os.O_NONBLOCK))
            except OSError:
                pass

    def poll(self, timeout, extra_fds=()):
        fds = self.fds + [f for f in extra_fds if f is not None]
        if not fds:
            time.sleep(timeout)
            return []
        try:
            r, _, _ = select.select(fds, [], [], timeout)
        except (OSError, ValueError):
            return []
        pressed = []
        for fd in r:
            if fd not in self.fds:
                continue
            try:
                data = os.read(fd, EVENT_SIZE * 32)
            except (BlockingIOError, OSError):
                continue
            for off in range(0, len(data) - EVENT_SIZE + 1, EVENT_SIZE):
                _, _, etype, code, value = struct.unpack_from(EVENT_FMT, data, off)
                if etype == EV_KEY and value in (1, 2):
                    pressed.append((code, value))
        return pressed

    def close(self):
        for fd in self.fds:
            try:
                os.close(fd)
            except OSError:
                pass


# --------------------------------------------------------------------------
# App controller
# --------------------------------------------------------------------------
class App:
    def __init__(self):
        cfg = load_config()
        self.cfg = cfg
        self.gps = GpsState()
        self.rec = Recorder()
        self.units = Units(cfg["imperial"])
        self.page = P_DASH
        self.ui = {"sel": 0, "confirm": None, "toast": None, "toast_until": 0}
        self.error = None
        self.trails = list_trails()
        recovered = recover_unfinished()
        if recovered:
            self.trails = list_trails()
            self.toast("Recovered {} unfinished trail{}".format(
                len(recovered), "s" if len(recovered) > 1 else ""), GREEN, 4)

    def state(self):
        return {"gps": self.gps, "rec": self.rec, "units": self.units,
                "trails": self.trails, "error": self.error}

    def toast(self, msg, color=CYAN, secs=2.5):
        self.ui["toast"] = (msg, color)
        self.ui["toast_until"] = time.monotonic() + secs

    def on_line(self, line):
        if self.gps.feed(line) == "fix":
            self.rec.update(self.gps)

    def save(self):
        path = self.rec.stop()
        if path:
            self.trails = list_trails()
            dv, du = self.units.dist(self.rec.distance)
            self.toast("Saved {} ({} {})".format(os.path.basename(path), dv, du), GREEN, 4)
        else:
            self.toast("Nothing recorded — no GPS points", YELLOW, 3)
        return path

    def handle(self, code, repeat):
        """Return False to quit."""
        ui = self.ui
        c = ui["confirm"]
        if c:
            if code == KEY_Y and not repeat:
                ui["confirm"] = None
                if c == "stop":
                    self.save()
                elif c == "quit":
                    self.save()
                    return False
                elif c == "delete" and self.trails:
                    t = self.trails[min(ui["sel"], len(self.trails) - 1)]
                    try:
                        os.remove(t["path"])
                        self.toast("Deleted " + t["name"], DIM)
                    except OSError as e:
                        self.toast("Delete failed: " + e.strerror, RED)
                    self.trails = list_trails()
                    ui["sel"] = max(0, min(ui["sel"], len(self.trails) - 1))
            elif code in (KEY_N, KEY_ESC, KEY_Q):
                ui["confirm"] = None
            return True
        if repeat and code not in (KEY_UP, KEY_DOWN, KEY_SEMICOLON, KEY_DOT):
            return True
        if code in (KEY_ESC, KEY_Q):
            if self.rec.state != "idle":
                ui["confirm"] = "quit"
                return True
            return False
        if code in (KEY_RIGHT, KEY_TAB, KEY_SLASH):
            self.set_page(self.page + 1)
        elif code in (KEY_LEFT, KEY_COMMA):
            self.set_page(self.page - 1)
        elif code in NUM_KEYS:
            self.set_page(NUM_KEYS[code])
        elif code in (KEY_ENTER, KEY_SPACE) and self.page != P_FILES:
            if self.rec.state == "recording":
                self.rec.pause()
                self.toast("Paused", YELLOW)
            else:
                was_idle = self.rec.state == "idle"
                try:
                    self.rec.start(self.gps)
                except OSError as e:
                    self.toast("Can't write ~/Trails: " + e.strerror, RED, 4)
                    return True
                if was_idle and not self.gps.has_fix:
                    self.toast("Recording — waiting for GPS fix", YELLOW, 3)
                else:
                    self.toast("Recording" if was_idle else "Resumed", GREEN)
        elif code == KEY_S and self.rec.state != "idle":
            ui["confirm"] = "stop"
        elif code == KEY_W:
            if self.rec.state == "idle":
                self.toast("Start recording to mark waypoints", DIM)
            else:
                name = self.rec.add_waypoint(self.gps)
                self.toast("Waypoint {} marked".format(name) if name else "No GPS fix yet",
                           YELLOW if name else RED)
        elif code == KEY_U:
            self.units.imperial = not self.units.imperial
            self.cfg["imperial"] = self.units.imperial
            save_config(self.cfg)
            self.toast("Units: " + ("miles / feet" if self.units.imperial else "km / metres"))
        elif self.page == P_FILES:
            n = len(self.trails)
            if code in (KEY_UP, KEY_SEMICOLON):
                ui["sel"] = max(0, ui["sel"] - 1)
            elif code in (KEY_DOWN, KEY_DOT):
                ui["sel"] = min(max(0, n - 1), ui["sel"] + 1)
            elif code == KEY_D and n:
                ui["confirm"] = "delete"
        return True

    def set_page(self, p):
        self.page = p % len(PAGES)
        self.ui["confirm"] = None
        if self.page == P_FILES:
            self.trails = list_trails()


# --------------------------------------------------------------------------
# Preview with sample data
# --------------------------------------------------------------------------
def mock_app(tmpdir):
    global TRAIL_DIR
    TRAIL_DIR = tmpdir
    app = App()
    g = app.gps
    g.valid, g.fix_quality, g.fix_mode, g.sats_used = True, 1, 3, 9
    g.hdop, g.alt, g.speed, g.course = 0.9, 212.0, 1.42, 68.0
    g.ground_speed, g.stationary = 1.42, False
    g.utc = datetime(2026, 11, 14, 21, 41, 7, tzinfo=timezone.utc)
    g.sentences, g.last_sentence, g.last_fix_time = 4812, time.monotonic(), time.monotonic()
    g.first_fix_at = g.started + 27
    for i, (prn, snr, used) in enumerate([("12", 44, 1), ("25", 41, 1), ("05", 38, 1), ("29", 36, 1),
                                          ("18", 33, 1), ("31", 30, 1), ("11", 27, 1), ("20", 24, 1),
                                          ("09", 21, 1), ("02", 16, 0), ("14", 12, 0), ("23", 0, 0)]):
        g.sats[("GP", prn)] = {"sys": "GP", "prn": prn, "elev": 40, "az": i * 30, "snr": snr, "t": time.monotonic()}
        if used:
            g.used_prns.add(prn)
    r = app.rec
    r.state = "recording"
    r.seg_start = time.monotonic() - (47 * 60 + 12)
    lat0, lon0 = 41.8806, -87.6742
    t = time.monotonic() - 3000
    lat, lon, alt, dist = lat0, lon0, 205.0, 0.0
    for i in range(240):
        a = i / 240 * 2 * math.pi
        nlat = lat0 + 0.0042 * math.sin(a) + 0.0006 * math.sin(a * 5)
        nlon = lon0 + 0.0065 * (1 - math.cos(a)) + 0.0004 * math.cos(a * 3)
        if i:
            dist += haversine(lat, lon, nlat, nlon)
        lat, lon = nlat, nlon
        alt = 205 + 14 * math.sin(a * 1.5) + 4 * math.sin(a * 7)
        r.points.append((lat, lon, alt, t + i * 11))
        r.elev.append((dist, alt))
    r.distance, r.moving_time, r.max_speed = dist, 2650.0, 2.6
    r.climb, r.descent = 31.0, 22.0
    r.waypoints = [(r.points[70][0], r.points[70][1], "WP1"), (r.points[160][0], r.points[160][1], "WP2")]
    g.lat, g.lon = r.points[-1][0], r.points[-1][1]
    app.trails = [
        {"path": "", "name": "2026-11-14_1453", "dist": 3712.0, "dur": 2832, "points": 240, "wpts": 2, "size": 0},
        {"path": "", "name": "2026-11-12_0710", "dist": 5240.0, "dur": 3965, "points": 330, "wpts": 0, "size": 0},
        {"path": "", "name": "2026-11-09_1622", "dist": 1290.0, "dur": 1104, "points": 98, "wpts": 1, "size": 0},
        {"path": "", "name": "2026-11-08_1830", "dist": 8020.0, "dur": 6011, "points": 520, "wpts": 3, "size": 0},
    ]
    return app


def preview(path, page, mock):
    import tempfile
    if mock:
        app = mock_app(tempfile.mkdtemp())
    else:
        app = App()
    app.page = page
    Renderer().render(page, app.state(), app.ui).save(path)
    print("wrote", path)
    return 0


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    args = sys.argv[1:]
    if args and args[0] == "--preview":
        out = args[1] if len(args) > 1 else "preview.png"
        page = int(args[2]) if len(args) > 2 and args[2].isdigit() else 0
        return preview(out, page, "--mock" in args)
    if args and args[0] in ("--version", "-V"):
        print("Trail Recorder", VERSION)
        return 0

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGHUP, _stop)

    try:
        fb = Framebuffer(FB_DEVICE)
    except Exception as e:
        print("Trail: cannot open framebuffer {}: {}".format(FB_DEVICE, e), file=sys.stderr)
        return 3

    app = App()
    power = CapPower()
    port = GpsPort()
    if not power.enable():
        app.error = power.error
    elif not port.open():
        app.error = port.error
    keys = Keys()
    renderer = Renderer()
    try:
        next_frame = 0.0
        while _running:
            for line in port.lines():
                app.on_line(line)
            now = time.monotonic()
            if now >= next_frame:
                fb.show(renderer.render(app.page, app.state(), app.ui))
                next_frame = now + (0.5 if app.rec.state == "recording" else 1.0)
            for code, value in keys.poll(max(0.02, min(0.2, next_frame - time.monotonic())),
                                         extra_fds=[port.fd]):
                if not app.handle(code, value == 2):
                    return 0
                next_frame = 0.0
    finally:
        # Always keep the trail: a launcher close or SIGTERM saves it too.
        if app.rec.state != "idle":
            try:
                app.rec.stop()
            except Exception:
                pass
        keys.close()
        port.close()
        power.restore()
        try:
            fb.clear()
        except Exception:
            pass
        fb.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
