#!/usr/bin/env python3
"""
ORCP Reference Simulator — conformant to ORCP v1.1.

Creates a virtual serial port (PTY) — or a WebSocket (`--ws`) — that emulates an
ORCP controller. Connect test scripts, client libraries, or ROS 2 nodes and
develop against the protocol with no hardware.

Conformance level (§9) is selectable with --level (1/2/3, default 2).

Implementation profiles (--profile) describe a specific controller's ORCP
surface (identity, config-key set, vendor command modes, push events) layered
over the generic v1.1 core:

    --profile base   generic, fully-conformant ORCP v1.1 controller  [default]
    --profile mc1    First Layer Robotics MC1 (42-key surface, ! WARN AUX5V, ...)

Only `base` (the standard reference, which must not drift) is built into the
core. Every other profile — including MC1 — is a JSON data file: bundled ones
live in profiles/ and are discovered by name; private ones are loaded with
--profile-file. So a vendor adds a device with a data file, not code. A pip
entry-point plugin mechanism — for profiles that need custom behaviour, not just
a declarative surface — is a future direction.

Usage:
    python3 -m orcp_sim                       # base, Level 2, auto PTY
    python3 -m orcp_sim --profile mc1         # emulate an MC1 (bundled profile)
    python3 -m orcp_sim --profile-file acme.json   # emulate a vendor device
    python3 orcp_sim.py --ws 8765             # serve over WebSocket
    python3 orcp_sim.py --link /tmp/orcp      # also symlink the PTY
"""

import os
import sys
import pty
import json
import select
import time
import math
import argparse
import signal
import tempfile

# ---------------------------------------------------------------------------
# Protocol-level constants (shared by all profiles)
# ---------------------------------------------------------------------------

PROTO_VERSION = "1.1"           # → proto=ORCP/1.1
SIM_UUID      = "00SIMULATED00FF"

MAX_MOTOR_RADS   = 20.9         # rad/s at full duty (physical model)
MAX_LIN_VEL      = 1.0          # m/s   default CMD_VEL clamp (if no kin.max_v)
MAX_ANG_VEL      = 3.0          # rad/s default CMD_VEL clamp (if no kin.max_w)
VEL_FILTER_ALPHA = 0.2

CONTROL_HZ = 100
CONTROL_DT = 1.0 / CONTROL_HZ
LINE_MAX   = 256                # §2.2 command-line cap

# Command → minimum conformance level that implements it (§9).
COMMAND_LEVEL = {
    "PING": 1, "CMD_VEL": 1, "WHEEL": 1, "STOP": 1,
    "STATUS": 1, "ENABLE": 1, "PRESET": 1,
    "INFO": 2, "HB": 2, "STREAM": 2,
    "GET": 2, "SET": 2, "SAVE": 2, "LOAD": 2, "DEFAULTS": 2,
}

# ---------------------------------------------------------------------------
# Implementation profiles
#
# A profile is a declarative description of a controller's ORCP surface:
#   identity      — INFO fields (hw, fw, level, optional vendor/model/info_extra)
#   config        — ordered [(name, default, min, max)] (drives GET ALL order)
#   int_keys      — keys rendered as integers (others as %.3f)
#   wheel_modes   — vendor WHEEL modes beyond the default rad/s (e.g. DUTY)
#   warns         — ! WARN <type> events this device emits
#   battery       — STATUS battery field rendering: "percent" or "band"
#   aux5v         — whether a 5 V aux rail (and ! WARN AUX5V) is modelled
# ---------------------------------------------------------------------------

BASE_PROFILE = {
    "name": "base",
    "identity": {"hw": "ORCP-SIM", "vendor": "ORCP-Project", "model": "reference-sim",
                 "level": 2, "fw": "1.1.0", "info_extra": {}},
    "config": [
        # Standard ORCP v1.1 §7 parameter set.
        ("kin.counts_per_rev", 1996,  0,     10000),
        ("kin.wheel_radius",   0.049, 0.01,  0.5),
        ("kin.track_width",    0.175, 0.05,  1.0),
        ("kin.max_accel",      10.2,  0.0,   100.0),
        ("pid.kp",             0.05,  0.0,   10.0),
        ("pid.ki",             0.3,   0.0,   50.0),
        ("pid.kd",             0.0,   0.0,   1.0),
        ("batt.full_v",        12.6,  5.0,   30.0),
        ("batt.low_v",         10.2,  5.0,   30.0),
        ("batt.critical_v",    9.6,   5.0,   30.0),
        ("slow.duty_limit",    0.30,  0.05,  1.0),
        ("normal.duty_limit",  0.90,  0.05,  1.0),
        ("slow.timeout_ms",    0,     0,     60000),
        ("normal.timeout_ms",  250,   0,     60000),
        ("hb.timeout_ms",      500,   10,    60000),
    ],
    "int_keys": {"kin.counts_per_rev", "slow.timeout_ms", "normal.timeout_ms", "hb.timeout_ms"},
    "wheel_modes": ["DUTY"],
    "warns": ["BATT"],
    "battery": "percent",
    "aux5v": False,
}

# Vendor profiles (other than `base`) ship as JSON data files in profiles/ and
# are discovered into PROFILES below. MC1 is profiles/mc1.json — the first
# worked example of the bundled tier; a vendor adds their own the same way.

# Keys the core control loop reads unconditionally — a profile that drops any
# of these would crash mid-run, so load_profile_file rejects it up front with a
# clear message. (slow.timeout_ms is intentionally NOT here: it's optional, read
# with a default of 0, so a device like the MC1 — whose SLOW timeout is a fixed
# constant, not a settable key — is fully valid without it.)
_CORE_REQUIRED_KEYS = {
    "pid.kp", "pid.ki", "pid.kd",
    "slow.duty_limit", "normal.duty_limit", "normal.timeout_ms", "hb.timeout_ms",
    "kin.counts_per_rev", "kin.wheel_radius", "kin.track_width", "kin.max_accel",
    "batt.full_v", "batt.low_v", "batt.critical_v",
}
# Additionally required only when the profile models a 5 V aux rail.
_AUX5V_REQUIRED_KEYS = {"aux5v.warn_amps", "aux5v.warn_volts"}


def load_profile_file(path):
    """Load a vendor profile from a JSON file (``--profile-file`` or bundled).

    The JSON mirrors BASE_PROFILE's shape. Two conveniences bridge JSON and the
    Python schema: the ``int_keys`` and ``warns`` fields are JSON arrays here
    and coerced to sets; ``config`` rows are ``[name, default, min, max]`` arrays
    and coerced to tuples. Optional behaviour flags default to a generic v1.1
    controller.

    Validation is deliberately strict and gives a clear message on the common
    mistakes (missing field, wrong config-row arity, dropping a key the core
    relies on) rather than failing later with a KeyError.
    """
    with open(path) as f:
        data = json.load(f)

    for field in ("name", "identity", "config"):
        if field not in data:
            raise ValueError(f"profile file {path}: missing required field '{field}'")

    ident = data["identity"]
    for field in ("hw", "fw", "level"):
        if field not in ident:
            raise ValueError(f"profile file {path}: identity must include '{field}'")

    cfg = []
    for row in data["config"]:
        if len(row) != 4:
            raise ValueError(
                f"profile file {path}: each config row must be "
                f"[name, default, min, max]; got {row!r}")
        name, default, mn, mx = row
        cfg.append((name, default, mn, mx))
    data["config"] = cfg

    data["int_keys"] = set(data.get("int_keys", []))
    data["warns"] = set(data.get("warns", []))
    data.setdefault("wheel_modes", [])
    data.setdefault("battery", "percent")
    data.setdefault("aux5v", False)

    required = set(_CORE_REQUIRED_KEYS)
    if data.get("aux5v"):
        required |= _AUX5V_REQUIRED_KEYS
    missing = required - {name for name, *_ in cfg}
    if missing:
        raise ValueError(
            f"profile file {path}: config is missing keys the core requires: "
            f"{', '.join(sorted(missing))}")

    return data


# ---------------------------------------------------------------------------
# Profile registry: the built-in `base` standard reference, plus every bundled
# vendor profile discovered from profiles/*.json. A vendor gets a first-class
# `--profile <name>` by contributing a profiles/<name>.json data file — exactly
# how MC1 ships (see the README "Creating a profile" section).
# ---------------------------------------------------------------------------
_PROFILE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")


def _discover_profiles():
    found = {"base": BASE_PROFILE}
    if os.path.isdir(_PROFILE_DIR):
        for fn in sorted(os.listdir(_PROFILE_DIR)):
            if fn.endswith(".json"):
                prof = load_profile_file(os.path.join(_PROFILE_DIR, fn))
                found[prof["name"]] = prof
    return found


PROFILES = _discover_profiles()
# Back-compat alias: MC1 used to be an in-module dict; it now ships as
# profiles/mc1.json but stays importable as orcp_sim.MC1_PROFILE.
MC1_PROFILE = PROFILES.get("mc1")

# ---------------------------------------------------------------------------
# PID controller
# ---------------------------------------------------------------------------

class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.integral = 0.0
        self.prev_measurement = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_measurement = 0.0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        p_term = self.kp * error
        self.integral += self.ki * error * dt
        max_integral = max(self.limit - abs(p_term), 0.0)
        self.integral = max(-max_integral, min(max_integral, self.integral))
        d_term = -self.kd * (measurement - self.prev_measurement) / dt if dt > 0 else 0.0
        self.prev_measurement = measurement
        output = p_term + self.integral + d_term
        return max(-self.limit, min(self.limit, output))

# ---------------------------------------------------------------------------
# Motor physics
# ---------------------------------------------------------------------------

class MotorSim:
    """First-order motor model: duty -> velocity with a time constant."""
    def __init__(self):
        self.velocity = 0.0
        self.duty = 0.0
        self.counts = 0
        self.delta_counts = 0
        self.filtered_vel = 0.0
        self.tau = 0.05

    def step(self, duty, dt, counts_per_rev, alpha):
        target_vel = duty * MAX_MOTOR_RADS
        a = dt / (self.tau + dt)
        self.velocity += a * (target_vel - self.velocity)
        if counts_per_rev > 0:
            rads_this_tick = self.velocity * dt
            counts_float = rads_this_tick * counts_per_rev / (2.0 * math.pi)
            self.delta_counts = int(round(counts_float))
            self.counts += self.delta_counts
            raw_vel = self.delta_counts * (2.0 * math.pi) / (counts_per_rev * dt)
        else:
            self.delta_counts = 0
            raw_vel = self.velocity
        self.filtered_vel += alpha * (raw_vel - self.filtered_vel)

# ---------------------------------------------------------------------------
# ORCP simulator
# ---------------------------------------------------------------------------

class ORCPSim:
    def __init__(self, profile=None, level=None, config_file=None, vbat=12.4,
                 estop=False, aux5v_amps=None, aux5v_volts=None):
        self.profile = profile or BASE_PROFILE
        ident = self.profile["identity"]
        self.identity = ident
        self.level = level if level is not None else ident["level"]
        self.config_file = config_file

        # Config schema from the profile.
        self.cfg = {name: default for name, default, _, _ in self.profile["config"]}
        self.ranges = {name: (mn, mx) for name, _, mn, mx in self.profile["config"]}
        self.int_keys = set(self.profile["int_keys"])
        self.wheel_modes = {"VEL"} | {m.upper() for m in self.profile.get("wheel_modes", [])}
        self.warns = set(self.profile.get("warns", []))
        self.battery_display = self.profile.get("battery", "percent")
        self.has_aux5v = self.profile.get("aux5v", False)

        self.motor_l = MotorSim()
        self.motor_r = MotorSim()

        self.mode = "IDLE"
        self.target_l = self.target_r = 0.0
        self.ramped_target_l = self.ramped_target_r = 0.0

        self.pid_l = PID(self.cfg["pid.kp"], self.cfg["pid.ki"], self.cfg["pid.kd"], 0.30)
        self.pid_r = PID(self.cfg["pid.kp"], self.cfg["pid.ki"], self.cfg["pid.kd"], 0.30)
        self.duty_limit = self.cfg["slow.duty_limit"]

        # Safety
        self.preset = "SLOW"
        self.enabled = True
        self.fault = "OK"
        self.latched = False
        self.estop = estop
        self.last_motion_time = 0.0
        self.last_hb_time = 0.0
        self.motion_active = False

        # Battery
        self.vbat = vbat
        self.battery_pct = self._estimate_pct(vbat)
        self.batt_warned = False

        # Aux 5 V rail (MC1-style profiles)
        self.aux5v_i = 1.0 if aux5v_amps is None else aux5v_amps
        self.aux5v_v = 5.0 if aux5v_volts is None else aux5v_volts
        self.aux5v_warned = False

        # Streaming + async push
        self.stream_active = False
        self.stream_rate = 10
        self.pending_push = []

        self.boot_time = time.monotonic()
        self.tick_count = 0

        self._apply_config()
        self._apply_preset("SLOW")

    # -- helpers --

    def millis(self):
        return int((time.monotonic() - self.boot_time) * 1000)

    def _fmt(self, key, val):
        if key in self.int_keys:
            return str(int(round(val)))
        return f"{float(val):.3f}"

    def _estimate_pct(self, vbat):
        if vbat <= 2.0:
            return 0
        lo, hi = self.cfg["batt.critical_v"], self.cfg["batt.full_v"]
        return max(0, min(100, int((vbat - lo) / (hi - lo) * 100)))

    def _battery_band(self):
        if self.vbat <= 2.0:
            return "NONE"
        if self.vbat < self.cfg["batt.critical_v"]:
            return "CRITICAL"
        if self.vbat < self.cfg["batt.low_v"]:
            return "LOW"
        return "OK"

    def _battery_field(self):
        return self._battery_band() if self.battery_display == "band" else f"{self.battery_pct}%"

    def _batt_critical(self):
        return 2.0 < self.vbat < self.cfg["batt.critical_v"]

    def _hb_required(self):
        return self.level >= 2 and self.preset == "NORMAL"

    # -- config application --

    def _apply_config(self):
        self.pid_l.kp = self.pid_r.kp = self.cfg["pid.kp"]
        self.pid_l.ki = self.pid_r.ki = self.cfg["pid.ki"]
        self.pid_l.kd = self.pid_r.kd = self.cfg["pid.kd"]
        self.duty_limit = (self.cfg["slow.duty_limit"] if self.preset == "SLOW"
                           else self.cfg["normal.duty_limit"])
        self.pid_l.limit = self.pid_r.limit = self.duty_limit

    def _apply_preset(self, preset):
        self.preset = preset
        self._stop_motors("BRAKE")
        self.fault = "OK"
        self.latched = False
        self.motion_active = False
        if preset == "SLOW":
            self.duty_limit = self.cfg["slow.duty_limit"]
            self.enabled = True
        else:
            self.duty_limit = self.cfg["normal.duty_limit"]
            self.enabled = False
        self.pid_l.limit = self.pid_r.limit = self.duty_limit

    def _stop_motors(self, stop_mode="BRAKE"):
        self.mode = "IDLE"
        self.target_l = self.target_r = 0.0
        self.ramped_target_l = self.ramped_target_r = 0.0
        self.motor_l.duty = self.motor_r.duty = 0.0
        self.pid_l.reset()
        self.pid_r.reset()
        if stop_mode != "COAST":
            self.motor_l.velocity = self.motor_r.velocity = 0.0

    # -- safety --

    def _can_move(self):
        if self.estop:
            return False, "ESTOP"
        if self.latched:
            return False, self.fault
        if self._batt_critical():
            return False, "LOWBATT"
        if self.preset == "NORMAL" and not self.enabled:
            return False, "NOT_ENABLED"
        return True, "OK"

    def _raise_fault(self, code):
        self.fault = code
        self.latched = True
        self.motion_active = False
        self._stop_motors("BRAKE")
        self.pending_push.append(f"! FAULT {code}")

    def _check_safety(self):
        now = time.monotonic()
        if self.estop and self.fault != "ESTOP":
            self._raise_fault("ESTOP")
            return
        if self.latched or self.fault == "ESTOP":
            return
        if self._batt_critical():
            self._raise_fault("LOWBATT")
            return
        if self.preset == "NORMAL" and self.motion_active:
            if self._hb_required() and (now - self.last_hb_time) > self.cfg["hb.timeout_ms"] / 1000.0:
                self._raise_fault("HEARTBEAT")
                return
            to = self.cfg["normal.timeout_ms"] / 1000.0
            if to > 0 and (now - self.last_motion_time) > to:
                self._raise_fault("TIMEOUT")
                return
        if self.preset == "NORMAL" and not self.enabled:
            self.fault = "NOT_ENABLED"
        else:
            self.fault = "OK"

    def _check_pushes(self):
        # Battery warning (§6.2).
        if "BATT" in self.warns:
            low, crit = self.cfg["batt.low_v"], self.cfg["batt.critical_v"]
            if self.level >= 2 and crit < self.vbat < low and not self.batt_warned:
                self.pending_push.append(
                    f"! WARN BATT level={self._battery_band()} "
                    f"vbat={self.vbat:.3f} battery={self.battery_pct}%")
                self.batt_warned = True
            elif self.vbat >= low:
                self.batt_warned = False
        # 5 V aux rail warning (vendor extension, e.g. MC1).
        if self.has_aux5v and "AUX5V" in self.warns and self.level >= 2:
            over = (self.aux5v_i > self.cfg["aux5v.warn_amps"]
                    or self.aux5v_v < self.cfg["aux5v.warn_volts"])
            if over and not self.aux5v_warned:
                self.pending_push.append(
                    f"! WARN AUX5V state=warn i={self.aux5v_i:.3f} v={self.aux5v_v:.3f}")
                self.aux5v_warned = True
            elif not over and self.aux5v_warned:
                self.pending_push.append(
                    f"! WARN AUX5V state=ok i={self.aux5v_i:.3f} v={self.aux5v_v:.3f}")
                self.aux5v_warned = False

    # -- 100 Hz control loop --

    def control_tick(self):
        self.tick_count += 1
        self._check_safety()
        self._check_pushes()

        if self.latched or self.fault not in ("OK", "NOT_ENABLED"):
            self.motor_l.duty = self.motor_r.duty = 0.0
        elif self.mode == "IDLE":
            self.motor_l.duty = self.motor_r.duty = 0.0
        elif self.mode == "OPEN_LOOP":
            self.motor_l.duty = self.target_l * self.duty_limit
            self.motor_r.duty = self.target_r * self.duty_limit
        elif self.mode == "VELOCITY":
            accel = self.cfg["kin.max_accel"]
            if accel > 0:
                d = accel * CONTROL_DT
                self.ramped_target_l += max(-d, min(d, self.target_l - self.ramped_target_l))
                self.ramped_target_r += max(-d, min(d, self.target_r - self.ramped_target_r))
            else:
                self.ramped_target_l, self.ramped_target_r = self.target_l, self.target_r
            duty_l = self.pid_l.update(self.ramped_target_l, self.motor_l.filtered_vel, CONTROL_DT)
            duty_r = self.pid_r.update(self.ramped_target_r, self.motor_r.filtered_vel, CONTROL_DT)
            self.motor_l.duty = max(-self.duty_limit, min(self.duty_limit, duty_l))
            self.motor_r.duty = max(-self.duty_limit, min(self.duty_limit, duty_r))

        cpr = int(self.cfg["kin.counts_per_rev"])
        self.motor_l.step(self.motor_l.duty, CONTROL_DT, cpr, VEL_FILTER_ALPHA)
        self.motor_r.step(self.motor_r.duty, CONTROL_DT, cpr, VEL_FILTER_ALPHA)

    def get_stream_line(self):
        if not (self.stream_active and self.level >= 2):
            return None
        ticks_per_frame = max(1, CONTROL_HZ // self.stream_rate)
        if self.tick_count % ticks_per_frame != 0:
            return None
        return (
            f"! STREAM tl={self.target_l:.3f} tr={self.target_r:.3f} "
            f"vl={self.motor_l.filtered_vel:.3f} vr={self.motor_r.filtered_vel:.3f} "
            f"dl={self.motor_l.duty:.3f} dr={self.motor_r.duty:.3f} "
            f"vbat={self.vbat:.3f} battery={self._battery_field()} "
            f"t={self.millis()} el={self.motor_l.counts} er={self.motor_r.counts}"
        )

    # -- command dispatch --

    def handle_command(self, line):
        line = line.replace('\r', '').replace('\n', '')
        if len(line) > LINE_MAX:
            return f'ERR code=TOO_LONG msg="line exceeds {LINE_MAX} bytes"'
        if not line.strip():
            return None

        parts = line.strip().split()
        token = parts[0].upper()
        args = parts[1:]

        kv, bare = {}, []
        for a in args:
            if '=' in a:
                k, v = a.split('=', 1)
                if len(v) >= 2 and v.startswith('"') and v.endswith('"'):
                    v = v[1:-1]
                kv[k] = v
            else:
                bare.append(a)   # original case; handlers upper-case keywords

        handler = getattr(self, f'_cmd_{token}', None)
        if handler is None or COMMAND_LEVEL.get(token, 1) > self.level:
            return f'ERR code=BAD_CMD msg="unknown command: {token}"'

        try:
            return handler(kv, bare)
        except Exception as e:           # pragma: no cover - defensive
            return f'ERR code=INTERNAL msg="{e}"'

    # ---- Level 1: motion + safety ----

    def _cmd_PING(self, kv, bare):
        return f"OK PONG t={self.millis()}"

    def _arm_motion(self):
        now = time.monotonic()
        self.last_motion_time = now
        if not self.motion_active:
            self.last_hb_time = now
        self.motion_active = True
        if self.fault in ("OK", "NOT_ENABLED"):
            self.fault = "OK"

    def _cmd_CMD_VEL(self, kv, bare):
        ok, reason = self._can_move()
        if not ok:
            return f'ERR code={reason} msg="motion rejected"'
        if int(self.cfg["kin.counts_per_rev"]) == 0:
            return 'ERR code=NO_FEEDBACK msg="closed-loop motion needs encoders"'
        if 'v' not in kv or 'w' not in kv or kv['v'] == '' or kv['w'] == '':
            return 'ERR code=BAD_ARG msg="missing or empty v= / w="'
        try:
            v = float(kv['v']); w = float(kv['w'])
        except ValueError:
            return 'ERR code=BAD_ARG msg="v and w must be numeric"'

        v = max(-self.cfg.get("kin.max_v", MAX_LIN_VEL), min(self.cfg.get("kin.max_v", MAX_LIN_VEL), v))
        w = max(-self.cfg.get("kin.max_w", MAX_ANG_VEL), min(self.cfg.get("kin.max_w", MAX_ANG_VEL), w))
        tw = self.cfg["kin.track_width"]; wr = self.cfg["kin.wheel_radius"]
        wl = (v - w * tw / 2.0) / wr
        wrr = (v + w * tw / 2.0) / wr
        max_vel = self.duty_limit * MAX_MOTOR_RADS
        wl = max(-max_vel, min(max_vel, wl))
        wrr = max(-max_vel, min(max_vel, wrr))

        self._arm_motion()
        if self.mode != "VELOCITY":
            self.pid_l.reset(); self.pid_r.reset()
            self.ramped_target_l = self.motor_l.filtered_vel
            self.ramped_target_r = self.motor_r.filtered_vel
        self.mode = "VELOCITY"
        self.target_l, self.target_r = wl, wrr
        return f"OK CMD_VEL v={v:.3f} w={w:.3f} wl={wl:.3f} wr={wrr:.3f}"

    def _cmd_WHEEL(self, kv, bare):
        ok, reason = self._can_move()
        if not ok:
            return f'ERR code={reason} msg="motion rejected"'
        if 'l' not in kv or 'r' not in kv or kv['l'] == '' or kv['r'] == '':
            return 'ERR code=BAD_ARG msg="missing or empty l= / r="'
        try:
            l_val = float(kv['l']); r_val = float(kv['r'])
        except ValueError:
            return 'ERR code=BAD_ARG msg="l and r must be numeric"'

        mode = kv.get('mode', 'VEL').upper()
        if mode not in self.wheel_modes:
            return f'ERR code=BAD_ARG msg="unknown mode: {mode}"'
        if mode == 'VEL' and int(self.cfg["kin.counts_per_rev"]) == 0:
            return 'ERR code=NO_FEEDBACK msg="closed-loop WHEEL needs encoders"'

        self._arm_motion()
        if mode == 'DUTY':
            l_val = max(-1.0, min(1.0, l_val))
            r_val = max(-1.0, min(1.0, r_val))
            self.mode = "OPEN_LOOP"
            self.target_l, self.target_r = l_val, r_val
        else:
            max_vel = self.duty_limit * MAX_MOTOR_RADS
            l_val = max(-max_vel, min(max_vel, l_val))
            r_val = max(-max_vel, min(max_vel, r_val))
            if self.mode != "VELOCITY":
                self.pid_l.reset(); self.pid_r.reset()
                self.ramped_target_l = self.motor_l.filtered_vel
                self.ramped_target_r = self.motor_r.filtered_vel
            self.mode = "VELOCITY"
            self.target_l, self.target_r = l_val, r_val
        return f"OK WHEEL l={l_val:.3f} r={r_val:.3f}"

    def _cmd_STOP(self, kv, bare):
        bare_u = [b.upper() for b in bare]
        stop_mode = "COAST" if ("COAST" in bare_u or kv.get("mode", "").upper() == "COAST") else "BRAKE"
        self._stop_motors(stop_mode)
        self.motion_active = False
        return f"OK STOP mode={stop_mode}"

    def _cmd_STATUS(self, kv, bare):
        return (
            f"OK STATUS preset={self.preset} mode={self.mode} "
            f"en={'1' if self.enabled else '0'} fault={self.fault} "
            f"estop={'1' if self.estop else '0'} "
            f"tl={self.target_l:.3f} tr={self.target_r:.3f} "
            f"vl={self.motor_l.filtered_vel:.3f} vr={self.motor_r.filtered_vel:.3f} "
            f"dl={self.motor_l.duty:.3f} dr={self.motor_r.duty:.3f} "
            f"lim={self.duty_limit:.3f} "
            f"vbat={self.vbat:.3f} battery={self._battery_field()}"
        )

    def _cmd_ENABLE(self, kv, bare):
        state = bare[0].upper() if bare else "ON"
        if state == "ON":
            if self.estop:
                return 'ERR code=ESTOP msg="emergency stop active"'
            self.enabled = True
            self.fault = "OK"
            self.latched = False
            now = time.monotonic()
            self.last_hb_time = now
            self.last_motion_time = now
            return "OK ENABLE state=ON"
        elif state == "OFF":
            self.enabled = False
            self._stop_motors("BRAKE")
            self.motion_active = False
            return "OK ENABLE state=OFF"
        return 'ERR code=BAD_ARG msg="expected ON or OFF"'

    def _cmd_PRESET(self, kv, bare):
        if not bare:
            return self._preset_response(self.preset)
        p = bare[0].upper()
        if p not in ("SLOW", "NORMAL"):
            return f'ERR code=BAD_ARG msg="unknown preset: {p}"'
        self._apply_preset(p)
        return self._preset_response(p)

    def _preset_response(self, p):
        if p == "SLOW":
            # slow.timeout_ms is optional (e.g. the MC1 has no such config key —
            # its SLOW timeout is a fixed 0); default to 0 when absent.
            timeout = self.cfg.get("slow.timeout_ms", 0); enreq = 0; duty = self.cfg["slow.duty_limit"]
        else:
            timeout = self.cfg["normal.timeout_ms"]; enreq = 1; duty = self.cfg["normal.duty_limit"]
        hb = 1 if (p == "NORMAL" and self.level >= 2) else 0
        return (f"OK PRESET name={p} timeout_ms={int(timeout)} "
                f"enable_required={enreq} hb_required={hb} duty_limit={duty:.3f}")

    # ---- Level 2: system, telemetry, config ----

    def _cmd_INFO(self, kv, bare):
        ident = self.identity
        out = [f"fw={ident['fw']}"]
        for k, v in ident.get("info_extra", {}).items():
            out.append(f"{k}={v}")
        out.append(f"hw={ident['hw']}")
        out.append(f"proto=ORCP/{PROTO_VERSION}")
        if ident.get("vendor"):
            out.append(f"vendor={ident['vendor']}")
        if ident.get("model"):
            out.append(f"model={ident['model']}")
        out.append(f"level={self.level}")
        out.append(f"uuid={SIM_UUID}")
        return "OK INFO " + " ".join(out)

    def _cmd_HB(self, kv, bare):
        now = time.monotonic()
        self.last_hb_time = now
        self.last_motion_time = now
        if self.fault == "TIMEOUT":
            self.fault = "OK"; self.latched = False
        return "OK HB"

    def _cmd_STREAM(self, kv, bare):
        if not bare:
            return f"OK STREAM state={'ON' if self.stream_active else 'OFF'} rate={self.stream_rate}"
        state = bare[0].upper()
        if state == "ON":
            self.stream_active = True
            rate_src = kv.get('rate', bare[1] if len(bare) > 1 else None)
            if rate_src is not None:
                try:
                    self.stream_rate = max(1, min(50, int(rate_src)))
                except ValueError:
                    return 'ERR code=BAD_ARG msg="rate must be integer 1-50"'
            return f"OK STREAM state=ON rate={self.stream_rate}"
        elif state == "OFF":
            self.stream_active = False
            return f"OK STREAM state=OFF rate={self.stream_rate}"
        return 'ERR code=BAD_ARG msg="expected ON or OFF"'

    def _cmd_GET(self, kv, bare):
        if bare and bare[0].upper() == "ALL":
            fields = " ".join(f"{k}={self._fmt(k, v)}" for k, v in self.cfg.items())
            return f"OK GET {fields}"
        if not bare:
            return 'ERR code=BAD_ARG msg="GET needs a parameter name or ALL"'
        key = bare[0]
        if key not in self.cfg:
            return f'ERR code=BAD_ARG msg="unknown parameter: {key}"'
        return f"OK GET {key}={self._fmt(key, self.cfg[key])}"

    def _cmd_SET(self, kv, bare):
        if not kv:
            return 'ERR code=BAD_ARG msg="SET needs parameter=value"'
        key, raw = next(iter(kv.items()))
        if key not in self.cfg:
            return f'ERR code=BAD_ARG msg="unknown parameter: {key}"'
        if raw == '':
            return 'ERR code=BAD_ARG msg="empty value"'
        try:
            val = int(raw) if key in self.int_keys else float(raw)
        except ValueError:
            return 'ERR code=BAD_ARG msg="value must be numeric"'
        lo, hi = self.ranges[key]
        if val < lo or val > hi:
            return f'ERR code=BAD_VAL msg="{key} out of range [{lo}, {hi}]"'
        self.cfg[key] = val
        self._apply_config()
        return f"OK SET {key}={self._fmt(key, self.cfg[key])}"

    def _cmd_SAVE(self, kv, bare):
        if not self.config_file:
            return 'ERR code=FLASH_ERR msg="no persistent store configured"'
        try:
            with open(self.config_file, 'w') as f:
                json.dump(self.cfg, f)
        except OSError as e:
            return f'ERR code=FLASH_ERR msg="{e}"'
        return "OK SAVE"

    def _cmd_LOAD(self, kv, bare):
        if not self.config_file or not os.path.exists(self.config_file):
            return 'ERR code=FLASH_ERR msg="no saved configuration"'
        try:
            with open(self.config_file) as f:
                saved = json.load(f)
        except (OSError, ValueError) as e:
            return f'ERR code=FLASH_ERR msg="{e}"'
        for k, v in saved.items():
            if k in self.cfg:
                self.cfg[k] = int(v) if k in self.int_keys else float(v)
        self._apply_config()
        return "OK LOAD"

    def _cmd_DEFAULTS(self, kv, bare):
        self.cfg = {name: default for name, default, _, _ in self.profile["config"]}
        self._apply_config()
        return "OK DEFAULTS"

# ---------------------------------------------------------------------------
# PTY serial bridge + main loop
# ---------------------------------------------------------------------------

def _banner(sim, extra):
    print(f"ORCP Reference Simulator — proto ORCP/{PROTO_VERSION}, "
          f"profile {sim.profile['name']} (hw={sim.identity['hw']}), Level {sim.level}")
    print(extra)
    print(f"Preset: {sim.preset} | Battery: {sim.vbat:.1f}V | Config: {sim.config_file or '(none)'}")


def run_simulator(sim, link_path=None):
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)

    if link_path:
        try:
            if os.path.islink(link_path):
                os.unlink(link_path)
            os.symlink(slave_path, link_path)
            print(f"Symlink: {link_path} -> {slave_path}")
        except OSError as e:
            print(f"Warning: could not create symlink: {e}", file=sys.stderr)

    _banner(sim, f"Serial device: {slave_path}")
    print("Waiting for connection... (Ctrl+C to quit)")

    rx_buf = b""
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        print("\nShutting down...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    last_tick = time.monotonic()
    tick_interval = CONTROL_DT

    def emit(text):
        try:
            os.write(master_fd, (text + "\n").encode())
        except OSError:
            pass

    try:
        while running:
            now = time.monotonic()
            if now - last_tick >= tick_interval:
                last_tick += tick_interval
                if now - last_tick > tick_interval * 10:
                    last_tick = now
                sim.control_tick()
                for msg in sim.pending_push:
                    emit(msg)
                sim.pending_push.clear()
                stream_line = sim.get_stream_line()
                if stream_line:
                    emit(stream_line)

            timeout = max(0.0, tick_interval - (time.monotonic() - last_tick))
            ready, _, _ = select.select([master_fd], [], [], timeout)
            if ready:
                try:
                    data = os.read(master_fd, 1024)
                except OSError:
                    break
                if not data:
                    break
                rx_buf += data
                while b'\n' in rx_buf:
                    line, rx_buf = rx_buf.split(b'\n', 1)
                    response = sim.handle_command(line.decode('ascii', errors='replace'))
                    if response is not None:
                        emit(response)
                    for msg in sim.pending_push:
                        emit(msg)
                    sim.pending_push.clear()
    finally:
        os.close(master_fd)
        os.close(slave_fd)
        if link_path and os.path.islink(link_path):
            os.unlink(link_path)
        print("Simulator stopped.")


# ---------------------------------------------------------------------------
# WebSocket transport (optional — for browser hosts, e.g. the web configurator)
# ---------------------------------------------------------------------------

async def _ws_run(sim, host, port, ready=None, stop=None):
    """Serve `sim` over a WebSocket. Each text frame carries one or more
    newline-terminated ORCP lines; clients treat the stream like the serial
    byte stream. `ready`/`stop` are test hooks."""
    import asyncio
    import websockets

    clients = set()

    async def handler(ws):
        clients.add(ws)
        try:
            async for message in ws:
                text = message if isinstance(message, str) else message.decode('ascii', 'replace')
                for line in text.split('\n'):
                    if not line.strip():
                        continue
                    resp = sim.handle_command(line)
                    if resp is not None:
                        await ws.send(resp + '\n')
        except Exception:
            pass
        finally:
            clients.discard(ws)

    server = await websockets.serve(handler, host, port)
    if ready is not None and not ready.done():
        ready.set_result(server.sockets[0].getsockname()[1])

    loop = asyncio.get_running_loop()
    next_t = loop.time()
    try:
        while not (stop is not None and stop.is_set()):
            sim.control_tick()
            out = list(sim.pending_push)
            sim.pending_push.clear()
            line = sim.get_stream_line()
            if line:
                out.append(line)
            if out and clients:
                dead = set()
                for ws in list(clients):
                    for m in out:
                        try:
                            await ws.send(m + '\n')
                        except Exception:
                            dead.add(ws)
                clients -= dead
            next_t += CONTROL_DT
            await asyncio.sleep(max(0.0, next_t - loop.time()))
    finally:
        server.close()
        await server.wait_closed()


def run_ws_simulator(sim, port=8765):
    import asyncio
    try:
        import websockets  # noqa: F401
    except ImportError:
        print("WebSocket mode requires the 'web' extra:\n"
              "    pip install 'orcp-sim[web]'    (or: pip install websockets)",
              file=sys.stderr)
        sys.exit(1)

    _banner(sim, f"WebSocket endpoint: ws://localhost:{port}")
    print("Point the web configurator's simulator/dev mode at this URL. (Ctrl+C to quit)")

    async def _main():
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, stop.set)
            except NotImplementedError:
                pass
        await _ws_run(sim, "localhost", port, stop=stop)

    try:
        asyncio.run(_main())
    except KeyboardInterrupt:
        pass
    print("\nSimulator stopped.")


def main():
    parser = argparse.ArgumentParser(description="ORCP Reference Simulator (ORCP v1.1)")
    parser.add_argument("--profile", choices=sorted(PROFILES), default="base",
                        help="Built-in controller profile to emulate (default: base = generic v1.1)")
    parser.add_argument("--profile-file", type=str, default=None, metavar="PATH",
                        help="Load a vendor profile from a JSON file (overrides --profile). "
                             "See the README 'Creating a profile' section.")
    parser.add_argument("--level", type=int, choices=(1, 2, 3), default=None,
                        help="Override the profile's conformance level (1/2/3)")
    parser.add_argument("--link", "-l", type=str, default=None,
                        help="Create a symlink to the PTY at this path (e.g. /tmp/orcp)")
    parser.add_argument("--vbat", type=float, default=12.4,
                        help="Simulated battery voltage (default: 12.4V)")
    parser.add_argument("--estop", action="store_true",
                        help="Start with e-stop active")
    parser.add_argument("--aux5v-amps", type=float, default=None,
                        help="Simulated 5V aux-rail current (profiles with an aux rail)")
    parser.add_argument("--aux5v-volts", type=float, default=None,
                        help="Simulated 5V aux-rail voltage (profiles with an aux rail)")
    parser.add_argument("--config-file", type=str,
                        default=os.path.join(tempfile.gettempdir(), "orcp_sim_config.json"),
                        help="Persistent config store for SAVE/LOAD (default: temp dir)")
    parser.add_argument("--ws", type=int, metavar="PORT", default=None,
                        help="Serve over a WebSocket on PORT (for browser hosts) instead of a PTY. "
                             "Needs the 'web' extra.")
    args = parser.parse_args()

    if args.profile_file is not None:
        try:
            profile = load_profile_file(args.profile_file)
        except (OSError, ValueError, json.JSONDecodeError) as e:
            parser.error(f"could not load --profile-file: {e}")
    else:
        profile = PROFILES[args.profile]

    sim = ORCPSim(profile=profile, level=args.level,
                  config_file=args.config_file, vbat=args.vbat, estop=args.estop,
                  aux5v_amps=args.aux5v_amps, aux5v_volts=args.aux5v_volts)

    if args.ws is not None:
        run_ws_simulator(sim, port=args.ws)
    else:
        run_simulator(sim, link_path=args.link)


if __name__ == "__main__":
    main()
