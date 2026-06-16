#!/usr/bin/env python3
"""
ORCP Reference Simulator — conformant to ORCP v1.1.

Creates a virtual serial port (PTY) that emulates an ORCP controller. Connect
test scripts, client libraries, or ROS 2 nodes to the PTY device path printed
at startup.

A single simulator, configurable to a conformance level (§9 of the spec):

    --level 1   Basic    : PING CMD_VEL WHEEL STOP STATUS ENABLE PRESET
                           (motion + mandatory safety; no runtime config,
                            no heartbeat monitoring, no streaming)
    --level 2   Standard : Level 1 + INFO HB STREAM GET SET SAVE LOAD DEFAULTS
                           (runtime config with persistence, heartbeat &
                            e-stop monitoring, telemetry streaming)   [default]
    --level 3   Extended : Level 2 + (CAN binary encoding — not emulated over a
                           PTY; behaves as Level 2 on this ASCII transport)

The controller declares its level in the INFO response (`level=`), exactly as a
real device does, so a host can test how it adapts to each level by flipping the
flag. Commands above the active level return `ERR code=BAD_CMD` — a Level 1
controller genuinely does not implement SET, etc.

Usage:
    python3 orcp_sim.py                       # Level 2, auto PTY
    python3 orcp_sim.py --level 1             # Level 1 controller
    python3 orcp_sim.py --link /tmp/orcp      # also symlink the PTY
    python3 orcp_sim.py --config-file cfg.json

The simulator runs a 100 Hz control loop with PID, encoder quantisation, the
safety system, and streaming telemetry.
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
# Identity (ORCP v1.1)
# ---------------------------------------------------------------------------

PROTO_VERSION = "1.1"           # → proto=ORCP/1.1
FW_VERSION    = "1.1.0"
HW_ID         = "ORCP-SIM"
VENDOR        = "ORCP-Project"
MODEL         = "reference-sim"
SIM_UUID      = "00SIMULATED00FF"

# Physical model constants (not part of the standard config surface)
MAX_MOTOR_RADS = 20.9           # rad/s at full duty
MAX_LIN_VEL    = 1.0            # m/s   CMD_VEL sanity clamp
MAX_ANG_VEL    = 3.0           # rad/s CMD_VEL sanity clamp
VEL_FILTER_ALPHA = 0.2

CONTROL_HZ = 100
CONTROL_DT = 1.0 / CONTROL_HZ
LINE_MAX   = 256                # §2.2 command-line cap

# ---------------------------------------------------------------------------
# Standard configuration parameters (ORCP v1.1 §7)
# ---------------------------------------------------------------------------

# Insertion order = GET ALL output order.
CONFIG_DEFAULTS = {
    # §7.1 Kinematics
    "kin.counts_per_rev": 1996,
    "kin.wheel_radius":   0.049,
    "kin.track_width":    0.175,
    "kin.max_accel":      10.2,
    # §7.2 PID
    "pid.kp":             0.05,
    "pid.ki":             0.3,
    "pid.kd":             0.0,
    # §7.3 Battery
    "batt.full_v":        12.6,
    "batt.low_v":         10.2,
    "batt.critical_v":    9.6,
    # §7.4 Safety timing
    "slow.duty_limit":    0.30,
    "normal.duty_limit":  0.90,
    "slow.timeout_ms":    0,        # 0 = no command timeout in restricted mode
    "normal.timeout_ms":  250,
    "hb.timeout_ms":      500,
}

CONFIG_INT_KEYS = {
    "kin.counts_per_rev", "slow.timeout_ms", "normal.timeout_ms", "hb.timeout_ms",
}

CONFIG_RANGE = {
    "kin.counts_per_rev": (0, 10000),
    "kin.wheel_radius":   (0.01, 0.5),
    "kin.track_width":    (0.05, 1.0),
    "kin.max_accel":      (0.0, 100.0),
    "pid.kp":             (0.0, 10.0),
    "pid.ki":             (0.0, 50.0),
    "pid.kd":             (0.0, 1.0),
    "batt.full_v":        (5.0, 30.0),
    "batt.low_v":         (5.0, 30.0),
    "batt.critical_v":    (5.0, 30.0),
    "slow.duty_limit":    (0.05, 1.0),
    "normal.duty_limit":  (0.05, 1.0),
    "slow.timeout_ms":    (0, 60000),
    "normal.timeout_ms":  (0, 60000),
    "hb.timeout_ms":      (10, 60000),
}

def fmt_value(key, val):
    if key in CONFIG_INT_KEYS:
        return str(int(round(val)))
    return f"{float(val):.3f}"

# Command → minimum conformance level that implements it (§9).
COMMAND_LEVEL = {
    "PING": 1, "CMD_VEL": 1, "WHEEL": 1, "STOP": 1,
    "STATUS": 1, "ENABLE": 1, "PRESET": 1,
    "INFO": 2, "HB": 2, "STREAM": 2,
    "GET": 2, "SET": 2, "SAVE": 2, "LOAD": 2, "DEFAULTS": 2,
}

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
            # No encoders: report the modelled velocity directly (open-loop only).
            self.delta_counts = 0
            raw_vel = self.velocity
        self.filtered_vel += alpha * (raw_vel - self.filtered_vel)

# ---------------------------------------------------------------------------
# ORCP simulator
# ---------------------------------------------------------------------------

class ORCPSim:
    def __init__(self, level=2, config_file=None, vbat=12.4, estop=False):
        self.level = level
        self.config_file = config_file

        self.cfg = dict(CONFIG_DEFAULTS)

        self.motor_l = MotorSim()
        self.motor_r = MotorSim()

        self.mode = "IDLE"          # IDLE | OPEN_LOOP | VELOCITY
        self.target_l = 0.0
        self.target_r = 0.0
        self.ramped_target_l = 0.0
        self.ramped_target_r = 0.0

        self.pid_l = PID(self.cfg["pid.kp"], self.cfg["pid.ki"], self.cfg["pid.kd"], 0.30)
        self.pid_r = PID(self.cfg["pid.kp"], self.cfg["pid.ki"], self.cfg["pid.kd"], 0.30)
        self.duty_limit = self.cfg["slow.duty_limit"]

        # Safety
        self.preset = "SLOW"
        self.enabled = True
        self.fault = "OK"
        self.latched = False        # fault requires ENABLE ON to clear
        self.estop = estop
        self.last_motion_time = 0.0
        self.last_hb_time = 0.0
        self.motion_active = False

        # Battery
        self.vbat = vbat
        self.battery_pct = self._estimate_pct(vbat)
        self.batt_warned = False

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

    def _estimate_pct(self, vbat):
        if vbat <= 2.0:
            return 0
        lo, hi = self.cfg["batt.critical_v"], self.cfg["batt.full_v"]
        return max(0, min(100, int((vbat - lo) / (hi - lo) * 100)))

    def _batt_critical(self):
        return 2.0 < self.vbat < self.cfg["batt.critical_v"]

    def _hb_required(self):
        # Heartbeat is a Level 2+ feature (§5.5); the full preset declares it.
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
        # Highest priority: e-stop can escalate over any lower latched fault.
        if self.estop and self.fault != "ESTOP":
            self._raise_fault("ESTOP")
            return
        if self.latched or self.fault == "ESTOP":
            return  # stays latched until ENABLE ON
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
        # Transient gate state (not latched)
        if self.preset == "NORMAL" and not self.enabled:
            self.fault = "NOT_ENABLED"
        else:
            self.fault = "OK"

    # -- battery warning push (§6.2) --

    def _check_battery_warn(self):
        low = self.cfg["batt.low_v"]
        crit = self.cfg["batt.critical_v"]
        if self.level >= 2 and crit < self.vbat < low and not self.batt_warned:
            self.pending_push.append(f"! WARN BATT vbat={self.vbat:.3f}")
            self.batt_warned = True
        elif self.vbat >= low:
            self.batt_warned = False

    # -- 100 Hz control loop --

    def control_tick(self):
        self.tick_count += 1
        self._check_safety()
        self._check_battery_warn()

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
                max_delta = accel * CONTROL_DT
                self.ramped_target_l += max(-max_delta, min(max_delta, self.target_l - self.ramped_target_l))
                self.ramped_target_r += max(-max_delta, min(max_delta, self.target_r - self.ramped_target_r))
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
            f"vbat={self.vbat:.3f} battery={self.battery_pct}% "
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

        # Level gating (§9): a command above the active level is not implemented.
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

        v = max(-MAX_LIN_VEL, min(MAX_LIN_VEL, v))
        w = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, w))
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

        # Default semantics are rad/s closed-loop velocity (§4 WHEEL). A vendor
        # mode=DUTY extension opts into open-loop direct duty.
        mode = kv.get('mode', 'VEL').upper()
        if mode not in ('VEL', 'DUTY'):
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
            f"vbat={self.vbat:.3f} battery={self.battery_pct}%"
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
            timeout = self.cfg["slow.timeout_ms"]; enreq = 0; duty = self.cfg["slow.duty_limit"]
        else:
            timeout = self.cfg["normal.timeout_ms"]; enreq = 1; duty = self.cfg["normal.duty_limit"]
        hb = 1 if (p == "NORMAL" and self.level >= 2) else 0
        return (f"OK PRESET name={p} timeout_ms={int(timeout)} "
                f"enable_required={enreq} hb_required={hb} duty_limit={duty:.3f}")

    # ---- Level 2: system, telemetry, config ----

    def _cmd_INFO(self, kv, bare):
        return (f"OK INFO fw={FW_VERSION} hw={HW_ID} proto=ORCP/{PROTO_VERSION} "
                f"vendor={VENDOR} model={MODEL} level={self.level} uuid={SIM_UUID}")

    def _cmd_HB(self, kv, bare):
        now = time.monotonic()
        self.last_hb_time = now
        self.last_motion_time = now      # heartbeat also resets command timeout (§4 HB)
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
            fields = " ".join(f"{k}={fmt_value(k, v)}" for k, v in self.cfg.items())
            return f"OK GET {fields}"
        if not bare:
            return 'ERR code=BAD_ARG msg="GET needs a parameter name or ALL"'
        key = bare[0]
        if key not in self.cfg:
            return f'ERR code=BAD_ARG msg="unknown parameter: {key}"'
        return f"OK GET {key}={fmt_value(key, self.cfg[key])}"

    def _cmd_SET(self, kv, bare):
        if not kv:
            return 'ERR code=BAD_ARG msg="SET needs parameter=value"'
        key, raw = next(iter(kv.items()))
        if key not in self.cfg:
            return f'ERR code=BAD_ARG msg="unknown parameter: {key}"'
        if raw == '':
            return 'ERR code=BAD_ARG msg="empty value"'
        try:
            val = int(raw) if key in CONFIG_INT_KEYS else float(raw)
        except ValueError:
            return f'ERR code=BAD_ARG msg="value must be numeric"'
        lo, hi = CONFIG_RANGE[key]
        if val < lo or val > hi:
            return f'ERR code=BAD_VAL msg="{key} out of range [{lo}, {hi}]"'
        self.cfg[key] = val
        self._apply_config()
        return f"OK SET {key}={fmt_value(key, self.cfg[key])}"

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
        self.cfg = dict(CONFIG_DEFAULTS)
        for k, v in saved.items():
            if k in self.cfg:
                self.cfg[k] = int(v) if k in CONFIG_INT_KEYS else float(v)
        self._apply_config()
        return "OK LOAD"

    def _cmd_DEFAULTS(self, kv, bare):
        self.cfg = dict(CONFIG_DEFAULTS)
        self._apply_config()
        return "OK DEFAULTS"

# ---------------------------------------------------------------------------
# PTY serial bridge + main loop
# ---------------------------------------------------------------------------

def run_simulator(level=2, link_path=None, vbat=12.4, estop=False, config_file=None):
    sim = ORCPSim(level=level, config_file=config_file, vbat=vbat, estop=estop)

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

    print(f"ORCP Reference Simulator — proto ORCP/{PROTO_VERSION}, fw {FW_VERSION}, Level {level}")
    print(f"Serial device: {slave_path}")
    print(f"Preset: {sim.preset} | Battery: {sim.vbat:.1f}V | Config: {config_file or '(none)'}")
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


def main():
    parser = argparse.ArgumentParser(description="ORCP Reference Simulator (ORCP v1.1)")
    parser.add_argument("--level", type=int, choices=(1, 2, 3), default=2,
                        help="Conformance level to emulate (default: 2)")
    parser.add_argument("--link", "-l", type=str, default=None,
                        help="Create a symlink to the PTY at this path (e.g. /tmp/orcp)")
    parser.add_argument("--vbat", type=float, default=12.4,
                        help="Simulated battery voltage (default: 12.4V)")
    parser.add_argument("--estop", action="store_true",
                        help="Start with e-stop active")
    parser.add_argument("--config-file", type=str,
                        default=os.path.join(tempfile.gettempdir(), "orcp_sim_config.json"),
                        help="Persistent config store for SAVE/LOAD (default: temp dir)")
    args = parser.parse_args()

    run_simulator(level=args.level, link_path=args.link, vbat=args.vbat,
                  estop=args.estop, config_file=args.config_file)


if __name__ == "__main__":
    main()
