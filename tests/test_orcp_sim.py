"""Compliance and behaviour tests for the ORCP reference simulator (ORCP v1.1)."""
import re
import time
import pytest

from orcp_sim import ORCPSim, MC1_PROFILE


@pytest.fixture
def sim(tmp_path):
    """A Level 2 simulator with an isolated persistent-config file."""
    return ORCPSim(level=2, config_file=str(tmp_path / "cfg.json"))


@pytest.fixture
def mc1(tmp_path):
    """A simulator running the MC1 profile."""
    return ORCPSim(profile=MC1_PROFILE, config_file=str(tmp_path / "mc1.json"),
                   aux5v_amps=1.0, aux5v_volts=5.0)


# ---------------------------------------------------------------------------
# Identity / system
# ---------------------------------------------------------------------------

def test_info_advertises_v11_and_level(sim):
    r = sim.handle_command("INFO")
    assert "proto=ORCP/1.1" in r
    assert "fw=1.1.0" in r
    assert "level=2" in r


def test_ping(sim):
    assert sim.handle_command("PING").startswith("OK PONG")


# ---------------------------------------------------------------------------
# Presets — heartbeat decoupling (§5.5 / §9)
# ---------------------------------------------------------------------------

def test_preset_slow_fields(sim):
    r = sim.handle_command("PRESET SLOW")
    assert "name=SLOW" in r
    assert "enable_required=0" in r
    assert "hb_required=0" in r
    assert "duty_limit=0.300" in r


def test_preset_normal_requires_heartbeat_at_level2(sim):
    r = sim.handle_command("PRESET NORMAL")
    assert "name=NORMAL" in r
    assert "enable_required=1" in r
    assert "hb_required=1" in r
    assert "timeout_ms=250" in r


def test_preset_normal_no_heartbeat_at_level1(tmp_path):
    s = ORCPSim(level=1, config_file=str(tmp_path / "c.json"))
    r = s.handle_command("PRESET NORMAL")
    assert "name=NORMAL" in r
    assert "hb_required=0" in r          # heartbeat is Level 2+


# ---------------------------------------------------------------------------
# Motion + WHEEL default semantics
# ---------------------------------------------------------------------------

def test_enable(sim):
    assert sim.handle_command("ENABLE ON") == "OK ENABLE state=ON"
    assert sim.handle_command("ENABLE OFF") == "OK ENABLE state=OFF"


def test_wheel_defaults_to_radps(sim):
    # The default (no mode=) MUST be rad/s closed-loop velocity, not duty.
    assert sim.handle_command("WHEEL l=5 r=-5") == "OK WHEEL l=5.000 r=-5.000"
    assert sim.handle_command("STATUS").count("mode=VELOCITY") == 1


def test_wheel_duty_is_opt_in(sim):
    r = sim.handle_command("WHEEL l=0.5 r=0.5 mode=DUTY")
    assert r.startswith("OK WHEEL")
    assert "mode=OPEN_LOOP" in sim.handle_command("STATUS")


def test_stop(sim):
    assert sim.handle_command("STOP") == "OK STOP mode=BRAKE"


def test_status_fields(sim):
    r = sim.handle_command("STATUS")
    for field in ("preset=", "mode=", "en=", "fault=", "estop=",
                  "tl=", "tr=", "vl=", "vr=", "dl=", "dr=", "lim=", "vbat=", "battery="):
        assert field in r


# ---------------------------------------------------------------------------
# Configuration surface (Level 2)
# ---------------------------------------------------------------------------

def test_get_set_roundtrip(sim):
    assert sim.handle_command("GET pid.kp") == "OK GET pid.kp=0.050"
    assert sim.handle_command("SET pid.kp=0.080") == "OK SET pid.kp=0.080"
    assert sim.handle_command("GET pid.kp") == "OK GET pid.kp=0.080"


def test_set_out_of_range_is_bad_val(sim):
    assert "code=BAD_VAL" in sim.handle_command("SET pid.kp=99")


def test_set_empty_value_is_bad_arg(sim):
    assert "code=BAD_ARG" in sim.handle_command("SET pid.kp=")


def test_set_unknown_key_is_bad_arg(sim):
    assert "code=BAD_ARG" in sim.handle_command("SET nope.key=1")


def test_get_all_single_line(sim):
    r = sim.handle_command("GET ALL")
    assert r.startswith("OK GET ")
    assert "kin.counts_per_rev=1996" in r
    assert "hb.timeout_ms=500" in r


def test_save_load_defaults_persistence(sim):
    sim.handle_command("SET pid.kp=0.080")
    assert sim.handle_command("SAVE") == "OK SAVE"
    assert sim.handle_command("DEFAULTS") == "OK DEFAULTS"
    assert sim.handle_command("GET pid.kp") == "OK GET pid.kp=0.050"   # reset
    assert sim.handle_command("LOAD") == "OK LOAD"
    assert sim.handle_command("GET pid.kp") == "OK GET pid.kp=0.080"   # restored


def test_load_without_saved_config_is_flash_err(sim):
    assert "code=FLASH_ERR" in sim.handle_command("LOAD")


def test_stream_response_shape(sim):
    assert sim.handle_command("STREAM ON 20") == "OK STREAM state=ON rate=20"
    assert sim.handle_command("STREAM OFF") == "OK STREAM state=OFF rate=20"


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------

def test_too_long(sim):
    assert "code=TOO_LONG" in sim.handle_command("PING " + "x" * 300)


def test_unknown_command(sim):
    assert "code=BAD_CMD" in sim.handle_command("FLOOP")


def test_no_feedback_when_no_encoders(sim):
    sim.handle_command("SET kin.counts_per_rev=0")
    assert "code=NO_FEEDBACK" in sim.handle_command("CMD_VEL v=0.1 w=0")
    assert "code=NO_FEEDBACK" in sim.handle_command("WHEEL l=5 r=5")
    # vendor duty mode does not need feedback
    assert sim.handle_command("WHEEL l=0.5 r=0.5 mode=DUTY").startswith("OK WHEEL")


def test_estop_rejection_codes(tmp_path):
    s = ORCPSim(level=2, config_file=str(tmp_path / "c.json"), estop=True)
    assert "code=ESTOP" in s.handle_command("ENABLE ON")
    assert "code=ESTOP" in s.handle_command("CMD_VEL v=0.1 w=0")


# ---------------------------------------------------------------------------
# Level gating (§9)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("cmd", ["INFO", "HB", "STREAM ON", "GET pid.kp", "SET pid.kp=0.1",
                                 "SAVE", "LOAD", "DEFAULTS"])
def test_level1_rejects_level2_commands(tmp_path, cmd):
    s = ORCPSim(level=1, config_file=str(tmp_path / "c.json"))
    assert "code=BAD_CMD" in s.handle_command(cmd)


def test_level1_motion_still_works(tmp_path):
    s = ORCPSim(level=1, config_file=str(tmp_path / "c.json"))
    assert s.handle_command("PING").startswith("OK PONG")
    s.handle_command("PRESET SLOW")
    assert s.handle_command("CMD_VEL v=0.1 w=0").startswith("OK CMD_VEL")


# ---------------------------------------------------------------------------
# Dynamic safety behaviour
# ---------------------------------------------------------------------------

def _arm_normal(s):
    s.handle_command("PRESET NORMAL")
    s.handle_command("ENABLE ON")
    s.handle_command("WHEEL l=5 r=5")


def test_heartbeat_timeout_latches_and_pushes(sim):
    _arm_normal(sim)
    sim.last_hb_time -= 1.0          # exceed the 0.5 s heartbeat timeout
    sim.pending_push.clear()
    sim.control_tick()
    assert sim.fault == "HEARTBEAT"
    assert "! FAULT HEARTBEAT" in sim.pending_push
    # latched: motion rejected until ENABLE ON
    assert sim.handle_command("WHEEL l=5 r=5").startswith("ERR code=HEARTBEAT")
    assert sim.handle_command("ENABLE ON") == "OK ENABLE state=ON"
    assert sim.fault == "OK"


def test_command_timeout_latches(sim):
    _arm_normal(sim)
    sim.last_motion_time -= 1.0      # exceed the 0.25 s command timeout
    sim.last_hb_time = time.monotonic()
    sim.control_tick()
    assert sim.fault == "TIMEOUT"


def test_level1_normal_has_no_heartbeat_fault(tmp_path):
    s = ORCPSim(level=1, config_file=str(tmp_path / "c.json"))
    s.handle_command("PRESET NORMAL")
    s.handle_command("ENABLE ON")
    s.handle_command("WHEEL l=5 r=5")
    s.last_hb_time -= 5.0
    s.control_tick()
    assert s.fault != "HEARTBEAT"


def test_estop_latches_until_enable(tmp_path):
    s = ORCPSim(level=2, config_file=str(tmp_path / "c.json"), estop=True)
    s.control_tick()
    assert s.fault == "ESTOP"
    s.estop = False
    s.control_tick()
    assert s.fault == "ESTOP"         # persists after release
    assert s.handle_command("ENABLE ON") == "OK ENABLE state=ON"


def test_streaming_emits_frames(sim):
    sim.handle_command("STREAM ON 20")     # 100 Hz / 20 Hz -> a frame every 5 ticks
    frames = []
    for _ in range(12):
        sim.control_tick()
        line = sim.get_stream_line()
        if line:
            frames.append(line)
    assert len(frames) >= 2
    assert frames[0].startswith("! STREAM tl=")
    for field in ("vl=", "dl=", "vbat=", "battery="):
        assert field in frames[0]


# ---------------------------------------------------------------------------
# WebSocket transport (optional 'web' extra)
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Implementation profiles — MC1
# ---------------------------------------------------------------------------

def test_base_profile_has_no_vendor_keys(sim):
    # The generic reference must not carry MC1-specific keys.
    assert "code=BAD_ARG" in sim.handle_command("SET batt.hyst_v=0.3")
    assert len(sim.cfg) == 15


def test_mc1_identity(mc1):
    r = mc1.handle_command("INFO")
    assert "hw=MC1" in r
    assert "bl=1.4.0" in r
    assert "fw=1.14.3" in r
    assert "level=2" in r
    assert "vendor=" not in r          # MC1 INFO carries no vendor/model fields


def test_mc1_full_key_surface(mc1):
    assert len(mc1.cfg) == 63          # FW 1.13.0 / CONFIG 25
    # the key that originally failed against the generic sim:
    assert mc1.handle_command("SET batt.hyst_v=0.3") == "OK SET batt.hyst_v=0.300000"
    assert mc1.handle_command("GET batt.hyst_v") == "OK GET batt.hyst_v=0.300000"
    # a few more MC1-only keys:
    for key in ("aux5v.i_warn", "aux5v.i_max", "motor.stall_vel", "current.offset_left", "motor.i_max", "current_loop.enable"):
        assert mc1.handle_command(f"GET {key}").startswith("OK GET " + key + "=")


def test_mc1_key_surface_matches_firmware(mc1):
    """Keys the firmware has that the profile once lacked, and one it removed.

    ⚠️ current.scale is the case that matters: it was SPLIT per-side at CONFIG
    24, and a profile still offering it lets host code SET a key that ERRs on
    real hardware — worse than a missing key, because it fails only on the
    robot."""
    for key in ("ff.static", "ff.visc", "ff.inertia", "pid.kp_c", "pid.ki_c",
                "encoder.obs_enable", "encoder.obs_alpha", "encoder.obs_beta",
                "current.scale_left", "current.scale_right", "reflex.enable",
                "coast.park_vel", "coast.park_ms", "coast.max_ms",
                "hold.kp", "hold.max_vel", "hold.deadband", "hold.max_ms"):
        assert key in mc1.cfg, f"{key} missing from the MC1 profile"
    assert "current.scale" not in mc1.cfg
    assert mc1.handle_command("GET current.scale").startswith("ERR")


def test_mc1_defaults_match_firmware(mc1):
    """Spot-check defaults that were WRONG while the profile was hand-maintained.

    ⚠️ The invert flags are the dangerous ones: both motors and both encoders
    were reversed, so anyone establishing wheel direction against the simulator
    got the opposite of the real board."""
    assert mc1.cfg["motor.invert_left"] == 1
    assert mc1.cfg["motor.invert_right"] == 0
    assert mc1.cfg["encoder.invert_left"] == 1
    assert mc1.cfg["encoder.invert_right"] == 0
    assert mc1.cfg["kin.max_accel"] == 5      # was 25.0; unachievable, see Rev C 3.5
    assert mc1.cfg["ff.inertia"] == 0.052     # measured, Rev C 3.6
    assert mc1.cfg["pid.kp"] == 0.5
    assert mc1.cfg["pid.ki"] == 7
    assert mc1.cfg["batt.low_v"] == 9.9
    assert mc1.cfg["batt.critical_v"] == 9
    assert mc1.cfg["aux5v.i_max"] == 6        # rail is specified at 6 A, B.10


def test_mc1_renders_all_values_as_float(mc1):
    """MC1 has no int_keys AND uses six decimals since FW 1.11.0 (B.16) — three
    could not round-trip per-board calibration constants like
    current.offset_left (~0.006), so a GET ALL snapshot silently degraded the
    calibration it was meant to record."""
    assert mc1.handle_command("GET kin.counts_per_rev") == "OK GET kin.counts_per_rev=2249.000000"
    assert mc1.handle_command("SET current.offset_left=0.00375") == "OK SET current.offset_left=0.003750"
    assert mc1.handle_command("GET current.offset_left") == "OK GET current.offset_left=0.003750"


def test_base_profile_keeps_three_decimals(sim):
    """config_decimals is per-profile: the ORCP reference surface is unchanged."""
    assert sim.handle_command("GET pid.kp") == "OK GET pid.kp=0.050"


def test_mc1_status_battery_is_percent(mc1):
    """MC1 reports `battery=<n>%` (command.c formats "battery=%u%%"). The profile
    said "band" and this test asserted it — both were wrong together, which is
    how it survived."""
    assert re.search(r"battery=\d+%(\s|$)", mc1.handle_command("STATUS"))


def test_mc1_get_all_is_63_keys(mc1):
    r = mc1.handle_command("GET ALL")
    assert r.startswith("OK GET ")
    assert r.count("=") == 63


# ---------------------------------------------------------------------------
# Vendor extensions: coast-and-park, STOP HOLD
#
# ⚠️ Neither is ORCP v1.1. Every test here asserts BOTH that the MC1 profile has
# the behaviour and that `base` does not — the boundary is the point, because a
# host written against `base` must stay portable to any ORCP device.
# ---------------------------------------------------------------------------

def test_stop_coast_parks_itself(mc1):
    """STOP COAST coasts to rest and then applies a parking brake, so a coast on
    a slope cannot roll back down."""
    mc1.handle_command("WHEEL l=3 r=3")
    for _ in range(60):
        mc1.control_tick()
    assert mc1.motor_l.filtered_vel > 1.0

    assert mc1.handle_command("STOP mode=COAST") == "OK STOP mode=COAST parking=auto"
    assert mc1.coasting
    assert "coast=1" in mc1.handle_command("STATUS")

    moved_during_coast = mc1.motor_l.counts
    for _ in range(1000):
        mc1.control_tick()
        if not mc1.coasting:
            break
    assert not mc1.coasting
    # It genuinely coasted rather than stopping dead — the pre-1.12.0 bug was
    # motor_stop() discarding its mode argument, so COAST silently braked.
    assert mc1.motor_l.counts > moved_during_coast
    assert "coast=0" in mc1.handle_command("STATUS")


def test_base_stop_coast_does_not_park(sim):
    assert sim.handle_command("STOP mode=COAST") == "OK STOP mode=COAST"
    assert "coast=" not in sim.handle_command("STATUS")


def test_stop_hold_engages_and_reports(mc1):
    assert mc1.handle_command("STOP hold=1") == "OK STOP mode=BRAKE hold=on"
    mc1.control_tick()
    assert mc1.hold_state == 1
    assert "hold=1" in mc1.handle_command("STATUS")


def test_stop_coast_hold_latches_target_at_rest_not_at_command(mc1):
    """⚠️ The target must be latched where the robot STOPS, not where it was when
    asked to stop. Latching at command time would aim at a position the robot
    has since coasted away from — on a slope, the loop would drive back up it."""
    mc1.handle_command("WHEEL l=3 r=3")
    for _ in range(60):
        mc1.control_tick()
    at_command = mc1.motor_l.counts

    assert mc1.handle_command("STOP mode=COAST hold=1") == "OK STOP mode=COAST parking=auto hold=on"
    assert mc1.hold_pending and mc1.hold_state == 0   # not yet holding

    for _ in range(1000):
        mc1.control_tick()
        if mc1.hold_state == 1:
            break
    assert mc1.hold_state == 1
    assert mc1.hold_target[0] > at_command


def test_hold_exits_cleanly_without_reporting(mc1):
    """A plain STOP, ENABLE OFF and a new motion command are DELIBERATE exits:
    hold goes to 0, not to the fault/timeout value 2."""
    for exit_cmd in ("STOP", "ENABLE OFF", "WHEEL l=1 r=1"):
        mc1.handle_command("ENABLE ON")
        mc1.handle_command("STOP hold=1")
        mc1.control_tick()
        assert mc1.hold_state == 1, exit_cmd
        mc1.handle_command(exit_cmd)
        assert mc1.hold_state == 0, f"{exit_cmd} should not report hold=2"


def test_hold_broken_by_fault_reports_two(mc1):
    """A hold broken by a fault must be distinguishable from a normal stop — that
    is the moment a host should alert an operator."""
    mc1.handle_command("STOP hold=1")
    mc1.control_tick()
    mc1._raise_fault("ENCODER_STALL")
    assert mc1.hold_state == 2
    assert "hold=2" in mc1.handle_command("STATUS")
    # Sticky until the next command, then cleared.
    mc1.handle_command("ENABLE ON")
    mc1.handle_command("WHEEL l=1 r=1")
    assert mc1.hold_state == 0


def test_hold_times_out_and_brakes(mc1):
    """hold.max_ms is a THERMAL limit: holding on a gradient is a stalled motor,
    no back-EMF and no self-cooling. On expiry it brakes and reports."""
    mc1.handle_command("SET hold.max_ms=50")
    mc1.handle_command("STOP hold=1")
    mc1.control_tick()
    assert mc1.hold_state == 1
    time.sleep(0.08)
    mc1.control_tick()
    assert mc1.hold_state == 2
    assert mc1.mode == "IDLE"


def test_hold_zero_disables_the_timeout(mc1):
    mc1.handle_command("SET hold.max_ms=0")
    mc1.handle_command("STOP hold=1")
    time.sleep(0.05)
    for _ in range(20):
        mc1.control_tick()
    assert mc1.hold_state == 1


def test_hold_refused_is_reported_not_raised(mc1):
    """⚠️ ORCP v1.1 §STOP: "MUST be accepted regardless of safety state — STOP
    never fails." So a hold that cannot be honoured must NOT produce an ERR.

    Silence is not the alternative: the stop succeeds and the response says so
    explicitly, because a caller believing the robot is holding on a slope when
    nothing is holding it is the hazard the feature exists to prevent."""
    mc1.handle_command("ENABLE OFF")
    r = mc1.handle_command("STOP hold=1")
    assert r.startswith("OK STOP")
    assert "hold=refused reason=NOT_ENABLED" in r
    assert mc1.hold_state == 0

    mc1.handle_command("ENABLE ON")
    mc1.handle_command("SET kin.counts_per_rev=0")
    r = mc1.handle_command("STOP hold=1")
    assert r.startswith("OK STOP")
    assert "hold=refused reason=NO_ENCODERS" in r


def test_base_profile_rejects_hold_as_an_unknown_parameter(sim):
    """⚠️ ORCP v1.1 §4 preamble: "Conformant implementations MUST reject unknown
    bracketed parameters they do not recognise, rather than silently ignoring
    them, so a host writing against the spec cannot accidentally invoke
    vendor-specific behaviour on a different controller."

    This does NOT collide with §STOP's "STOP never fails", whose qualifier is
    *regardless of safety state* — an unrecognised parameter is not a safety
    state.

    ⚠️ An earlier version answered `OK STOP … hold=refused reason=UNSUPPORTED`.
    That satisfied the intent but not the letter, and it blurred two different
    answers: "I have this feature and cannot honour it now" versus "I have never
    heard of it". A host needs to tell those apart — the first is worth
    retrying, the second never will be."""
    r = sim.handle_command("STOP hold=1")
    assert r.startswith("ERR code=BAD_ARG")
    assert "unknown parameter" in r
    # …and a plain STOP is completely unaffected.
    assert sim.handle_command("STOP") == "OK STOP mode=BRAKE"
    assert "hold=" not in sim.handle_command("STATUS")


def test_unknown_stop_parameter_rejected_even_where_hold_is_supported(mc1):
    """The rule is about unrecognised keys, not about `hold` specifically."""
    r = mc1.handle_command("STOP mode=BRAKE wibble=3")
    assert r.startswith("ERR code=BAD_ARG") and "wibble" in r


def test_supported_hold_still_reports_a_runtime_refusal(mc1):
    """⚠️ The distinction being drawn: on a device that HAS the feature, a hold
    it cannot honour right now is still a successful STOP carrying
    hold=refused — not an error. Only the unknown-parameter case errors."""
    mc1.handle_command("ENABLE OFF")
    r = mc1.handle_command("STOP hold=1")
    assert r.startswith("OK STOP")
    assert "hold=refused reason=NOT_ENABLED" in r


def test_stop_accepts_the_spec_kv_form(mc1):
    """⚠️ ORCP v1.1 §STOP documents `STOP [mode=<vendor_mode>]`, matching
    WHEEL's `mode=DUTY`. Reading only bare arguments meant a host written
    against the published spec asked for a coast, got a brake, and was told
    `OK STOP mode=BRAKE` — the same failure class as a stop that discards its
    mode argument."""
    assert mc1.handle_command("STOP mode=COAST") == "OK STOP mode=COAST parking=auto"
    assert mc1.coasting

    assert mc1.handle_command("STOP mode=BRAKE") == "OK STOP mode=BRAKE"
    assert not mc1.coasting

    r = mc1.handle_command("STOP mode=COAST hold=1")
    assert r == "OK STOP mode=COAST parking=auto hold=on"
    assert mc1.hold_pending

    assert mc1.handle_command("STOP hold=0") == "OK STOP mode=BRAKE"
    assert mc1.handle_command("STOP mode=SIDEWAYS").startswith("ERR code=BAD_ARG")
    assert mc1.handle_command("STOP hold=maybe").startswith("ERR code=BAD_ARG")


def test_stop_bare_form_rejected_with_a_useful_message(mc1, sim):
    """⚠️ `STOP COAST` used to work. It was accepted for one release for
    compatibility with devices that shipped it — then removed, because no
    external host depended on it and two syntaxes for one command (one of them
    undocumented by ORCP) cost more than the compatibility was worth.

    The refusal must NAME the replacement: the old form is in a published
    datasheet, so someone will type it and needs telling what to type instead,
    not merely that they are wrong."""
    for s in (mc1, sim):
        for cmd in ("STOP COAST", "STOP BRAKE", "STOP HOLD"):
            r = s.handle_command(cmd)
            assert r.startswith("ERR code=BAD_ARG"), cmd
            assert "mode=BRAKE|COAST" in r, cmd
    # A bare STOP — the standard form — is untouched.
    assert mc1.handle_command("STOP") == "OK STOP mode=BRAKE"


def test_stop_rejects_unknown_bare_arg(mc1):
    assert mc1.handle_command("STOP SLOWLY").startswith("ERR code=BAD_ARG")


def test_mc1_aux5v_warn_push(tmp_path):
    # Over-current on the 5V rail raises ! WARN AUX5V (MC1 vendor push).
    s = ORCPSim(profile=MC1_PROFILE, config_file=str(tmp_path / "a.json"),
                aux5v_amps=6.0)        # warn_amps default 4.0
    s.pending_push.clear()
    s.control_tick()
    assert any(m.startswith("! WARN AUX5V state=warn") for m in s.pending_push)


def test_base_has_no_aux5v_warn(sim):
    sim.vbat = 12.4
    sim.pending_push.clear()
    sim.control_tick()
    assert not any("AUX5V" in m for m in sim.pending_push)


def test_websocket_transport(tmp_path):
    websockets = pytest.importorskip("websockets")
    import asyncio
    from orcp_sim import ORCPSim, _ws_run

    async def scenario():
        s = ORCPSim(level=2, config_file=str(tmp_path / "ws.json"))
        loop = asyncio.get_running_loop()
        ready = loop.create_future()
        stop = asyncio.Event()
        task = asyncio.create_task(_ws_run(s, "localhost", 0, ready=ready, stop=stop))
        try:
            port = await asyncio.wait_for(ready, 3)
            async with websockets.connect(f"ws://localhost:{port}") as ws:
                await ws.send("INFO\n")
                r = await asyncio.wait_for(ws.recv(), 3)
                assert "proto=ORCP/1.1" in r and "level=2" in r
                await ws.send("SET pid.kp=0.080\n")
                r2 = await asyncio.wait_for(ws.recv(), 3)
                assert "OK SET pid.kp=0.080" in r2
        finally:
            stop.set()
            await asyncio.wait_for(task, 3)

    asyncio.run(scenario())


# ---------------------------------------------------------------------------
# Vendor profile files (--profile-file)
# ---------------------------------------------------------------------------
import json
import os

from orcp_sim import load_profile_file, BASE_PROFILE

_EXAMPLE = os.path.join(os.path.dirname(__file__), "..", "docs", "example-profile.json")


def _base_profile_dict(**overrides):
    """A minimal valid profile dict (JSON-shaped) built from the standard keys."""
    d = {
        "name": "vendor",
        "identity": {"hw": "VENDOR-X", "fw": "1.0.0", "level": 2},
        "config": [[n, default, mn, mx] for (n, default, mn, mx) in BASE_PROFILE["config"]],
        "int_keys": list(BASE_PROFILE["int_keys"]),
        "warns": ["BATT"],
    }
    d.update(overrides)
    return d


def _write(tmp_path, data):
    p = tmp_path / "profile.json"
    p.write_text(json.dumps(data))
    return str(p)


def test_shipped_example_profile_loads_and_runs(tmp_path):
    prof = load_profile_file(_EXAMPLE)
    s = ORCPSim(profile=prof, config_file=str(tmp_path / "ex.json"))
    assert s.identity["hw"] == "ACME-DRIVE"
    assert s.identity["fw"] == "2.3.0"
    assert s.battery_display == "band"
    assert "acme.led_brightness" in s.cfg          # vendor key present
    # standard §7 keys still present
    assert "pid.kp" in s.cfg and "hb.timeout_ms" in s.cfg


def test_profile_file_coerces_sets_and_tuples(tmp_path):
    prof = load_profile_file(_write(tmp_path, _base_profile_dict()))
    assert isinstance(prof["int_keys"], set)
    assert isinstance(prof["warns"], set)
    assert all(isinstance(row, tuple) and len(row) == 4 for row in prof["config"])


def test_profile_file_defaults_optional_fields(tmp_path):
    prof = load_profile_file(_write(tmp_path, _base_profile_dict()))
    assert prof["battery"] == "percent"   # default
    assert prof["aux5v"] is False         # default
    assert prof["wheel_modes"] == []      # default


def test_profile_file_missing_field_rejected(tmp_path):
    bad = _base_profile_dict()
    del bad["identity"]
    with pytest.raises(ValueError, match="missing required field 'identity'"):
        load_profile_file(_write(tmp_path, bad))


def test_profile_file_identity_must_have_hw_fw_level(tmp_path):
    bad = _base_profile_dict(identity={"hw": "X", "fw": "1.0.0"})  # no level
    with pytest.raises(ValueError, match="identity must include 'level'"):
        load_profile_file(_write(tmp_path, bad))


def test_profile_file_bad_config_row_rejected(tmp_path):
    bad = _base_profile_dict()
    bad["config"].append(["too.short", 1.0, 0.0])   # 3 elements, not 4
    with pytest.raises(ValueError, match="config row must be"):
        load_profile_file(_write(tmp_path, bad))


def test_profile_file_missing_standard_key_rejected(tmp_path):
    bad = _base_profile_dict()
    bad["config"] = [row for row in bad["config"] if row[0] != "pid.kp"]
    with pytest.raises(ValueError, match="missing keys the core requires"):
        load_profile_file(_write(tmp_path, bad))


def test_mc1_ships_as_bundled_profile():
    """MC1 is discovered from profiles/mc1.json, not an in-module dict."""
    from orcp_sim import PROFILES
    assert "mc1" in PROFILES
    assert PROFILES["mc1"]["identity"]["hw"] == "MC1"
    assert len(PROFILES["mc1"]["config"]) == 63


def test_mc1_preset_slow_reports_timeout(mc1):
    """MC1 exposes slow.timeout_ms (default 0 = disabled); PRESET SLOW reports it."""
    assert "slow.timeout_ms" in mc1.cfg
    resp = mc1.handle_command("PRESET SLOW")
    assert "timeout_ms=0" in resp and "name=SLOW" in resp


def test_preset_slow_tolerates_missing_slow_timeout_key(tmp_path):
    """slow.timeout_ms is optional for vendor profiles (not a core-required
    key); a profile that omits it must default to 0, not crash, on PRESET SLOW."""
    prof = _base_profile_dict()
    prof["config"] = [r for r in prof["config"] if r[0] != "slow.timeout_ms"]
    s = ORCPSim(profile=load_profile_file(_write(tmp_path, prof)),
                config_file=str(tmp_path / "x.json"))
    assert "slow.timeout_ms" not in s.cfg
    assert "timeout_ms=0" in s.handle_command("PRESET SLOW")


# ---------------------------------------------------------------------------
# §4: unknown parameters must be REJECTED, not ignored
# ---------------------------------------------------------------------------

class TestUnknownParameterRejection:
    """⚠️ ORCP v1.1 §4 preamble: "Conformant implementations MUST reject unknown
    bracketed parameters they do not recognise, rather than silently ignoring
    them, so a host writing against the spec cannot accidentally invoke
    vendor-specific behaviour on a different controller."

    ⚠️ **This matters more in the simulator than in a product.** `--profile base`
    is what someone runs to check their host is conformant. A simulator that
    ignores an unknown parameter does not merely tolerate a bad host — it
    CERTIFIES one, telling the author their code is fine when it is asking for
    behaviour no controller implements. Every command below answered OK before
    this was fixed.
    """

    @pytest.mark.parametrize("cmd,bad", [
        ("WHEEL l=1 r=1 torque=5", "torque"),
        ("CMD_VEL v=0.1 w=0 accel=9", "accel"),
        ("STREAM ON 10 fmt=json", "fmt"),
        ("PRESET SLOW turbo=1", "turbo"),
        ("ENABLE ON force=1", "force"),
        ("STATUS verbose=1", "verbose"),
    ])
    def test_unknown_parameter_is_rejected(self, sim, cmd, bad):
        r = sim.handle_command(cmd)
        assert r.startswith("ERR code=BAD_ARG"), cmd
        assert bad in r

    @pytest.mark.parametrize("cmd", [
        "WHEEL l=1 r=1", "WHEEL l=1 r=1 mode=DUTY", "CMD_VEL v=0.1 w=0",
        "STREAM ON 10", "PRESET SLOW", "ENABLE ON",
        "STOP", "STOP mode=COAST", "STATUS", "GET pid.kp", "GET ALL",
    ])
    def test_legitimate_forms_still_accepted(self, sim, cmd):
        """The guard must not become stricter than the spec — every standard
        and documented-vendor form still has to work."""
        assert not sim.handle_command(cmd).startswith("ERR"), cmd

    def test_set_takes_exactly_one_parameter(self, sim):
        """⚠️ SET is exempt from the CENTRAL check because its key is a config
        parameter name — but exempt must not mean unchecked. It took the first
        pair and dropped the rest, so `SET pid.kp=0.1 scope=all` succeeded while
        silently ignoring `scope`: the same footgun, in the one command that
        opted out of the general guard."""
        assert sim.handle_command("SET pid.kp=0.1") == "OK SET pid.kp=0.100"
        r = sim.handle_command("SET pid.kp=0.1 scope=all")
        assert r.startswith("ERR code=BAD_ARG") and "scope" in r
        r = sim.handle_command("SET pid.kp=0.1 ALL")
        assert r.startswith("ERR code=BAD_ARG") and "ALL" in r
        # An unknown config key is still its own, distinct rejection.
        assert "nonsuch" in sim.handle_command("SET nonsuch=1")

    def test_stream_rate_is_a_bare_argument_not_a_parameter(self, sim):
        """⚠️ §STREAM's syntax is `STREAM <ON|OFF> [rate]` — the rate is BARE.

        The simulator used to accept `rate=` as well. That is a non-standard
        form, and a reference implementation accepting it certifies a host the
        spec does not. Found while sweeping §4 across the command surface, by
        checking the firmware (which only ever accepted the bare form) against
        the simulator and asking which one was right."""
        assert sim.handle_command("STREAM ON 10").startswith("OK STREAM")
        r = sim.handle_command("STREAM ON rate=10")
        assert r.startswith("ERR code=BAD_ARG") and "rate" in r

    def test_vendor_key_allowed_only_where_the_profile_declares_it(self, mc1, sim):
        """`hold` is a real parameter on MC1 and an unknown one on base — the
        guard is profile-aware, not a fixed list."""
        assert mc1.handle_command("STOP hold=1").startswith("OK STOP")
        assert sim.handle_command("STOP hold=1").startswith("ERR code=BAD_ARG")


def test_profile_declares_what_it_does_not_model(mc1, sim):
    """⚠️ A profile that emulates a real device must say what it leaves out.

    The mc1 profile implements 15 of the controller's 20 commands. Without this
    declaration a vendor tool reaching for CAPS or SERIAL gets a bare
    `ERR code=BAD_CMD` and looks like a broken tool, rather than a simulator
    that was never asked to model it. Purely declarative — it changes no
    behaviour, and the commands still (correctly) ERR."""
    from orcp_sim import PROFILES
    nm = PROFILES["mc1"]["not_modelled"]
    assert "SERIAL" in nm["commands"] and "CAPS" in nm["commands"]
    assert "il" in nm["status_fields"]
    # The declaration must be honest in BOTH directions: nothing listed as
    # unmodelled may actually be implemented.
    for cmd in nm["commands"]:
        assert not hasattr(mc1, f"_cmd_{cmd}"), f"{cmd} is listed as unmodelled but exists"
        assert mc1.handle_command(cmd).startswith("ERR code=BAD_CMD")
    status = mc1.handle_command("STATUS")
    for f in nm["status_fields"]:
        assert f"{f}=" not in status, f"{f} is listed as unmodelled but is reported"
    # `base` models the standard in full, so it disclaims nothing.
    assert PROFILES["base"]["not_modelled"] == {"commands": [], "status_fields": []}


# ---------------------------------------------------------------------------
# Differential-drive geometry — arc scaling and paired ramping
#
# Regression tests for the defect fixed in MC1 firmware 1.15.0 and mirrored
# here: commanded arcs straightened out as speed rose, and the robot twisted
# as it stopped. Both came from treating the two wheels as independent when
# they are not — their RATIO is the turn radius.
# ---------------------------------------------------------------------------

def _wheels(resp):
    """Pull the two wheel targets out of an OK CMD_VEL / OK WHEEL response."""
    kv = dict(re.findall(r"(\w+)=(-?[\d.]+)", resp))
    if "wl" in kv:
        return float(kv["wl"]), float(kv["wr"])
    return float(kv["l"]), float(kv["r"])


def _radius(l, r, track=0.175):
    """Turn radius implied by a wheel pair.

    wheel_radius cancels — R = (l + r)·track / (2·(r − l)) — so this is also
    invariant to a common scale error on both wheels, which is exactly the
    property that makes it a fair test of geometry alone.
    """
    return float("inf") if abs(r - l) < 1e-9 else (l + r) * track / (2 * (r - l))


def _expected_wheels(sim, v, w):
    """Unicycle → wheel targets using the simulator's OWN kinematics.

    ⚠️ Don't hard-code these: the base profile is not the MC1 profile (it uses
    wheel_radius 0.049, not 0.050), so literal expectations silently encode the
    wrong robot.
    """
    tw = sim.cfg["kin.track_width"]
    wr = sim.cfg["kin.wheel_radius"]
    return (v - w * tw / 2.0) / wr, (v + w * tw / 2.0) / wr


def test_arc_ratio_held_when_over_the_ceiling(sim):
    """⭐ The whole defect in one assertion.

    SLOW caps at 0.30 duty, so the ceiling is 0.30 × 20.9 = 6.27 rad/s. This
    arc asks for wheel targets of 4.69 / 7.31 — the outer wheel is over the
    ceiling. Clamping each wheel independently gave 4.69 / 6.27, a ratio of
    1.34 against the 1.56 commanded, which on a floor is a visibly wider arc.
    """
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    l, r = _wheels(sim.handle_command("CMD_VEL v=0.30 w=0.750"))
    assert r == pytest.approx(6.27, abs=0.01), "outer wheel should sit at the ceiling"
    assert r / l == pytest.approx(1.56, abs=0.01), "commanded ratio must survive"
    assert _radius(l, r) == pytest.approx(0.40, abs=0.01)


def test_arc_does_not_straighten_when_both_wheels_are_over(sim):
    """⚠️ The worst case: independent clamping pinned BOTH wheels to the same
    value, so the robot drove dead straight while still echoing w=1.000."""
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    l, r = _wheels(sim.handle_command("CMD_VEL v=1.00 w=1.000"))
    assert l != pytest.approx(r), "both wheels pinned to the ceiling = straight line"
    assert r / l == pytest.approx(1.1918, abs=0.001)


def test_arc_shape_is_speed_invariant(sim):
    """The property customers actually notice: the same commanded radius must
    come out the same shape at every speed, whether or not scaling engages."""
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    radii = []
    for v in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
        radii.append(_radius(*_wheels(
            sim.handle_command(f"CMD_VEL v={v:.2f} w={v / 0.40:.4f}"))))
    assert max(radii) - min(radii) < 0.005, f"radius drifts with speed: {radii}"


def test_under_the_ceiling_is_untouched(sim):
    """Scaling must not alter a pair that already fits."""
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    exp_l, exp_r = _expected_wheels(sim, 0.20, 0.500)
    assert max(abs(exp_l), abs(exp_r)) < sim.duty_limit * 20.9, \
        "test precondition: this pair must already fit under the ceiling"
    l, r = _wheels(sim.handle_command("CMD_VEL v=0.20 w=0.500"))
    assert l == pytest.approx(exp_l, abs=0.001)
    assert r == pytest.approx(exp_r, abs=0.001)


def test_wheel_vel_mode_scales_as_a_pair_too(sim):
    """WHEEL mode=VEL shares the ceiling, so it shared the defect. Host-side
    kinematics must not be a way to reach the broken path."""
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    l, r = _wheels(sim.handle_command("WHEEL l=4.688 r=7.312"))
    assert r / l == pytest.approx(1.56, abs=0.01)


def test_spin_in_place_stays_symmetric(sim):
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    l, r = _wheels(sim.handle_command("CMD_VEL v=0.00 w=2.000"))
    assert l == pytest.approx(-r, abs=1e-6)


def test_ramp_holds_the_ratio_and_lands_together(sim):
    """⚠️ The twist-on-release half. Ramping each wheel independently let the
    inner wheel reach its target — and later zero — before the outer one, so
    the robot curled into and out of every arc, worse the faster it was going.
    """
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    sim.handle_command("CMD_VEL v=0.30 w=0.750")
    target_ratio = sim.target_r / sim.target_l

    seen = []
    for _ in range(400):
        sim.control_tick()
        if abs(sim.ramped_target_l) > 0.2:
            seen.append(sim.ramped_target_r / sim.ramped_target_l)
        if sim.ramped_target_r == pytest.approx(sim.target_r, abs=1e-6):
            break
    assert seen, "ramp never moved"
    assert max(abs(x - target_ratio) for x in seen) < 0.01, \
        "ratio drifted during the ramp"

    # ...and on the way back down, both must reach zero on the same tick.
    sim.handle_command("CMD_VEL v=0.00 w=0.000")
    zero_l = zero_r = None
    for i in range(400):
        sim.control_tick()
        if zero_l is None and sim.ramped_target_l == 0.0:
            zero_l = i
        if zero_r is None and sim.ramped_target_r == 0.0:
            zero_r = i
        if zero_l is not None and zero_r is not None:
            break
    assert zero_l == zero_r, f"wheels stopped {abs(zero_r - zero_l)} ticks apart"


def test_cmd_vel_echoes_the_accepted_target_not_the_request(sim):
    """ORCP v1.1 §CMD_VEL: v/w are the "accepted target after any clamping".

    ⚠️ When scaling engages, the request and the acceptance differ — and that
    difference is precisely what a host needs in order to know the drivetrain
    could not do what it was asked. Echoing the request hides it.
    """
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    r = sim.handle_command("CMD_VEL v=1.00 w=1.000")
    kv = dict(re.findall(r"(\w+)=(-?[\d.]+)", r))
    v, w = float(kv["v"]), float(kv["w"])
    wl, wrr = float(kv["wl"]), float(kv["wr"])

    assert v < 1.00, "v must report the reduced, achievable speed"
    # The echoed v/w must be exactly what the echoed wheel targets mean.
    tw = sim.cfg["kin.track_width"]
    rad = sim.cfg["kin.wheel_radius"]
    assert v == pytest.approx((wl + wrr) / 2 * rad, abs=1e-3)
    assert w == pytest.approx((wrr - wl) * rad / tw, abs=1e-3)
    # ⭐ The commanded SHAPE survives even though the speed does not.
    assert v / w == pytest.approx(1.00 / 1.000, abs=0.01)


def test_cmd_vel_echo_unchanged_when_nothing_is_clamped(sim):
    """A request that already fits must echo back identically."""
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    r = sim.handle_command("CMD_VEL v=0.20 w=0.500")
    kv = dict(re.findall(r"(\w+)=(-?[\d.]+)", r))
    assert float(kv["v"]) == pytest.approx(0.20, abs=1e-3)
    assert float(kv["w"]) == pytest.approx(0.500, abs=1e-3)


def test_wheel_vel_echoes_the_accepted_value(sim):
    """§WHEEL carries the same wording for l/r."""
    sim.handle_command("PRESET SLOW")
    sim.handle_command("ENABLE ON")
    r = sim.handle_command("WHEEL l=4.688 r=7.312")
    kv = dict(re.findall(r"(\w+)=(-?[\d.]+)", r))
    assert float(kv["r"]) == pytest.approx(6.27, abs=0.01), "accepted, not requested"
    assert float(kv["r"]) / float(kv["l"]) == pytest.approx(1.56, abs=0.01)
