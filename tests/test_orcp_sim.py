"""Compliance and behaviour tests for the ORCP reference simulator (ORCP v1.1)."""
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
    assert "bl=1.1.1" in r
    assert "fw=1.7.0" in r
    assert "level=2" in r
    assert "vendor=" not in r          # MC1 INFO carries no vendor/model fields


def test_mc1_full_key_surface(mc1):
    assert len(mc1.cfg) == 43
    # the key that originally failed against the generic sim:
    assert mc1.handle_command("SET batt.hyst_v=0.3") == "OK SET batt.hyst_v=0.300"
    assert mc1.handle_command("GET batt.hyst_v") == "OK GET batt.hyst_v=0.300"
    # a few more MC1-only keys:
    for key in ("aux5v.warn_amps", "current.offset_left", "motor.i_max", "current_loop.enable"):
        assert mc1.handle_command(f"GET {key}").startswith("OK GET " + key + "=")


def test_mc1_renders_all_values_as_float(mc1):
    # MC1 has no int_keys — counts_per_rev comes back as 2249.000, not 2249.
    assert mc1.handle_command("GET kin.counts_per_rev") == "OK GET kin.counts_per_rev=2249.000"


def test_mc1_status_battery_is_band(mc1):
    assert mc1.handle_command("STATUS").rstrip().endswith("battery=OK")


def test_mc1_get_all_is_43_keys(mc1):
    r = mc1.handle_command("GET ALL")
    assert r.startswith("OK GET ")
    assert r.count("=") == 43


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
    assert len(PROFILES["mc1"]["config"]) == 43


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
