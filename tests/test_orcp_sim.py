"""Compliance and behaviour tests for the ORCP reference simulator (ORCP v1.1)."""
import time
import pytest

from orcp_sim import ORCPSim


@pytest.fixture
def sim(tmp_path):
    """A Level 2 simulator with an isolated persistent-config file."""
    return ORCPSim(level=2, config_file=str(tmp_path / "cfg.json"))


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
