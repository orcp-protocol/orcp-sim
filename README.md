# orcp-sim

Reference simulator for the [Open Robot Control Protocol (ORCP)](https://github.com/orcp-protocol/orcp).

`orcp-sim` emulates an ORCP-compliant motor controller on a virtual serial port
(PTY). Point any ORCP host — a terminal, the [`orcp` Python
library](https://github.com/orcp-protocol/orcp-python), or a ROS 2 node — at the
device path it prints, and develop against the protocol with no hardware.

It implements **ORCP v1.1** and runs a 100 Hz control loop with PID, a motor
model, encoder quantisation, the mandatory safety system, and streaming
telemetry.

## Requirements

Python 3.9+. No third-party dependencies — standard library only. (PTY support
is Unix-only: macOS and Linux.)

## Install

```bash
# Run straight from a checkout — no install needed:
python3 orcp_sim.py

# …or install the `orcp-sim` command:
pip install -e .
orcp-sim
```

## Usage

```bash
orcp-sim                      # Level 2 controller, auto-created PTY
orcp-sim --level 1            # Level 1 controller
orcp-sim --link /tmp/orcp     # also create a symlink at this path
orcp-sim --vbat 11.1          # set simulated battery voltage
orcp-sim --estop              # start with the e-stop active
```

On start it prints the serial device path:

```
ORCP Reference Simulator — proto ORCP/1.1, fw 1.1.0, Level 2
Serial device: /dev/ttys004
```

Connect with any serial terminal (e.g. `screen /dev/ttys004 115200`) and type
commands:

```
PING
<<< OK PONG t=1234
INFO
<<< OK INFO fw=1.1.0 hw=ORCP-SIM proto=ORCP/1.1 vendor=ORCP-Project model=reference-sim level=2 uuid=00SIMULATED00FF
PRESET NORMAL
<<< OK PRESET name=NORMAL timeout_ms=250 enable_required=1 hb_required=1 duty_limit=0.900
ENABLE ON
<<< OK ENABLE state=ON
WHEEL l=5.0 r=5.0
<<< OK WHEEL l=5.000 r=5.000
```

## Conformance levels

One simulator emulates any conformance level (ORCP spec §9); select it with
`--level`. The controller declares its level in the `INFO` response, exactly as
real hardware does, so a host can be tested against each tier by changing one
flag. Commands above the active level return `ERR code=BAD_CMD`.

| Level | `--level` | Commands available |
|-------|-----------|--------------------|
| 1 — Basic    | `1`         | `PING CMD_VEL WHEEL STOP STATUS ENABLE PRESET` (motion + mandatory safety) |
| 2 — Standard | `2` (default) | Level 1 **+** `INFO HB STREAM GET SET SAVE LOAD DEFAULTS` (runtime config with persistence, heartbeat & e-stop monitoring, telemetry streaming) |
| 3 — Extended | `3`         | CAN binary encoding is not emulated over a PTY; behaves as Level 2 on this ASCII transport |

Notable v1.1 behaviours the simulator demonstrates:

- `WHEEL` defaults to **rad/s closed-loop** velocity; open-loop duty is opt-in
  via the vendor extension `mode=DUTY`.
- Closed-loop commands are rejected with `ERR code=NO_FEEDBACK` when
  `kin.counts_per_rev=0` (no-encoder platform).
- Heartbeat monitoring is a **Level 2+** feature: the full preset reports
  `hb_required=1` at Level 2 and `hb_required=0` at Level 1, where the command
  timeout provides the dead-man.
- Safety faults latch until `ENABLE ON`; `! FAULT` and `! WARN` push messages
  are emitted on transitions.

## Profiles

A **profile** makes the simulator emulate a specific controller — its identity,
config-key surface, vendor command modes, and push events — layered over the
generic v1.1 core. Select one with `--profile`:

| Profile | What it emulates |
|---------|------------------|
| `base` (default) | Generic, fully-conformant ORCP v1.1 controller — the standard reference (15 standard §7 config keys). |
| `mc1` | First Layer Robotics MC1: the full 42-key config surface, `hw=MC1` / `bl=` in `INFO`, band-label battery, and the `! WARN AUX5V` vendor push. |

```bash
orcp-sim --profile mc1 --ws 8765      # emulate an MC1 over WebSocket
orcp-sim --profile mc1 --aux5v-amps 6 # trip the 5V-rail warning (! WARN AUX5V)
```

The `base` profile is the pure standard reference and is kept free of any
vendor-specific surface. New profiles (including third-party hardware) are a
planned extension; today the registry is built in.

## Connecting from a browser (web configurator)

Browsers reach real boards over the Web Serial API (USB CDC) and **cannot open a
PTY**, so to drive the simulator from a browser tool — such as the MC1 web
configurator — run it over a WebSocket instead:

```bash
pip install 'orcp-sim[web]'
orcp-sim --ws 8765            # serve on ws://localhost:8765
```

The WebSocket carries the identical ORCP line protocol, so the host only needs a
small transport switch (Web Serial for hardware, WebSocket for the simulator).
See [docs/websocket-transport.md](docs/websocket-transport.md) for the full
contract and a reference client.

## Configuration persistence

`SAVE`/`LOAD` persist the standard configuration parameters (ORCP §7) to a JSON
file. By default this is `orcp_sim_config.json` in the system temp directory;
override with `--config-file PATH`. `DEFAULTS` restores factory values.

## Tests

```bash
pip install -e ".[dev]"
pytest
```

The suite checks v1.1 compliance (response formats, error codes, level gating)
and the dynamic safety behaviour (heartbeat/command-timeout faults, e-stop
latching, streaming).

## License

MIT — see [LICENSE](LICENSE).
