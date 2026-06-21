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
python3 -m orcp_sim

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

The simulator emulates all conformance levels (ORCP spec §9); select one with
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
generic v1.1 core.

A profile is **declarative data**, not code. Exactly one profile — `base`, the
standard reference — is built into the core; every other profile (including
MC1) is a JSON data file. That keeps the simulator vendor-neutral: no
manufacturer is privileged in the core, and adding a device is a data change,
not a code change.

Select a bundled profile by name with `--profile`, or load your own file with
`--profile-file`:

| Profile | Ships as | What it emulates |
|---------|----------|------------------|
| `base` (default) | built into the core | Generic, fully-conformant ORCP v1.1 controller — the standard reference (15 standard §7 config keys). |
| `mc1` | `orcp_sim/profiles/mc1.json` | First Layer Robotics MC1: the full 42-key config surface, `hw=MC1` / `bl=` in `INFO`, band-label battery, and the `! WARN AUX5V` vendor push. |

```bash
orcp-sim --profile mc1 --ws 8765      # emulate an MC1 over WebSocket
orcp-sim --profile mc1 --aux5v-amps 6 # trip the 5V-rail warning (! WARN AUX5V)
```

The `base` profile is the pure standard reference and is kept free of any
vendor-specific surface.

### Creating a profile (third-party vendors)

There are three ways to ship a profile, in increasing order of permanence — all
use the same JSON format:

| Tier | How | Selector |
|------|-----|----------|
| **Private / local** | Keep your own JSON file anywhere | `--profile-file acme.json` |
| **Bundled (contributed)** | Add `orcp_sim/profiles/acme.json` and open a PR — a **data** change, reviewed and shipped in the next release | `--profile acme` |
| **Independent** *(planned)* | A pip entry-point plugin, for profiles that need custom **code** | `--profile acme` |

MC1 is the worked example of the bundled tier — copy `orcp_sim/profiles/mc1.json`
and you have a vendor profile that ships first-class, exactly like it. For a
private profile:

```bash
orcp-sim --profile-file acme.json          # emulate your controller
orcp-sim --profile-file acme.json --ws 8765 # …and serve it to a browser tool
```

The fields are:

| Field | Required | Meaning |
|-------|----------|---------|
| `name` | yes | Short profile name (the `--profile` selector and the start-up banner) |
| `identity` | yes | `INFO` fields — must include `hw`, `fw`, `level`; optional `vendor`, `model`, and `info_extra` (e.g. `{"bl": "1.0.0"}`) |
| `config` | yes | Ordered `[name, default, min, max]` rows — drives `GET`/`SET`/`GET ALL`. Must include the config keys the core requires (the §7 essentials); add vendor keys freely |
| `int_keys` | no | Config keys rendered as integers (others as `%.3f`) |
| `wheel_modes` | no | Vendor `WHEEL` modes beyond rad/s, e.g. `["DUTY"]` |
| `warns` | no | `! WARN <type>` events the device emits, e.g. `["BATT"]` |
| `battery` | no | STATUS battery field: `"percent"` (default) or `"band"` |
| `aux5v` | no | `true` to model a 5 V aux rail + `! WARN AUX5V` |

A complete example is provided at
[docs/example-profile.json](docs/example-profile.json) — copy it and edit. The
loader validates on start-up and reports the common mistakes clearly (missing
field, malformed `config` row, or a dropped key the core requires).

#### Getting your profile bundled

To turn a private profile into a first-class `--profile <name>` that ships with
the simulator, contribute it back. The process:

1. **Write and validate it locally.** Copy `docs/example-profile.json`, edit it
   to match your controller, and prove it loads:
   ```bash
   orcp-sim --profile-file your-device.json     # must start without error
   ```
   Connect a host (a terminal, the `orcp` library, the web configurator) and
   confirm `INFO`, `GET ALL`, and a `PRESET`/`WHEEL` exchange look right. Aim to
   mirror your **shipping firmware's** `GET ALL` output exactly — same keys, same
   order, same defaults — so the profile is a faithful stand-in for the device.
2. **Fork** [`orcp-protocol/orcp-sim`](https://github.com/orcp-protocol/orcp-sim)
   and add your file as `orcp_sim/profiles/<name>.json`. Use a short, unique,
   lowercase `name` (it becomes the `--profile` selector); don't touch `base` or
   another vendor's profile.
3. **List it** by adding a row to the profile table in this README.
4. *(Encouraged)* **Add a quick test** in `tests/` — e.g. assert your profile is
   discovered and reports the identity / key-count you expect (see the `mc1`
   tests for the pattern). The loader already validates every bundled file on
   import, so a broken profile fails the suite.
5. **Open a pull request.** CI runs the test suite (which loads and validates
   every `profiles/*.json`). A maintainer reviews it as a data change — the bar
   is "does it accurately represent a real, shipping ORCP controller?". On merge
   it's in the next release and selectable as `--profile <name>` everywhere.

Because a bundled profile is pure data and the loader validates it on import,
accepting one is low-risk: it cannot run vendor code or affect the `base`
reference or any other profile.

**Scope:** a file profile describes a controller's *declarative surface* —
identity, config keys, which standard behaviours are switched on. It cannot add
genuinely new behaviour (custom telemetry physics, bespoke commands, new warn
*trigger logic*); those live in the simulator core. A pip entry-point plugin
mechanism for profiles that need custom code is a planned future direction.

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
