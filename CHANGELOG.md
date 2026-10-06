# orcp-sim — Changelog

---

## Unreleased

### ⚠️ Arcs straightened out as speed rose, and the robot curled as it stopped

Both halves of the defect fixed in **MC1 firmware 1.15.0**, mirrored here. The
simulator had them independently — it is a reference implementation, and it was
reproducing the bug rather than the standard.

**Wheel-target pairs are now scaled, not clamped individually.** `CMD_VEL` and
`WHEEL mode=VEL` both limit targets to `duty_limit × MAX_MOTOR_RADS`. Clamping
each wheel against that ceiling separately changes `left:right` — and for a
differential drive that ratio **is** the turn radius. An arc whose outer wheel
exceeded the ceiling came out wider than asked; once **both** exceeded it they
pinned to the same value and the robot drove **dead straight while still
echoing the commanded `w`**:

```
before   CMD_VEL v=1.00 w=1.000  ->  wl=6.270 wr=6.270   identical. straight.
after    CMD_VEL v=1.00 w=1.000  ->  wl=5.261 wr=6.270   radius 1.000 m
before   CMD_VEL v=0.30 w=0.750  ->  wl=4.783 wr=6.270   radius 0.650 m
after    CMD_VEL v=0.30 w=0.750  ->  wl=4.019 wr=6.270   radius 0.400 m
```

Scaling gives up speed and keeps the commanded path, which is the correct trade:
the caller asked for a shape, and speed is the part physically unavailable.
Nothing saturates at low speed, which is why this only ever showed above a
threshold — and why it survived so long.

**The acceleration ramp now advances both wheels as a pair.** `kin.max_accel`
was applied to each wheel independently, so the wheel with less distance to
cover arrived first and the ratio drifted throughout every acceleration and
deceleration. On release the inner wheel reached zero while the outer was still
turning, so the robot curled at both ends of every arc, worse the faster it was
going. Both wheels now travel the straight line to their targets and land on the
same tick.

📋 The simulator did **not** share the firmware's third defect: its
`kin.max_accel = 0` handling was already correct.

⚠️ **Note for anyone matching a real controller:** correct targets are not
sufficient. If `motor.max_rads` overstates what the drivetrain delivers, the
ceiling is unreachable, both wheels saturate their duty limit and the *actual*
ratio collapses toward 1.0 — the same symptom, downstream of this fix. On the
MC1 reference chassis the shipped default overstated it ~2×, and a commanded
0.400 m radius was driven as 5.423 m until it was measured and corrected.

Eight regression tests added, including a speed-invariance check (the same
commanded radius must come out the same shape at every speed) and a paired-ramp
check (ratio held throughout, both wheels reaching zero on the same tick).

### Fixed: `test_mc1_identity` asserted a stale profile version

Asserted `fw=1.13.3` after the profile below was re-synced to **1.14.3**, so the
suite has been one test red since that change. 📋 The real MC1 is now on
**1.15.0** — the profile is due another re-sync, which is a separate change.


### `mc1` profile re-synced to firmware 1.14.3

Was declaring **1.13.3**. Three changes, no key-surface change — the profile
still models all **63** keys and `CONFIG` is still 25:

* `identity.fw` → **1.14.3**.
* ⚠️ **`kin.wheel_radius` default `0.049` → `0.050`.** The firmware changed this
  in **1.14.1** and the profile had not followed. It is a **2 % odometry scale
  error**, so anyone comparing simulated odometry against a real board would
  have seen a genuine, small, and very confusing discrepancy.
* ⚠️ **`active` added to `not_modelled.status_fields`.** The real MC1 emits
  `active=0xNNNN` — a bitmask of live safety conditions — in **every** `STATUS`.
  The simulator does not model it, which is fine, but it was the **one gap that
  was not declared**: it read as a missing field rather than a stated
  limitation. Now announced at start-up with the other fourteen.

📋 **R-STOP is deliberately still not modelled.** `reflex` was already in the
declared list and stays there — the firmware gained a real `REFLEX` fault in
1.14.0, but the input is a hardware line a simulator has no way to represent.

---

## 0.2.0 — 2026-08-27

### ⚠️ Behaviour change: unknown parameters are now rejected

`base` and every profile now **reject unrecognised `key=value` parameters**
instead of ignoring them, per ORCP v1.1 §4:

> *"Conformant implementations MUST reject unknown bracketed parameters they do
> not recognise, rather than silently ignoring them, so a host writing against
> the spec cannot accidentally invoke vendor-specific behaviour on a different
> controller."*

**This will fail host code that previously passed**, which is precisely the
point — and precisely why it is a surprise. `WHEEL l=1 r=1 torque=5`,
`ENABLE ON force=1` and `CMD_VEL v=0.1 w=0 accel=9` all used to answer `OK`.

⚠️ **A simulator that ignores an unknown parameter does not merely tolerate a
bad host — it CERTIFIES one**, telling the author their code is fine while it
asks for behaviour no controller implements. `--profile base` is what people run
to check conformance, so getting this wrong is worse here than in a product.

`SET` additionally rejects more than one parameter.

**Also removed: `STREAM ON rate=10`.** §STREAM's syntax is
`STREAM <ON|OFF> [rate]` — the rate is a *bare* argument. The kv form was
non-standard and accepting it certified hosts the spec does not. Bare
`STREAM ON 10` is unaffected.

### Added

* **Three profile capability fields**, all defaulting off so `base` stays a clean
  ORCP v1.1 reference: `config_decimals` (fractional digits in `GET`),
  `coast_park` (`STOP COAST` rolls to rest then parks), `stop_hold`
  (`STOP … hold=` and the `STATUS hold=` field).
* **`not_modelled`** — a profile declares what it does *not* reproduce, announced
  at start-up. The MC1 profile implements 15 of the controller's 20 commands;
  without this, a vendor tool reaching for `CAPS` or `SERIAL` gets a bare
  `ERR code=BAD_CMD` and looks broken rather than unmodelled. A test asserts the
  declaration is honest in both directions — ⚠️ *a stale disclaimer would be
  worse than none, because it would be believed.*
* `STOP [mode=…] [hold=…]`, coast-and-park, and the position-hold model.

### Fixed — the MC1 profile had drifted five firmware releases

Regenerated **from `App/Src/config.c`** rather than hand-maintained: 46 → **63
keys**, tracking FW 1.13.3 / CONFIG 25.

⚠️ **It still offered `current.scale`**, which the firmware split per-side at
CONFIG 24 — so host code written against the simulator would `SET` a key that
`ERR`s on a real board. **A "missing keys" check would never have caught that**,
which is why the drift check must fail on *removed* keys too.

Also: 13 wrong defaults, **the motor and encoder invert flags reversed on all
four channels** (anyone establishing wheel direction against the simulator got
the opposite of a real board), `battery` rendering as a band label when the MC1
reports a percentage, and `GET` at 3 decimals when the MC1 moved to 6.

📋 The profile and its own test asserted the same wrong battery value, which is
how it survived. Tests written against the thing they test do not check it.

### ⚠️ Known limitation

A vendor profile models the **protocol surface, not the whole command set**. See
`docs/vendor-surface-gap.md`.
