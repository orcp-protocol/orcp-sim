# orcp-sim — Changelog

---

## Unreleased

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
