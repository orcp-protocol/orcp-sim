# Profiles cannot express a vendor's *command* surface

**Design note · 2026-08-17 · orcp-sim 0.2.0**
Status: **problem statement + options**, no implementation. Written after a
firmware↔simulator conformance comparison found the gap.

---

## 1. What the gap is

A profile describes a controller's declarative surface: identity, config keys,
wheel modes, warn types, battery rendering, and the capability flags added in
0.2.0 (`config_decimals`, `coast_park`, `stop_hold`). The README already says a
profile *"cannot add genuinely new behaviour … bespoke commands"*.

That scoping is right for `base`. It is **wrong for a vendor profile that claims
to emulate a specific controller**, and the MC1 profile now demonstrates the
cost.

## 2. How it was found, and why that method matters

The two implementations were compared **against the spec**, not against each
other: per-command parameter sets extracted from `App/Src/command.c` and from
`COMMAND_KV_KEYS`, then diffed.

⚠️ **Comparing implementations to each other would have found none of this.**
Both were self-consistent. The `STREAM rate=` divergence fixed in the same pass
turned up the same way — the simulator accepted a form the spec does not define,
and only reading §STREAM settled which side was wrong. **Worth repeating for
other commands, and worth preferring over "do they agree?" as a check.**

## 3. The measured gap (MC1 profile vs FW 1.13.2)

**Commands the firmware has and the simulator does not — 5:**

| Command | What it does | Modelling it |
|---|---|---|
| `ENC` | read/reset encoder totals | Easy and valuable — the sim already tracks counts |
| `CAPS` | declare board capabilities | Easy — static, per-profile data |
| `SERIAL` | read/write the board serial number | Easy — static, per-profile data |
| `PID` | legacy gain get/set | Easy — maps onto existing config keys |
| `BOOTLOADER` | reset into the bootloader | ⚠️ Questionable in a simulator; see §5 |

⚠️ **Consequence: three production tools cannot run against the simulator at
all** — `mc1_soak.py`, `mc1_bringup.py` and `mc1_flash.py` use `SERIAL` / `CAPS`
/ `BOOTLOADER` and die on the first call. These are exactly the tools it would
be most useful to exercise without hardware, because they are the ones run
against a board that has just been built.

**STATUS fields the firmware emits and the simulator does not — ~12:**
`il`, `ir`, `isense`, `casc`, `eff_lim`, `mcu_temp`, `aux5v_v`, `aux5v_i`,
`aux5v_warn`, `encstall_l/r`, `mfault_l/r`.

A host written to read motor current or the cascade state from `STATUS` gets
nothing from the simulator and no indication why.

## 4. Why this is one gap, not two

Both follow from the same thing: **the profile schema can only switch on
behaviour the core already implements.** It has no vocabulary for "this device
also answers `X`" or "this device also reports field `y`". So every vendor
extension beyond the ones anticipated in the core requires a core change — which
is precisely what "profiles are data, not code" was meant to avoid.

⚠️ **It will recur with the next extension.** `coast_park` and `stop_hold` were
added as core flags because there was nowhere else to put them; the next two
will be too, and the core accumulates one boolean per vendor feature until it is
a list of everyone's product decisions.

## 5. Options

**A. Static response tables in the profile.** Let a profile declare commands
whose reply is fixed or templated:

```json
"commands": {
  "CAPS":   { "response": "OK CAPS motors=2 encoders=2 aux5v=1" },
  "SERIAL": { "response": "OK SERIAL {serial}", "vars": { "serial": "MC1-2632-00002" } }
}
```

Covers `CAPS` and `SERIAL` — the two blocking the bring-up tools — with no core
change and no vendor code. Does **not** cover `ENC`, which must read live state.

**B. Declarative STATUS extras.** Let a profile name additional STATUS fields
and bind them to simulator state or a constant:

```json
"status_extra": {
  "eff_lim": "duty_limit", "il": "0.000", "ir": "0.000", "casc": "0"
}
```

⚠️ **Constants are a trap here.** A field that always reads `0.000` is worse than
an absent one: absent is honestly "not modelled", whereas `il=0.000` looks like a
measurement and will be plotted as one. If a field cannot be bound to something
real, **leave it out and say so** — the same reasoning that made `hold` default
to `None` rather than `0`.

**C. Core support for the genuinely stateful ones.** `ENC` reads and resets
counts the simulator already maintains; it is a small core addition and arguably
belongs there anyway, since encoder totals are a standard concept even though the
command is not.

**D. The planned plugin tier.** Already in the README as a future direction.
Correct for anything needing real behaviour, and the heaviest option.

**E. Do nothing, and say so.** Document that vendor profiles model the
*protocol* surface and not the whole command set, so nobody discovers it by
having a tool fail. ⚠️ **This is the current state minus the surprise**, and it
is a legitimate choice — but it leaves the bring-up tools untestable.

## 6. Recommendation

**A + B + C**, in that order, and **not D**. A and B are data-only and keep the
"profiles are data" property that makes third-party contribution safe; C is one
small core addition for the one command that needs live state.

⚠️ **`BOOTLOADER` should stay unimplemented** whatever else happens. A simulator
cannot meaningfully reset into a bootloader, and faking success would let a flash
tool "succeed" against something that flashed nothing. **Rejecting it is the
honest answer**, and `ERR code=BAD_CMD` is exactly right.

✅ **DONE 2026-08-17 — the cheap half is implemented.** A profile now declares
its own omissions via `not_modelled`, printed at start-up:

```
  NOT modelled: commands CAPS, ENC, PID, SERIAL, BOOTLOADER (emulating 15 of 20) — these answer ERR code=BAD_CMD
```

It changes no behaviour — the commands still correctly `ERR` — but a confusing
tool failure becomes a stated limitation. A test asserts the declaration is
honest **in both directions**: nothing listed as unmodelled may actually be
implemented, and no disclaimed STATUS field may actually appear. ⚠️ *A stale
disclaimer would be worse than none, because it would be believed.*

Options A–D below remain open; this only removes the surprise.

## 7. Not urgent

Nothing here blocks the 0.2.0 release: the simulator is correct about everything
it does implement, and that was verified in the same pass. This is about how much
of a vendor device a profile can usefully represent, and it should be decided
deliberately rather than by accumulating flags in the core.
