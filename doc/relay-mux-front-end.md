# The relay-mux ratio front end: what it measures, and two defects that cost 141 µV

Bench campaign of 2026-09-02/03 on the **nest** — a XIAO ESP32-S3 driving the
three-board riser from `pwr-metering`: Coto relay mux → 51:1 divider → shunt-adc
(ADS1262), env `xiao_nest`.

Everything below is measured on that rig, through BLE into InfluxDB. The board's
USB-CDC is dead in both directions, so *no* number here came from the console.

## The headline

One 1.2 V source into the mux's left input, right input open, published as raw ADC
volts (`dividerRatio` deliberately unset).

| build | VMUX_A spread | VMUX_A sd | VMUX_B mean (should be 0) |
|---|---|---|---|
| G=1, PGA bypassed, temp read on | **141 µV** | — | **+3327 µV** |
| …averaging 2 conversions | 65 µV | — | |
| …averaging 4 conversions | 34.4 µV | 7.1 µV | |
| …averaging 4, temp read **off** | 3.9 µV | 0.9 µV | |
| **G=1, PGA enabled, temp read back on** | **4.5 µV** | **0.9 µV** | **−13.9 µV** |

207 consecutive samples in the last row landed in a single 5 µV bin.

## Defect 1 — the die-temperature read injects a settling transient

`ads1262.h`'s `TEMP_EVERY_N_SCANS = 64` drops chop, retunes gain and INPMUX, then
restores production config and toggles `START`. The comment there already said it:
*"every one of those restarts is a fresh settling transient in the middle of a
precision measurement."* Nobody had measured what it cost.

64 × 52.22 ms = **3.34 s**. Against the ~836 ms round trip in force when this was
measured, that is **one dwell in four** — the period-4 pattern that started this.
It is a fact about that measurement, not a standing property: at the ~1.15 s round
trip that 4-conversion averaging produces, the ratio is ~2.9. Affected samples also
arrived ~60 ms late, the restart's own latency.

**How it was proved.** `ADS1262_TEMP_EVERY_N_SCANS=0` disables the read outright.
The artefact vanished (141 µV → 3.9 µV), rather than merely becoming rarer. That is
causal, not correlational.

**A wrong turn worth recording.** The first fix averaged consecutive conversions,
justified as "averaging N conversions cancels any disturbance whose period divides
N". That predicted ~0 µV at N=4. The bench returned 34.4 µV, and the residual
halved with each doubling of N — 1/N dilution of *one* bad conversion, not
cancellation of a periodic term. The reasoning was wrong even though the numbers
improved, which is the trap: a change that helps is not thereby explained.

**Does `SHUNT_ADC` have it too?** Unverified, and probably not in the same form.
It runs the same driver and gets the same restart, but at **G=32 with the PGA
already enabled** — so the bias-current-through-source-impedance mechanism that
made this so large does not apply to it, and its source is Kelvin-sensed at near
zero ohms besides. Only the relay-mux path was measured. Worth checking, not worth
asserting.

## Defect 2 — PGA bypass cost 3.3 mV of offset

The relay-mux pair inherited **G=1 with the PGA bypassed** from the DCCT burden
channel. The stated reason: at G=1 the PGA's output swing leaves each input pin
roughly −2.2…+2.2 V on ±2.5 V rails, which a ground-referenced divider output could
exceed.

That worst case is not this source. Ground-referenced means AINN sits at AGND, so
V_cm = V_diff/2 and the pins land at **V_diff and 0**, not ±V_diff:

| input | divider out | PGA pins | margin to ±2.2 V |
|---|---|---|---|
| 80 V (qualified) | 1.569 V | 1.569 / 0 V | 0.586 V |
| 100 V (engineering) | 1.961 V | 1.961 / 0 V | 0.194 V |

Those margins are the **tolerated** ones. AVDD comes from an ADP7118 specified
±1.8%, so worst-case AVDD is 2.455 V and the limit is 2.155 V rather than the
nominal 2.2 V — putting the guaranteed clip point at **~109.9 V**, not ~112 V. The
board's 100 V engineering limit still sits inside it.

**That margin is the divider's, not the ADC's** — re-range the divider and this
must be re-derived.

What bypass cost was input current. Both channels carried the *same* additive
offset: A read 22.79 mV where 51:1 predicts 19.61 mV, and B (open) read 3.33 mV
where 0 is predicted. ~3.3 mV across the divider's 1 MΩ ∥ 20 kΩ = **19.6 kΩ**
implies **~165 nA** of input bias current — 17% of a 1 V reading, and the reason
`A − B` was needed to recover 1 V instead of reading A directly.

Enabling the PGA collapsed it to **−13.9 µV**, a factor of 230, and A became
directly usable: 23.431 mV against 23.529 mV nominal for 1.2 V ÷ 51, i.e. an
implied ratio of 51.21.

**It also removed defect 1's mechanism.** With the input buffered, bias current is
picoamps and the 19.6 kΩ source impedance stops converting configuration changes
into offset. The temperature read still restarts the converter — it just no longer
has a lever. Hence the last table row: temp read on, artefact absent.

The 165 nA is **inferred from the offset, not read from SBAS661C**. It has not been
checked against the bypass-mode input-current spec.

## Defect 3 — the step guard could silence a healthy channel permanently

Raising the source 1.0 → 1.2 V made VMUX_A publish NaN forever. The new reading
matched neither `lastAccepted[A]` (the stale 22.79 mV) nor B, so the "both contacts
open?" branch fired — and `take()` refreshes `lastAccepted[]` only from *finite*
samples, so the stale baseline froze and every later sample failed identically.

A guard that cannot return to PASS on healthy input is broken whatever else it
catches.

**The obvious fix is also wrong, and review caught it.** Requiring three agreeing
readings before adopting the new value rests on "a real source repeats, an open
contact floats". It does not: with the mux open the ADC still sees the divider's
own termination through the 20 kΩ low leg, so an **open contact reads a perfectly
stable value**. Three agreeing readings would adopt it, `take()` would store it,
and every later open reading would then match and publish forever — trading
permanent silence for a permanent plausible wrong number, which is the worse
failure and the one the guard exists to prevent.

One channel's readings cannot separate "the input moved" from "the contact opened";
the information is not there. So the shipped behaviour is the safe one: publish
NaN, count it, say so loudly, and let a **reboot** clear it — which on this board
costs nothing, since it reflashes and reboots over BLE. Auto-adoption survives only
behind `rebaselineOnUnmatched`, default false.

A second latch also survives and is *not* fixed: if A changes to within `moveEpsV`
of B, `expectMove && !moved` fires and NaN persists indefinitely.

## VMUX_B's oscillation is not a fault

B (open input) shows ~50 µV peak-to-peak of slow sine. That is mains reaching the
ADC node *through* the 1 MΩ high leg from the open pigtail — attenuated 51:1, then
hit by the FIR's 50/60 Hz notch — beating at the grid's own drift against the
notch. The ADC node itself is ~20 kΩ either way; what differs is that A's divider
top is driven by a low-impedance source that shorts the pickup out, while B's
floats. Expect it to collapse when B is connected to anything real.

Its *character* changed when the temperature read was disabled: the restart had
been jittering the sample interval (1008–1208 ms), smearing the alias. Without it
the interval is 1147–1149 ms and the alias goes coherent. The same defect was
corrupting timing as well as amplitude.

## Not verified

- **High-voltage headroom.** The PGA margin table above is arithmetic. Nothing has
  been measured above 1.2 V in, so the 80 V and 100 V rows are unconfirmed.
- **The 165 nA** figure, as noted.
- **The post-restart discard** — the direct fix for defect 1 — is not implemented.
  Enabling the PGA removed the symptom on this path by removing the *mechanism*
  (bias current into a 19.6 kΩ source); the restart itself is untouched.
- **The BLE `download` trigger** is on the board but has never been fired.
- **`moveEpsV = 1 mV` is wrong for this instrument** and is knowingly left alone.
  At ~20 mV raw that is 5% — 50,000 ppm. Worse, when the two inputs differ by less
  than 1 mV the switch-integrity guard **disables itself**, which is exactly the
  ratio case it is most needed for. With the PGA enabled the noise floor is ~0.9 µV
  sd, so it can come down two or three orders — but the value is a bench fact about
  the contacts, not a guess.
- **The BLE OTA/control surface is unauthenticated.** Any nearby client can write
  `download` and force the board into the ROM downloader; secure boot is disabled,
  so an attacker could already push a chosen image. Recoverable (power cycle), but
  it is a denial-of-service on an open characteristic.
- **`download` recovery was mis-documented.** "Reset twice" does not work: in the
  ROM downloader the app never runs, so `setup()`'s disarm never executes. Only a
  flash or a **power cycle** clears the bit.

## Reflashing this board

USB is dead. Use OTA over BLE — it works and needs nobody at the bench. The recipe
and its three separate gotchas (venv, Bumble backend rather than BlueZ, root for
the HCI socket) are in `doc/ota-over-ble.md` and the session memory.
