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

## Defect 3 — the step guard silenced a healthy channel on every input change

Raising the source 1.0 → 1.2 V made VMUX_A publish NaN forever. Raising it again,
1.2 → 1.5 V, did it a second time. Both times the channel was working perfectly.

The mechanism: the new reading matched neither `lastAccepted[A]` (the stale old
value) nor B, so the "matches neither channel" branch fired — and `take()` refreshes
`lastAccepted[]` only from *finite* samples, so the first NaN froze the baseline the
test compares against and every later sample failed identically.

**Two fixes were tried and both were wrong.**

*Withholding on the mismatch* assumes the input is static between visits. That is
false by construction for an instrument whose job is to track a changing voltage —
which is why it fired on both deliberate source changes and on nothing else.

*Adopting after N agreeing readings* rests on "a real source repeats, an open
contact floats". Also false: with the mux open the tap is tied to the Kelvin return
through the 20 kΩ low leg, so an open contact reads a **perfectly stable** value.
The agreeing readings adopt it and publish it forever — permanent silence traded for
a permanent plausible wrong number, the worse failure.

**The shipped behaviour publishes the value and flags the doubt.** One channel's
readings cannot separate "the input moved" from "the contact opened"; the
information is not there, and neither NaN nor adoption can manufacture it. So the
number goes out — it exists, and it is far more often a changed input than a failed
relay — carrying `DIAG_RELAY_UNMATCHED` in the diagnostic field, where a consumer
can see it, query it, and decide. The console reports a running count.

That does not leave the open-contact fault unguarded. An open contact reads the
network termination, and whenever the other channel is not sitting on that same
termination, `expectMove && !moved` fires on it and still publishes NaN. On this rig
that is measured, not assumed: B's input is open and reads −14 µV, exactly where an
open A would land. What is genuinely lost is the case where both the fault *and* the
other channel sit near the termination — and that case was never separable here.

`moveEpsV` came down 1 mV → **100 µV** at the same time. With the PGA enabled the
noise floor is ~1 µV sd, so 1 mV was ~1000 σ: it disarmed the guard across a band far
wider than any real ambiguity. 100 µV is still ~100 σ.

Verified after the fix, source at 1.5 V: **29190.1 µV, spread 4.4 µV, sd 1.14 µV**,
no diagnostics set. The `DIAG_RELAY_UNMATCHED` path itself has not yet been observed
on hardware — it only fires at the moment of a change, and the next source step will
exercise it.

## The settle campaign: 250 ms buys two time constants of something unidentified

Measured 2026-09-03 on the nest with A on the 26.225 V bench bus (raw 0.5147 V,
implied divider ratio 50.95) and B open. `env:xiao_nest_sweep` steps settleMs and
publishes the value in force as `P`.

**The method had to be fixed once before it measured anything.** The first sweep
walked the settle list repeatedly, on the stated grounds that repetition averaged
thermal drift out. It does not: the walk is always in the same order, so a monotonic
drift maps onto the settle axis identically on every pass, and repeating it averages
the noise while preserving the correlation exactly. Split in half, that run carried
~8 uV more apparent bias in its second half at EVERY settle value including ones
certainly long enough -- a clock, not a relay. The reported "sharp break at 50 ms"
was drift.

The fix is to alternate ref, test, ref and compare each test block against the mean
of the two references either side, which cancels drift to first order, with the
reference value left in the test list as a NULL CONTROL. That control is what makes
the rest of the table readable: it comes out at **+0.17 +- 0.62 uV**, i.e. zero.

| sample instant | A bias | B bias |
|---|---|---|
| 366 ms (null control) | **+0.17 +- 0.62 uV** | -1.30 +- 0.50 |
| 313 ms | -1.48 +- 0.61 | +1.65 +- 0.39 |
| 261 ms | -3.60 +- 0.25 | +4.25 +- 3.56 |
| 209 ms | -6.35 +- 0.44 | +5.42 +- 0.52 |
| 157 ms | **-13.07 +- 0.32** | +5.82 +- 0.32 |

A is an exponential in the time from ARM with **tau ~ 75 ms**. B is opposite in sign,
2.25x smaller, and flattens rather than continuing to grow.

**The settle axis is quantised, and nine values are five experiments.** The sample
instant is `ceil(S / 52.22) * 52.22 + 104.44`, so S=150 and S=120 return *identical*
biases -- they are the same experiment run twice. The measured dwells confirm it:
50/35/20 ms all give 365.5 ms, 90/70 give 418, 150/120 give 470.

## 157 ms is a hardware floor, and DISCARD_SCANS = 2 is exactly right

It is tempting to read the quantisation as an invitation to cut the discards. It is
not, and doing so would return a *better* number for the worst reason.

The ARM path is an RC delay-on (1 MOhm x 100 nF into an HCT14) whose slow corner puts
contact close at **65 ms**. The first conversion boundary strictly after that is
2 x 52.22 = 104.4 ms, and that conversion runs to **156.7 ms**. So the earliest
entirely-post-close conversion ends at 157 ms, which is exactly where DISCARD_SCANS=2
puts it. At DISCARD_SCANS=1 the used conversion would span 52->104 ms, straddling
contact close: contaminated, and biased in the direction that looks like success.

Predicted minimum dwell 50 + 157 + 3 x 52.2 = 364 ms against 365.5 measured.

Going below 157 ms needs the ARM RC shortened or a faster data rate. It is not a
firmware lever.

## Duty cycle

`dwell = 125 + settle + 50 x AVG`, integration is `50 x AVG`, so at the floor settle
`duty = 52.2A / (154.8 + 52.2A)`:

| AVG | dwell | duty | alternation |
|---|---|---|---|
| 4 (today, settle 250) | 575 ms | **34.8%** | 1.15 s |
| 4 (floor settle) | 364 ms | 57% | 0.73 s |
| 8 (floor settle) | 572 ms | **73%** | 1.14 s |
| 16 (floor settle) | 990 ms | 84% | 1.98 s |

Averaging MORE raises duty, because averaging is the ADC doing useful work; only dead
time, settle and discards are waste. Cutting AVG_CONVERSIONS to 1 now that the PGA has
removed the transient it was a palliative for would take duty to 12%.

**But the floor settle is the -13 uV condition.** Duty and the uV tail trade directly,
and having both requires removing the tail at its source.

## What the tail is: RESOLVED -- dielectric absorption in the 100 nF X7R

Measured 2026-09-03 by a six-plateau sweep at 20/12/8/4/2/20 V on the E3633A
(`pwr-metering` `plans/relay-mux-settle-tail.md`, raw data archived alongside it).
The discriminator was whether the bias scales with the input step: dielectric
absorption does, thermal EMF does not.

| plateau | step at ADC | t=157 ms | t=209 ms | t=261 ms | null control |
|---|---|---|---|---|---|
| 20 V #1 | 0.3989 V | -9.64 +- 0.73 | -5.70 +- 0.46 | -3.17 +- 0.42 | +0.20 +- 0.62 |
| 12 V | 0.2393 V | -6.60 +- 0.24 | -3.36 +- 0.31 | -1.58 +- 0.45 | +0.18 +- 0.19 |
| 8 V | 0.1596 V | -3.91 +- 0.17 | -2.30 +- 1.15 | -1.11 +- 0.35 | +0.00 +- 0.13 |
| 4 V | 0.0799 V | -2.17 +- 0.16 | -0.84 +- 0.30 | -0.86 +- 0.22 | -0.13 +- 0.36 |
| 2 V | 0.0401 V | -1.26 +- 0.41 | -0.25 +- 0.26 | -0.32 +- 0.23 | +0.42 +- 0.40 |
| 20 V #6 | 0.3992 V | -11.22 +- 2.58 | -- | -4.45 +- 0.38 | +2.63 +- 0.83 |

Regressing bias on step size at the earliest reachable instant:

    t=157 ms   slope -25.68 +- 1.38 uV/V (18.6 sigma)
               intercept -0.07 +- 0.22 uV (0.3 sigma)

DA predicted a significant slope and a zero intercept. Thermal EMF predicted a
zero slope and a **-13.07 uV** intercept, which this **excludes at 59 sigma**.
The 2 V plateau decides it on its own: thermal EMF predicted -13.07 uV there, DA
predicted -1.03, and the measurement is **-1.26 +- 0.41 uV**. This is why the LOW
plateaus are the sharp test and not the high ones -- the noise floor is ADC noise
and stays at ~0.6 uV, while the gap between the predictions grows as the input
falls.

Three independent checks agree. The null control is zero on every plateau. The
drift control (first and last plateau, both at 20 V) agrees to 0.6 sigma. And the
fitted slope reproduces the earlier 26 V bench-bus anchor of -13.07 uV / 0.5147 V
= -25.4 uV/V, from a completely different source.

At 366 ms the slope falls to 0.5 sigma -- the tail is simply gone by then, which
is the same statement the original null control made, arrived at independently.

**So the asymmetry that pointed at two relays warming differently was a red
herring.** A and B do bias toward each other, but B's input is open: it has no
step to absorb, so nothing about B's magnitude constrains a step-proportional
mechanism on A.

### What to do about it

`hw/shunt-adc/sch_design.py:727` chose X7R over C0G and justified it partly as
"with the PGA enabled the input is 1 GOhm so there is no charge-transfer path for
dielectric absorption". That is true of the ADC side, and it is why this defect
was not anticipated -- **DA's error develops across the SOURCE**, which this
fixture changed from the Kelvin-sensed ~0 Ohm burden shunt the filter was designed
for to the divider's 19.6 kOhm. Correct that comment wherever the part is
respecified. The implied DA branch is ~772 MOhm with C_da ~ 97 pF at tau = 75 ms,
an effective DA of 0.097% -- entirely ordinary for X7R, and harmless in the
application the filter was drawn for.

The fix is passive: C0G or film in place of the 100 nF X7R, a smaller value, or a
lower source impedance. Then settle can fall toward the 157 ms hardware floor and
ADC duty rises from 35% to ~73%. Re-run the plateau sweep afterwards to confirm
the tail is gone rather than merely smaller.

**No relay change is needed.** That branch was also the expensive one: the fitted
part is `3502-05-511`, already Coto's best thermal-EMF grade (<0.5 uV
differential), with stock 0 and 0 on order.

The filter's own RC was never a candidate: 21.6 kOhm x 100 nF = 2.16 ms, i.e.
e^-73 by 157 ms.

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
- **`DIAG_RELAY_UNMATCHED` has never been seen set.** The code path is reached only
  on a source change; the fix was verified by the channel *recovering*, not by the
  flag appearing.
- **`moveEpsV = 100 µV` is a reasoned choice, not a measured one.** It is ~100× the
  noise floor, which bounds false "moved" verdicts, but the value that matters is
  how far apart the two channels must be before a stuck contact is worth catching,
  and that is a bench fact about the contacts.
- **The implied divider ratio moved with the source**: 51.21 at 1.2 V, 51.39 at
  1.5 V (0.35%). That is within a bench supply's dial accuracy and is not evidence
  of divider nonlinearity — but it has not been checked against a DMM.
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
