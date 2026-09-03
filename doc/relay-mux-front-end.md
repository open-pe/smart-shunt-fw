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
| 4, settle 250 (until 2026-09-03) | 575 ms | **34.8%** | 1.15 s |
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

    t=157 ms   slope -25.73 +- 0.86 uV/V (29.9 sigma)
               intercept -0.06 +- 0.09 uV (0.7 sigma)

DA predicted a significant slope and a zero intercept. Thermal EMF predicted a
zero slope and a **-13.07 uV** intercept, which this excludes at **33.7 sigma** via the
intercept -- and at **145 sigma** via the zero-step plateau below, which does not
depend on the -13.07 anchor or its +-0.32 at all.
The 2 V plateau decides it on its own: thermal EMF predicted -13.07 uV there, DA
predicted -1.03, and the measurement is **-1.26 +- 0.41 uV**. This is why the LOW
plateaus are the sharp test and not the high ones -- the noise floor is ADC noise
and stays at ~0.6 uV, while the gap between the predictions grows as the input
falls.

Three independent checks agree. The null control is zero on every plateau. The
drift control (first and last plateau, both at 20 V) agrees to 0.6 sigma. And the
fitted slope reproduces the earlier 26 V bench-bus anchor of -13.07 uV / 0.5147 V
= -25.4 uV/V, from a completely different source.

At 366 ms the slope falls to ~1 sigma -- the tail is simply gone by then, which
is the same statement the original null control made, arrived at independently.

**The decisive point came for free.** After the sweep switched the supply off the
board kept alternating, giving a plateau at step ~ 0:

| plateau | step | t=157 ms | t=209 ms | t=261 ms |
|---|---|---|---|---|
| 0 V | 0.0001 V | **-0.06 +- 0.09** | +0.18 +- 0.13 | +0.20 +- 0.16 |

The relay is switching exactly as before -- same contacts, same ~71 mW coil, same
cadence -- so thermal EMF is fully present there and still predicts -13.07 uV.
Measured -0.06 +- 0.09 uV: excluded at **145 sigma at the point**, not by
extrapolating a fitted intercept to a step nothing was measured at.

**And the exponent identifies the mechanism, where the slope only discriminated
it.** Several mechanisms scale with the step; they differ in the power.
chi2/dof at t=157 ms over seven plateaus:

| model | mechanism | chi2/dof |
|---|---|---|
| **y = a*step** | **dielectric absorption** | **1.10** |
| y = b*step^2 | resistor self-heating (V^2/R) | 42.2 |
| y = const | thermal EMF, or any fixed offset | 181.2 |

The exponent is 1, which rules out self-heating in the divider's 1 MOhm
resistors -- the most plausible remaining step-correlated alternative. Together
with tau ~ 75 ms, which no lumped RC here can produce (the filter's own is
2.16 ms; 75 ms across 19.6 kOhm would need 3.8 uF), what survives is dielectric
relaxation.

**So the asymmetry that pointed at two relays warming differently was a red
herring.** A and B do bias toward each other, but B's input is open: it has no
step to absorb, so nothing about B's magnitude constrains a step-proportional
mechanism on A.

### What this does and does not establish

It establishes a **slow relaxation, linear in the input step, tau ~ 102 +- 7 ms**,
and it excludes both a fixed series offset (thermal EMF) and any V^2-driven
mechanism such as resistor self-heating.

It does **not** on its own localise that relaxation to the shunt-adc board's
`CD1`-`CD4`. Dielectric effects in the common-mode caps, the relay package or the
wiring, and slow memory in the ADC front end, all remain consistent with a linear
step-proportional relaxation. `CD1`-`CD4` is the prime suspect because it is the
largest dielectric in the loop by two orders of magnitude and sits directly
across the measured pair -- and the cap swap is itself the experiment that
closes the question.

(An earlier revision of this section quoted tau = 75 ms, an effective DA of
0.097%, and a 59 sigma exclusion. The 75 ms was carried over from the earlier
bench-bus run rather than fitted to this one; the DA loop resistance omitted the
filter's own series pair; and the 59 sigma omitted the uncertainty on the value
being excluded. Corrected after an independent review.)

### What to do about it

**Done and CONFIRMED 2026-09-03.** `CD1`-`CD4` on the shunt-adc board moved to
`GRM3195C1H104JA05D`, 100 nF C0G 50 V 1206 (`pwr-metering` `hw/shunt-adc`
finding F23), the board regenerated and re-verified (ERC 0/0, DRC 0/0,
`audit_pcb.py` all PASS with every guard calibrated, 0 unrouted, input-pair
mismatch still 0.0 um), then the identical sweep re-run:

| | slope at t=157 ms |
|---|---|
| X7R 0603 | -25.73 +- 0.86 uV/V (29.9 sigma) |
| **C0G 1206** | **-0.24 +- 2.04 uV/V (0.1 sigma -- consistent with zero)** |
| difference | 25.49 +- 2.21 = **11.5 sigma** |

Per plateau at t=157 ms: 20 V -9.64 -> **-0.81 +- 0.91**, 12 V -6.60 ->
**-1.13 +- 0.52**, 8 V -3.91 -> **+0.08 +- 0.15**, 2 V -1.26 -> **-0.24 +- 0.14**.
At the 20 V plateau the bias goes from 3.9x the 2.5 uV target to inside it.

**This also closes the localisation gap above.** Only `CD1`-`CD4` changed --
same relays, divider, wiring, ADC and firmware -- and the tail vanished, so the
mechanism was in those capacitors and not in the CM caps, the relay package, the
wiring or the ADC front end.

**So the 250 ms settle is no longer buying anything**, and settle can fall
toward the 157 ms hardware floor: ADC duty 35 % -> ~73 % at AVG 8. Residual to
watch: t=209 ms reads -4.06 +- 1.09 uV/V (3.7 sigma) on n=4, which cannot be a
real tail while t=157 ms reads 0.1 sigma, but should be re-checked.

`hw/shunt-adc/sch_design.py:727` chose X7R over C0G and justified it partly as
"with the PGA enabled the input is 1 GOhm so there is no charge-transfer path for
dielectric absorption". That is true of the ADC side, and it is why this defect
was not anticipated -- **DA's error develops across the SOURCE**, which this
fixture changed from the Kelvin-sensed ~0 Ohm burden shunt the filter was designed
for to the divider's 19.6 kOhm. Correct that comment wherever the part is
respecified. The implied DA branch follows from tau = 102 +- 7 ms across the 21.6 kOhm loop
(19.6 kOhm source plus the filter's own 2 kOhm series pair),
an effective DA of ~0.123% -- entirely ordinary for X7R, and harmless in the
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

## The production retune: least dead time, half the switching frequency

With the tail gone the settle margin no longer buys anything, so the timing was
re-derived from what the parts actually specify. Requested outcome: the smallest
defensible dead time, and half the switching rate.

| | before | after |
|---|---|---|
| `DEAD_TIME_MS` | 50 | **5** |
| `SETTLE_MS_DEFAULT` | 250 | **30** |
| `AVG_CONVERSIONS` | 4 | **20** |
| round trip, **measured** | **1.149 s** (22 conv, no spread) | **2.298 / 2.356 s** (44 / 45 conv), mean 2.338 |
| switching frequency | 0.870 Hz | **0.428 Hz — down 50.8%** |
| duty (integration / dwell) | 36.4% | **89.3%** |
| sample instant after ARM | 157 ms | 157 ms (unchanged) |

Both round-trip figures are measured from the published sample timestamps, not
derived: 3109 points/hour for eight hours before the change, all at 1.149 s with
median equal to mode, and 179 points over seven minutes after it.

Two things make this free rather than a trade.

**The sample instant does not move.** The converter free-runs, so `settleMs` does not
decide *when* the sample is taken -- only which free-running conversion the discards
land on. Every settle from 1 to 52.22 ms yields the same 157 ms instant. Dropping 250
to 30 therefore costs nothing in settling, and there is no point going below it either.
A's spread over the seven minutes was 6.1 uV.

**The dead time is still a margin over an unmeasured quantity -- the datasheet does not
close it.** `DEAD_TIME_MS` exists to guarantee the outgoing contact has released before
the incoming one closes. The Coto 3500-series datasheet gives the 3502 **release 0.1 ms
typical**, and it is tempting to call 5 ms "50x that". It is not, for two reasons: the
0.1 ms is measured "At Nominal Coil Voltage, 30 Hz Square Wave" with **no coil
suppression specified**, while this board fits D3/D4, plain 1N4148W flyback diodes that
hold coil current circulating at ~0.7 V of forcing and stretch the flux decay -- and
DESIGN.md says so itself, *"change to a diode/Zener clamp only if the plain diode cannot
meet dead-time requirements"*. Second, 0.1 ms is typical and Coto publishes no maximum.

What makes 5 ms comfortable is that the dead window is **not** the primary non-overlap
margin. On an A<->B alternation the incoming channel's delay-on capacitor has been
discharged for a whole dwell, so after ARM rises that coil still needs the fastest-corner
15.95 ms before it can pull in. The dead window is a second, independent margin -- the
one that survives a failed delay-on network -- and 5 ms of it costs nothing at the
current rate. Spending it down to `DEAD_MIN_MS = 1` would buy 4 ms of headroom nobody
has shown is needed.

That number was *not* measurable on this rig, and the attempt is worth recording. A
dead-time sweep was written to read the ADC at the end of the dead window and look for
the moment the reading stops being the channel just left. It was flashed before the
flaw was noticed: **B's node discharges through the divider's 19.6 kOhm into 100 nF,
tau ~ 2 ms, while the sample is 157-366 ms later.** Whatever the contacts were doing
had decayed by e^-75 before anything looked. The firmware comment that justified the
sweep claimed tau ~ 100 ms through the 1 MOhm leg and had simply ignored the 20 kOhm
low leg. **An ADC behind a divider cannot measure contact overlap** -- that needs a
scope on the node, and DESIGN.md's qualification item stays open. 0.1 ms is *typical*;
Coto publishes no maximum.

### The rate cliff is real, and 45 ms was on the wrong side of it

The dwell is a whole number of conversions,
`ceil((dead + settle) / 52.22) + 1 + AVG_CONVERSIONS`, so keeping `dead + settle`
inside one conversion is worth a whole conversion *per dwell*, i.e. two per round trip.
The first production build set settle to 45 ms -- `dead + settle = 50`, satisfying the
inequality with 2.2 ms to spare -- and the bench returned round trips of **45 / 46 / 47
conversions (2.350 / 2.402 / 2.461 s), with 47 the mode**, against the 44 predicted.

Settle went to **30 ms**, and the whole distribution moved down by exactly one
conversion per dwell: **44 / 45 conversions, 31% / 68%**. A 15 ms change cannot move a
`ceil` whose argument is 35-50 ms against a 52.22 ms period -- unless the argument is
larger than `dead + settle`. It is: the timer is evaluated by `realTimeTask`, which
also runs `relayMux.tick()` and `samplers.updateAll()` on every pass, so the quantity
being rounded up is `dead + settle + latency`.

The `static_assert` bound therefore went from 52 to **40**, so the build fails while
there is still headroom for that latency rather than only once the inequality itself
breaks. At 45 ms the inequality held and the cliff was fallen off anyway.

### What is still not deterministic, and why 50.8% rather than 50.0%

The old configuration was perfectly repeatable -- 1.149 s, no second mode, over
thousands of samples. The new one is not: two thirds of round trips take one conversion
more than the other third. Since `dead + settle = 35 ms` now sits 17.2 ms below the
boundary and the loop's own yield is `vTaskDelay(1)`, a simple 1 ms poll granularity
does not account for it, and **the mechanism is not established.**

What can be said is the size of it: 0.4 conversions per dwell on average, 2.3% of the
round trip, which is why the switching frequency fell by 50.8% rather than 50.0%.
`AVG_CONVERSIONS` is the only coarse knob and it must stay even, so 20 is the closest
reachable value -- 18 would undershoot to ~1.85x. Do not read the 2x in this section as
exact; the measured ratio is **2.035**.

### The auto-widener had to be fixed to survive this

The widener compared `tStableMs` against `settleMs`. That was a category error hidden
by the old 250 ms value: `stable` needs `STABLE_N = 3` agreeing conversions, so
`tStableMs` cannot report earlier than ~157 ms, and against a 30 ms settle the test
would be true on **every** switch -- ratcheting the settle straight back up and undoing
the configuration permanently on the first dwell.

It now compares against `sampleInstantMs(settleMs) + CONVERSION_MS`, i.e. against when
the published conversion actually ends, plus one conversion of headroom. The headroom
is required, not padding: `tStableMs` is quantised to the conversion grid and its floor
ranges over a whole conversion (~104-157 ms) depending on phase, so without it the
widener fires on the unlucky phase alone. Rarely is worse than never here, because one
spurious widen is permanent -- the widener only ever increases.

Confirmed on the bench: over seven minutes the first half and second half of the run
both median 2.356 s. A firing widener would show as a monotonically growing round trip.

## VMUX_B's oscillation is not a fault

B (open input) shows ~50 µV peak-to-peak of slow sine. That is mains reaching the
ADC node *through* the 1 MΩ high leg from the open pigtail — attenuated 51:1, then
hit by the FIR's 50/60 Hz notch — beating at the grid's own drift against the
notch. The ADC node itself is ~20 kΩ either way; what differs is that A's divider
top is driven by a low-impedance source that shorts the pickup out, while B's
floats. Expect it to collapse when B is connected to anything real.

**Its visibility is set by the settle time, which is why it arrives in bursts
during a settle sweep.** Median block sd on B against A, over the 2026-09-03
runs:

| settle | B (open) | A (driven) |
|---|---|---|
| 250 ms | 2.6 uV | 1.1 uV |
| 120 ms | 16.3 uV | 1.2 uV |
| 35 ms | 17.2 uV | 1.0 uV |

A is flat at ~1 uV at every settle; only B varies, monotonically. After the
contact closes, B's node -- 1 MOhm into an unterminated pigtail -- settles
through picoamp currents, and the residual at the sample instant depends on the
mains phase *at the moment of closure*, which is random from switch to switch.
The tell that it is a random-phase transient and not an offset: B's MEAN stays
at -14 uV across every tag while its spread changes 6x. On a Grafana trace this
looks like bursts of noise every ~20 s, which is simply the firmware stepping to
the next settle value. It does not enter the A measurement.

Its *character* changed when the temperature read was disabled: the restart had
been jittering the sample interval (1008–1208 ms), smearing the alias. Without it
the interval is 1147–1149 ms and the alias goes coherent. The same defect was
corrupting timing as well as amplitude.

## Not verified

- **Contact release time.** `DEAD_TIME_MS = 5` rests on Coto's *typical* 0.1 ms; no
  maximum is published, and the datasheet is the only source. It cannot be measured
  from the ADC (see the retune section: the node's own tau is ~2 ms against a 157 ms
  sample), so this stays a scope job.
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
