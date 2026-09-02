# Power and thermals on the ESP32-S3 shunt boards

Measured 2026-09-02 on the live two-INA228 XIAO (`esp32s3_2ina228`), firmware `29514e8` +
`8122fb6`. Die temperatures come from the ESP32-S3 on-die sensor: coarse, self-heated, a
trend indicator and not a calibrated figure. Rates are points/s/device from the collector's
journal on `rpi.local`.

## The short version

**Only the CPU clock moves the temperature.** Everything else tested is flat.

| Lever | Effect on die temp | Verdict |
|---|---|---|
| CPU 240 → 80 MHz | **−6.5 K** | keep |
| BLE TX power, +9 → −12 dBm | 0.16 K, below the 0.9 K error bar | dropped, reverted to stack default |
| BLE conn interval, 30 ms → 195 ms + latency 3 | ~0 K | keep for airtime, **not** for heat |
| PSRAM power-down | — | **no lever exists**, never initialised |
| BT controller / CPU sleep | not measurable | **out of reach** in the Arduino libs |

Best configuration: **80 MHz, stack-default TX power, 195 ms / lat 3, 400 ms publish
period.** Full data rate (2.48/s/device), 780 ms central→device window inside the 1 s
budget, and one step back from the 400 ms interval that loses 19% of samples.

It is not the *coolest* configuration — no BLE setting changes the temperature at all. It
is the coolest one the CPU clock can give, with the link set where it costs nothing.

## What the numbers were

CPU clock, one reset, port held open across the switch, so the halves differ only in the
variable under test:

| Config | Die °C | Rate /s |
|---|---|---|
| 80 MHz | **63.7** | 2.43 |
| 240 MHz | **70.2** | 2.49 |

BLE sweep, 240 s per point, only the last 100 s averaged so the thermal transient after
each change is excluded. **Read the rate column, not the temperature column** — see the
drift caveat below.

| # | Point | Die °C | Link | Rate /s |
|---|---|---|---|---|
| 1 | TX −3 dBm | 64.62 | 195 ms / lat 3 | 2.48 |
| 2 | TX +9 dBm | 64.50 | 195 ms / lat 3 | 2.46 |
| 3 | TX −12 dBm | 64.66 | 195 ms / lat 3 | 2.48 |
| 4 | TX −3 dBm *(repeat)* | 64.00 | 195 ms / lat 3 | 2.48 |
| 5 | conn 30 ms / lat 0 | 64.22 | 30 ms / lat 0 | 2.49 |
| 6 | conn 195 ms / lat 0 | 63.78 | 195 ms / lat 0 | 2.48 |
| 7 | conn 195 ms / lat 3 *(repeat)* | 63.72 | 195 ms / lat 3 | 2.49 |
| 8 | conn 400 ms / lat 3 | 63.78 | 397.5 ms / lat 3 | **2.01** |

### The error bar is bigger than every BLE effect

The same configuration, `195 ms / lat 3`, was measured three times across the run:
**64.62 → 64.00 → 63.72 °C**. And the whole column drifts downward monotonically with
elapsed time, regardless of what is being tested — ambient falling through the evening.

**Sweep order is therefore confounded with the effect**, and every BLE difference in the
table is smaller than that 0.9 K. The flaw was sweeping sequentially instead of
interleaving. The CPU A/B is unaffected: it ran 80 → 240 → 80 inside one 11-minute window,
and the return leg confirmed the drop (70.7 → 68.7 → 66.7).

### The only BLE settings that do anything, break things

Point 8 (397.5 ms interval) lost **19% of samples**; a 200 ms publish period at a 195 ms
interval lost **36%**. Both were thermally flat. `flush()` ships one indication per round
trip and a round trip costs ~1.6 connection intervals, so a period near the interval has no
headroom and the queue sheds. Point 8's skip window is also 1590 ms, past the 1 s budget.

A sweep judged on temperature alone would have called both free wins. **The throughput
cross-check is the only thing that saw them** — and it is why `blerate`'s floor is now
computed from the live interval instead of the fixed 200 ms that measurement disproved.

### Why BLE cannot show up

Cutting RX windows 26× (30 ms → 780 ms effective) changed nothing thermally, and a 3.5×
change in publish rate moved it 0.2 K. The radio's *event* work is a rounding error next to
a CPU and a BLE controller that never sleep. Which is the real finding:

> The residual ~64 °C is baseline, not workload.

`CONFIG_PM_ENABLE` and tickless idle are absent from the prebuilt Arduino sdkconfig, so the
obvious next move looks like an ESP-IDF port to switch them on.

**Do not make that port for thermal reasons — it has already been tried and it does not
work.** `~/dev/ha/nimble-ble-proxy` is a native ESP-IDF build that already sets
`CONFIG_PM_ENABLE=y`, `CONFIG_FREERTOS_USE_TICKLESS_IDLE=y` and
`CONFIG_PM_POWER_DOWN_CPU_IN_LIGHT_SLEEP=y`. Its `docs/thermal-notes.md` records the
result: **0 °C change, temperature flat at 57.6 °C across the soak.** Light sleep never
engages, because the SoC refuses it while Bluetooth is enabled — boot log, verbatim:

> light sleep mode will not be able to apply when bluetooth is enabled

This board holds a BLE connection permanently, so the same applies. The always-on radio
baseline is not a configuration mistake that a port would fix; it is what an ESP32-S3 costs
while its controller is up. See "Prior art" below.

## Configuration notes

### 80 MHz is a floor, not a preference

Below 80 MHz arduino-esp32 switches the CPU to the XTAL and powers the PLL down — taking
the 48 MHz USB-Serial-JTAG clock with it (console gone, physical BOOT+RESET to recover) and
dropping APB below 80 MHz, re-deriving every UART/I2C/SPI divider configured against it. At
80 MHz the PLL stays up and **APB stays at 80 MHz**, so peripheral clocks are bit-for-bit
unchanged; the ADS1262 is on hardware SPI, not bit-banged, so its timing does not move
either. NimBLE also requires ≥ 80 MHz. Override with `-D CPU_FREQ_MHZ=160`.

### Slave latency: keep it, for airtime not for heat

Points 6 and 7 isolate it at equal interval: latency 0 and latency 3 both deliver
**2.48 points/s**, so a peripheral with data queued does transmit at the next connection
event regardless of latency. Measured, not inferred. What is delayed is central→device
delivery (aux commands, OTA start), by at most
`latency × interval` = 780 ms. OTA takes the link back to latency 0 via the existing
`Fast` path. Supervision timeout must outlast the skipping —
`(1 + latency) × interval_max × 2` = 1.6 s against the 4 s requested — and `bleconn`
refuses anything that violates it.

It does not cool the board. Keep it because it cuts airtime on an rpi sharing one radio
between three links, a continuous scan and a 20 s initiating window — the contention that
has previously killed OTA transfers with an LL timeout.

### Console knobs (none persist)

    cpufreq [80|160|240]        clock + APB + die temp
    bletx [dBm]                 TX power; radio rounds to its own ladder
    blerate [ms]                publish period; floor = 2x negotiated interval, max 700 ms
    bleconn <min> <max> <lat>   LL units (1.25 ms), refuses unsafe latency

None persist, deliberately: a value that starves the link should be one power-cycle away
from gone. Make one permanent with a build flag. The `blerate` floor is computed live
rather than fixed, because the central owns the interval — see the ratio table in
`telemetry.h`.

## Measurement traps found the hard way

Three claims were asserted in writing before they were measured, and all three were wrong.
They are recorded because each one is a trap this bench will set again.

**1. Reading the serial port RESETS this board.** Native USB-CDC. Every
connect-ask-disconnect reading is taken ~0.5 s after a reset and describes the reset, not
the running board. Two were once compared as a before/after; they differed only in how long
the chip had been idle in the bootloader, so the *improved* config read hotter. The same
transient flatters a touch test: the board takes ~5 minutes to climb from 50 °C to 63.7 °C,
and anything felt inside that window feels like an improvement.

> To watch a running board: pay one reset, hold the port open, and let the board talk.
> That is why the firmware prints a periodic `pwr:` line rather than answering a query.

Note this contradicts the general rule in `CLAUDE.md` that serial reads are free on the
ESP32-S3. It holds for the boards in that note; it does **not** hold here.

**2. The collector's RSSI is useless for this.** It reports `self.advertisement.rssi`, and
`smart-shunt-ble-client.py` line 2 carries its own `todo use bluetoothctl to get real-time
rssi`. A connected peripheral does not advertise, so the value is frozen at the last
advertisement before the connection. A −38 → −48 shift was briefly read as a TX change
taking effect; it was a reconnect picking up a different advertisement. Device-side RSSI
does not fix it either — at the peripheral, `ble_gap_conn_rssi` reports what the *central*
transmits. Only the rpi can measure our TX power, and bumble owns that adapter exclusively.
**TX power is unmeasurable on this bench.**

**3. PSRAM was never running.** The boot banner reads `psram 0 B`.
`CONFIG_SPIRAM_BOOT_INIT` is set in the `qio_opi` sdkconfig, but
`CONFIG_SPIRAM_IGNORE_NOTFOUND` silently tolerates a part that does not answer, and
`ESP.getPsramSize()` would report 8 MB had it initialised (`SPIRAM_USE_MALLOC` is set). The
"8 MB clocked for nothing" lever never existed, and the bootloader-level `memory_type`
change it needed — failure mode: physical BOOT+RESET — was correctly not made.

## Prior art: the same investigation, on a different board

`~/dev/ha/nimble-ble-proxy/docs/thermal-notes.md` ran this exercise first, on an ESP32-S3
running WiFi STA + a NimBLE scanner. Read it before re-deriving anything here. It found the
same shape of answer, independently:

| Lever | Their result | Ours |
|---|---|---|
| CPU clock | −3.0 °C (160 → 80 MHz) | −6.5 K (240 → 80 MHz) |
| Radio TX power | −1.7 °C (WiFi, 20 → 13 dBm) | 0.16 K (BLE, 21 dB span) |
| Light sleep / CPU power-down | **0 °C** — never engages with BT on | not reachable, and moot |
| Sensor noise | ±1.5 °C, wandered 3 steps at fixed settings | 0.9 K drift over one sweep |

Two things follow. First, **we already hold both of their working levers**: this board runs
at 80 MHz, and its WiFi radio is off entirely (`disableWifi` is the boot default), which is
strictly better than turning its TX power down. Second, their sensor-noise caveat matches
ours almost exactly — two independent investigations both found the per-lever deltas
comparable to the measurement noise.

Their board settles at **51.6 °C (80 MHz) to 58 °C**, against our 63.7 °C, despite *also*
running WiFi. Since the firmware levers are already equal, the difference is not firmware:
it is board thermal design — the XIAO is a very small PCB with little copper to spread into
— plus ambient, airflow, and the fact that two different dies' on-die sensors are not
cross-calibrated. **Die temperature is not comparable between boards.**

## Method — and how this sweep got it wrong

- Change one variable, hold 240 s, average only the last 100 s. The board takes ~3 minutes
  to settle; anything shorter measures the transient.
- **Always cross-check throughput** from the collector journal. A "saving" that quietly
  drops data is not a saving, and the sample rate is the only thing that catches it. This
  caught the 400 ms knee, which was thermally invisible.
- **Repeat a point, at both ends.** Without the repeats there would have been no error bar,
  and a 0.16 K non-monotonic wobble would have been written up as a trend.
- **Restore state between points.** The first sweep applied each point's commands
  *cumulatively*, so the three publish-rate points inherited the broken 400 ms connection
  interval from point 8 and measured a rate × degraded-link interaction instead of rate.
  All three were discarded and re-run with the connection parameters re-pinned before every
  point. The same bug also left the board sitting on the lossy 397.5 ms interval when the
  run ended — a sweep must end by restoring the shipping config explicitly, not by assuming
  its last point did.
- **Interleave or randomise.** Sequential sweeping confounded every sub-1 K result with
  ambient drift.

## Open

- **Board prefix is unset.** This unit publishes `ESP32_INA228_2` / `_4` with nothing
  identifying the board; they will collide with any other board using those names. One
  console command: `board-prefix <name>`, reset to apply.
- Publish-period sweep (200 / 400 / 700 ms at a pinned 195 ms / lat 3) is re-running after
  the cumulative-state bug above. Expect it to be thermally flat and to matter only at the
  throughput and latency bounds.
- **No further firmware lever is known.** The ESP-IDF/light-sleep route is closed (above).
  Getting meaningfully below ~64 °C means either disabling BLE — which is the product — or
  changing the thermal path: airflow, or copper for the package to spread into.
