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

CPU clock, one reset, port held open across the switch, so the two halves differ only in
the variable under test:

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

**Sweep order is therefore confounded with the effect.** Later points read cooler no
matter what they test. Every BLE difference in the table is smaller than this 0.9 K drift,
so none of them is a measurement. The design flaw was sweeping sequentially instead of
interleaving; a repeat point at both ends is the minimum that makes the confound visible.

The CPU A/B is not affected — it ran 80 → 240 → 80 inside one 11-minute window, and the
return leg confirmed the drop (70.7 → 68.7 → 66.7).

### Latency does not delay our telemetry

Points 6 and 7 isolate it at equal interval: latency 0 and latency 3 both deliver
**2.48 points/s**. A peripheral with data queued transmits at the next connection event
regardless of latency, so only central→device traffic is delayed. Measured, not inferred.

### The knee is between 195 ms and 400 ms

Point 8 is the only BLE setting that broke anything: at a 397.5 ms interval the rate fell
to **2.01/s — 19% of samples lost**, while the temperature stayed flat like everything
else. `flush()` ships at most one indication per round trip, so a ~400 ms interval supplies
~2.5 events/s against a 400 ms publish tick needing 2.5 — no headroom, and the queue sheds.
Its skip window is also 397.5 × 4 = 1590 ms, past the 1 s budget.

A sweep judged on temperature alone would have called this a free win. **The throughput
cross-check is the only thing that saw it.**

### Why BLE cannot show up

Cutting RX windows 26× (30 ms → 780 ms effective) changed nothing thermally. The radio's
*event* work is a rounding error next to a CPU and a BLE controller that never sleep.
Which is the real finding:

> The residual ~64 °C is baseline, not workload. It is the always-on cost of a chip with
> no power management compiled in.

`CONFIG_BT_LE_SLEEP_ENABLE`, `CONFIG_PM_ENABLE` and tickless idle are all **absent** from
the prebuilt Arduino sdkconfig. That is where the remaining heat lives, and it is not
reachable without moving off the Arduino libraries to ESP-IDF. Anyone who wants the board
meaningfully cooler than 64 °C is signing up for that port; nothing at the application
level will do it.

## Configuration notes

### 80 MHz is a floor, not a preference

Below 80 MHz arduino-esp32 switches the CPU to the XTAL and powers the PLL down. That
takes the 48 MHz USB-Serial-JTAG clock with it — the console disappears, recoverable only
by a physical BOOT+RESET — and drops APB below 80 MHz, re-deriving every UART/I2C/SPI
divider configured against it. At 80 MHz the PLL stays up and **APB stays at 80 MHz**, so
peripheral clocks are bit-for-bit unchanged. The ADS1262 is on hardware SPI, not bit-banged,
so its timing does not move either. NimBLE also requires ≥ 80 MHz.

Override per env with `-D CPU_FREQ_MHZ=160`.

### Slave latency: keep it, for airtime not for heat

What is delayed is central→device delivery (aux commands, OTA start), by at most
`latency × interval` = 780 ms. OTA takes the link back to latency 0 via the existing
`Fast` path. Supervision timeout must outlast the skipping —
`(1 + latency) × interval_max × 2` = 1.6 s against the 4 s requested — and `bleconn`
refuses anything that violates it.

It does not cool the board. Keep it because it cuts airtime on an rpi sharing one radio
between three links, a continuous scan and a 20 s initiating window — the contention that
has previously killed OTA transfers with an LL timeout.

### Console knobs (none persist)

    cpufreq [80|160|240]     clock + APB + die temp
    bletx [dBm]              TX power; radio rounds to its own ladder
    blerate [ms]             publish period, bounded 200..700
    bleconn <min> <max> <lat>   LL units (1.25 ms), refuses unsafe latency

None are persisted, deliberately: a value that starves the link should be one power-cycle
away from gone. Make one permanent with a build flag.

The `blerate` bounds are load-bearing. Above ~700 ms the end-to-end sample age eats the 1 s
budget; below the connection interval the queue fills and sheds samples — the failure
`telemetry.h`'s catch-up-drain comment documents.

## Measurement traps found the hard way

Three claims were asserted in writing before they were measured, and all three were wrong.
They are recorded because each one is a trap this bench will set again.

**1. Reading the serial port RESETS this board.** Native USB-CDC. Every
connect-ask-disconnect temperature reading is taken ~0.5 s after a reset and describes the
reset, not the running board. Two such readings were once compared as a before/after; they
differed only in how long the chip had been idle in the bootloader — one followed a 20 s
flash (cool), the other 19 minutes of running (warm), so the *improved* config read hotter.

> To watch a running board: pay one reset, hold the port open, and let the board talk.
> That is why the firmware prints a periodic `pwr:` line rather than answering a query.

Note this contradicts the general rule in `CLAUDE.md` that serial reads are free on the
ESP32-S3. It holds for the boards in that note; it does **not** hold here.

**2. The collector's RSSI is useless for this.** It reports
`self.advertisement.rssi`, and `smart-shunt-ble-client.py` line 2 carries its own
`todo use bluetoothctl to get real-time rssi`. A connected peripheral does not advertise,
so the value is frozen at the last advertisement before the current connection. A −38 → −48
shift was briefly read as a TX change taking effect; it was a reconnect picking up a
different advertisement.

Device-side RSSI does not fix it either: measured at the peripheral, `ble_gap_conn_rssi`
reports what the *central* transmits. Only the rpi can measure our TX power, and bumble
owns that HCI adapter exclusively. **TX power is unmeasurable on this bench.**

**3. PSRAM was never running.** The boot banner reads `psram 0 B`.
`CONFIG_SPIRAM_BOOT_INIT` is set in the `qio_opi` sdkconfig, but
`CONFIG_SPIRAM_IGNORE_NOTFOUND` silently tolerates a part that does not answer, and
`ESP.getPsramSize()` would report 8 MB had it initialised and joined the heap
(`SPIRAM_USE_MALLOC` is set). The claimed "8 MB clocked at 80 MHz continuously" lever never
existed, and the bootloader-level `memory_type` change it needed — failure mode: physical
BOOT+RESET — was correctly not made.

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
- If the board must run cooler than ~62 °C, the only remaining lever is an ESP-IDF port to
  get `CONFIG_BT_LE_SLEEP_ENABLE` / `CONFIG_PM_ENABLE` / tickless idle. Nothing at the
  application level will do it.
