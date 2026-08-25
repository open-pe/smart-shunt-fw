# smart-shunt-fw

Firmware for the bench current/voltage shunt monitors. Targets: ESP32-S3 (primary) and
STM32H5 (port). `doc/receiver.md` owns the **wire contract**; this file owns the
**deployment path** — where bytes actually go, and how to read them back.

## Build and flash

```sh
pio run -e esp32s3_n16r8 -t buildprog                       # build only
pio run -e esp32s3_n16r8 -t upload --upload-port /dev/cu.usbmodem1101
```

Environments: `esp32s3`, `esp32s3_n16r8`, `stm32h5`. (`devkitc_s3` and `feather_s3` are
gone — old transcripts still name them.)

- **A bare `pio run -e <env>` UPLOADS AND THEN OPENS THE MONITOR.** `platformio.ini:12` sets
  `targets = upload, monitor` and every env inherits it, so the innocent-looking build command
  touches the board. Use `-t buildprog` to compile without flashing.
- **`-t build` is a no-op here.** It reports SUCCESS in ~2 s and recompiles nothing, even after
  the source is touched — it looks like a passing build and is not one. `-t buildprog` is the
  target that actually compiles and links.
- **Always pass `--upload-port` explicitly.** `platformio.ini` sets `upload_port=/dev/cu.usb*`,
  and the glob does not reliably resolve to `/dev/cu.usbmodem1101`: the upload step then fails
  with "Couldn't find a board on the selected port" even though the build and link succeeded.
  Read the tail, not the status line. (This env sets `board_upload.use_1200bps_touch = no` and
  `wait_for_upload_port = no`, so the 1200-baud reset dance is *not* the explanation.)
- Uploads flake intermittently (`Error -15`, or a mid-write drop). Retry once before
  investigating; it is USB, not the build.
- **`pio device monitor` does not work here** — miniterm dies on `termios.error: Operation not
  supported by device`. Read the port with pyserial instead.
- Opening the port with default DTR/RTS can leave the board wedged and silent. If it goes
  quiet and re-flashing also fails, it needs a physical **BOOT + RESET** to re-enter download
  mode — ask, don't try to fix it over the wire.
- **Before concluding "the board is silent", run `lsof /dev/cu.usbmodem1101`.** A leftover
  reader — a capture script that outlived its loop, a backgrounded task that was stopped but
  not reaped — keeps the port open and *drains* it, so a second reader gets zero bytes. The
  symptom is indistinguishable from a wedged board: no output, no boot banner, nothing. Kill
  the holding PID and read again before reaching for BOOT + RESET or a reflash. (Seen twice in
  one session; each time it sent the diagnosis down the wrong path, once as far as blaming an
  unrelated code change for a boot loop that was not happening.) `lsof` showing a non-zero
  `0t<offset>` on the device is the tell — that process has consumed that many bytes.

## Data plumbing (end to end)

```
board ──BLE indications──> smart-shunt-ble.service   (rpi.local)
                             │  decodes WireSample, tags device = "BLE_" + dev[16]
                             ▼
                           UDP 127.0.0.1:8086
                             ▼
                           influxdb-udp-relay.service (rpi.local)
                             │  batches, forwards over HTTPS
                             ▼
                           influx.fabi.me:8086  (InfluxDB 1.x, TLS)
                             database: open_pe   measurement: smart_shunt
```

**The firmware's own InfluxDB writer is dormant, not dead.** `influxWritePointUDP()` in
`src/util.cpp` targets a hardcoded `192.168.178.180:8086`, a host that does not exist on the lab
LAN. `disableWifi = true` is the **boot default** (`src/main_esp32.cpp`), so `onTelemetryFlush()`
does not call it and **every byte normally reaches InfluxDB via BLE**. But the `wifi on` console
command (`src/console.h`) clears `disableWifi` at runtime and arms that second path — so on a
board where someone typed it, points are being fired into the void over UDP as well. Do not debug
telemetry by probing that IP; equally, do not assume the path is unreachable.

Other facts that cost real debugging time:

- **Collector source lives in `pwr-metering`**, not here (`smart-shunt-ble-client.py`); see
  `doc/receiver.md` for deploy. `rpi.local:~/pwr-metering` is a git clone, so it can be asked
  what it runs.
- **Only one BLE central may hold a link**, which is why aux control and OTA go *through* the
  collector rather than connecting alongside it. Do not attach a second scanner casually.
- **Series names truncate to 16 chars** in `dev[16]`, so console and database names differ:
  `ftr_SHUNT_ADC_HEALTH` is `BLE_ftr_SHUNT_ADC_HE` in InfluxDB. Grepping the serial log for the
  name you see in Grafana finds nothing.
- **Fields are overloaded per channel, and units differ per channel.** `U`/`I`/`P` do not always
  mean voltage/current/power, and there is no single unit convention — check the sampler's FIELD
  MAPPING comment before reading a series.
  - `SHUNT_ADC`: ordinary volts and **amperes** (`shuntOhm` is set, so `I = U / 0.002`).
  - `SHUNT_ADC_ZERO`: `U` is the offset in volts, `I` is a within-burst standard deviation
    **in volts, not a current**, `P` is always 0.
  - `SHUNT_ADC_HEALTH`: `U` is fCLK in **Hz**, `I` is AVDD−AVSS in volts, `P` is always 0.
  - `DCCT`: J4/PAIR_CH1, a 500:N current transformer into a 5 Ω burden. `U` is the burden
    voltage in **volts** (unscaled — it doubles as a range readout), `I` is **primary**
    amperes, `P` is always 0. `I` is only correct for the turns count actually rigged: the
    N/100 ratio is carried by the EEPROM calibration factor (`calibrate DCCT I <1/N>`), which
    is the *same* factor any gain trim lives in — re-rigging N overwrites the trim.
    This pair runs G=1 with the **PGA bypassed** (2.4 V full scale is far outside the G=1
    PGA window), so the PGA over-range alarms cannot fire; over-range is caught digitally at
    ~98% of ±2.5 V FS and reported as `DIAG_PGA_RANGE` with `U`/`I` = NaN.
- **Registering `DCCT` un-parks the ADS1262 mux.** With only `SHUNT_ADC` registered the mux
  sits on CH0 and reaches its full 19.15 SPS; with both, the device scans CH0↔CH1 and each
  channel updates at **~4.8 SPS** (every pair switch restarts conversion, ~104 ms chopped).
  Still above the ~2.5 SPS on-air rate. Individual samples are no noisier — same gain, same
  filter, one conversion each — but any *fixed-duration average* is ~2× less certain, since a
  quarter as many CH0 conversions arrive. Commenting out the `samplers.add("DCCT", …)` line
  parks CH0 again automatically.
- NaN is **dropped, not stored**: `Point::addField` skips it, and the collector's `write_point`
  likewise skips `None`. An all-NaN sample therefore yields a point with tags, a timestamp and
  *no fields*. The current collector detects and counts those locally instead of sending them;
  older revisions did send them, and InfluxDB answers with a 400 `partial write` that rejects
  the offending point (valid points in the same batch still land).

## Reading historical data from InfluxDB

Credentials live only on the rpi, in `~/influxdb-udp-relay/configuration.yaml`. **Do not copy
them into this repo or into a command line.** Run the query on the box, letting it read its own
config:

```sh
ssh rpi.local "cd influxdb-udp-relay && venv/bin/python3 -c \"
import yaml, influxdb, urllib3; urllib3.disable_warnings()
o = yaml.safe_load(open('configuration.yaml'))['influxdb']
c = influxdb.InfluxDBClient(host=o['host'], port=int(o.get('port', 8086)),
                            username=o['username'], password=o['password'],
                            database=o['database'], ssl=True, verify_ssl=False)
q = \\\"SELECT mean(I), mean(U), mean(T) FROM smart_shunt \\
      WHERE device='BLE_ftr_SHUNT_ADC' AND time > now() - 6h GROUP BY time(1m)\\\"
for p in c.query(q).get_points(): print(p)
\""
```

- measurement `smart_shunt`; tag `device`; fields `I`, `U`, `I_max`, `U_max`, `P`, `E`, `T`,
  `diag` (the collector adds `diag`; the firmware's own dead UDP writer does not)
- measurement `smart_shunt_meta` carries per-link metadata (address, rssi)
- `SHOW TAG VALUES FROM smart_shunt WITH KEY = device` lists what is actually reporting
- times are device-derived (see `doc/receiver.md` on `t`), not reception times

## When no data arrives

Work the chain outward; each stage has its own evidence.

```sh
ssh rpi.local 'systemctl is-active smart-shunt-ble influxdb-udp-relay'
ssh rpi.local 'journalctl -u smart-shunt-ble    --no-pager -n 30'   # is the board connected?
ssh rpi.local 'journalctl -u influxdb-udp-relay --no-pager -n 30 | grep -i error'
ssh rpi.local 'bluetoothctl devices'                                # BlueZ cache, no scan
```

If the collector prints `no smart-shunt seen yet, scanning for Ns` while the board is plainly
advertising, **BlueZ discovery has gone deaf** — a documented failure of this setup, not a
firmware bug.

```sh
ssh rpi.local 'sudo systemctl restart bluetooth && sleep 5 && sudo systemctl restart smart-shunt-ble'
```

**Restart `bluetooth` first.** Restarting the collector alone can *induce* the deafness: it
drops every connected board and then finds nothing, so one board's outage becomes three. This
interrupts collection for every board on the bench, so confirm before doing it.

The collector is **not** passive about this: it scans continuously, retries known addresses whose
advertisements go stale, restarts its own scanner, and probes the adapter with an unfiltered scan
to tell "nothing is advertising" from "this adapter is deaf". So reach for the restart only after
those have visibly failed in the log — a board missing for a minute is normal, a board missing for
many minutes with the scanner reporting silence is not.

Observed 2026-08-16: a board that was boot-looping and being reflashed while the collector last
scanned stayed absent for ~30 minutes, and only the `bluetooth`-first restart recovered it.
