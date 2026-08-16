# Receiver

The host side of the BLE telemetry link: it connects to every advertising smart shunt, decodes
`WireSample` frames and writes them to InfluxDB over UDP.

| File | |
|---|---|
| `smart-shunt-ble-client.py` | the collector — connects, decodes, publishes, drives the aux switch |
| `smart-shunt-aux.py` | CLI that edits `aux-state.json`; the collector picks the change up |
| `util.py` | the three helpers the collector imports (see the file header) |
| `clock_selftest.py` | regression test for the device-clock handling — no BLE, no network |

```
python3 receiver/clock_selftest.py
```

## Where it actually runs

On `rpi.local`, as `/home/fab/pwr-metering/smart-shunt-ble-client.py`. That directory is **not a
git checkout** — it is an rsync/PyCharm push — so the box cannot be asked what version it is
running, and on 2026-08-16 it was ~118 lines behind the source repo. Worth checking before
concluding that a fix is live:

```sh
ssh rpi.local md5sum pwr-metering/smart-shunt-ble-client.py
md5 -q receiver/smart-shunt-ble-client.py
```

## Two copies

This directory is a **copy**. The collector's other home is `pwr-metering`, alongside the bench
tooling that shares its `util.py`, and that is the copy the rpi is deployed from.

That is a real hazard and worth stating plainly rather than discovering later: nothing enforces
that the two stay equal, and a fix applied to one is invisible from the other. They are byte-equal
as of 2026-08-16 and the import lines are deliberately spelled the same in both so a plain `diff`
is meaningful:

```sh
diff -u ../pwr-metering/smart-shunt-ble-client.py receiver/smart-shunt-ble-client.py
```

The reason for a copy here at all is that the frame layout is defined by this firmware — `dev[16]`,
the 56/64-byte variants, the CRC, the meaning of `t` — so a change on this side has to be made
together with the decoder. Picking one repo as the single home would be better than diffing; that
decision has not been made.

`util.py` here is **not** the same file as `pwr-metering/util.py`. It is the three functions the
collector needs, extracted verbatim; the original drags in pandas, pytz and dateutil for query and
calibration helpers the receiver never calls.

## Requirements

`bleak`, plus `bluek` (the D-Bus-free multi-central shim `import bluek.shadow` swaps in). The
firmware advertises one connectable peripheral and **only one central may hold the link** — that is
why aux control goes through the collector rather than a second connecting process.

## Timestamps

`WireSample.t` is `gettimeofday()` **milliseconds on the device** (`src/adc/sampling.h`). It arrives
in one of two shapes and nothing in the frame says which:

- **Unix ms** — the board reached `timeSync()`. Used as-is.
- **ms since boot** — a BLE-only board, where SNTP never runs because there is no WiFi. The
  collector estimates the boot instant as `min(host_now - t)` over frames: every observation is the
  true offset plus a non-negative transport delay, so the smallest is the least contaminated, and
  points are held until that estimate stops improving.

The two are told apart by magnitude (`EPOCH_MS_MIN`), and the switch from one to the other **while
a link is up** is handled — it happens the moment a device gets WiFi.

A frame that cannot be placed on the host timeline at all is **dropped**, not stamped with the time
it arrived. Reception time is a different quantity from measurement time and nothing downstream
could tell them apart afterwards.

`clock_selftest.py` covers all of this, including a host clock that steps forward — an rpi with no
RTC does exactly that the first time chrony syncs, and this collector runs on one.
