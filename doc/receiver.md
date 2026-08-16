# Receiver

The host side of the BLE telemetry link is **not in this repo**. It lives in
[`pwr-metering`](git@github.com:fl4p/pwr-metering.git), next to the bench tooling that shares its
`util.py` and shells out to its aux CLI:

| File | |
|---|---|
| `smart-shunt-ble-client.py` | the collector — connects, decodes, publishes, drives the aux switch |
| `smart-shunt-aux.py` | CLI that edits `aux-state.json`; the collector applies it over the live link |
| `smart_shunt_clock_selftest.py` | regression test for the clock handling below — no BLE, no network |
| `aux_control_selftest.py` | the aux control-file reader |

It stays there rather than here because that is where its dependencies point: `ate/fan.py` invokes
the aux CLI (locally and over ssh), `ate/fan_selftest.py` reads the collector's source to get its
real constants, and the rig, the venv and the deploy target all sit in that tree. This firmware repo
has no code dependency on it in the other direction — only the wire contract below.

## Deploying

`rpi.local:~/pwr-metering` is a **git clone** (since 2026-08-16; it was a one-way PyCharm rsync
before, which let the box drift ~118 lines behind and stranded nine measurement runs that existed
nowhere else). So the box can be asked what it is running, and deployment is a pull:

```sh
ssh rpi.local 'cd pwr-metering && git status --porcelain && git log --oneline -1'   # what's running
ssh rpi.local 'cd pwr-metering && git pull' && ssh rpi.local 'sudo systemctl restart smart-shunt-ble'
```

The service is `smart-shunt-ble.service`. PyCharm autoUpload is deliberately **off** — an automatic
push into a checkout leaves `git status` permanently dirty, and a checkout that is always dirty
cannot answer the question it exists to answer.

Only one BLE central may hold the link, which is why aux control and OTA both go *through* the
collector rather than connecting alongside it.

## Wire contract

Defined by this firmware (`src/adc/sampling.h`, `src/energy_counter.h`) and decoded there, so a
change on either side has to be made with the other in mind.

- **`dev[16]`** — filled by `std::string::copy()`, which **appends no terminator**. A name of
  exactly 16 characters arrives with no NUL, so a decoder must not assume one; `dev[:dev.find(0)]`
  silently drops the last character when `find()` returns −1. Names longer than 16 bytes are
  truncated on the wire, and two names differing only past character 16 become one InfluxDB series —
  see `src/board_prefix.h`, which checks for exactly that.
- **Frame size** — 56 or 64 bytes; the 64-byte variant added the `diag` field, shifting everything
  after it. The collector auto-detects from the first CRC-valid frame.
- **CRC** — CRC-16 MODBUS over everything preceding it.
- **`idx`** — `uint8`, and it **wraps**; a gap check must mask.
- **`t`** — `gettimeofday()` **milliseconds on the device**, in one of two shapes, with nothing in
  the frame saying which:
  - **Unix ms**, if the board reached `timeSync()`.
  - **ms since boot**, on a BLE-only board — no WiFi, so SNTP never runs. The collector recovers
    wall-clock time as `min(host_now - t)` over frames: each observation is the true offset plus a
    non-negative transport delay, so the smallest is the least contaminated.

  Both shapes must be handled, and so must the **switch between them on a live link** — it happens
  the moment a device gets WiFi. A frame that cannot be placed on the host timeline is dropped
  rather than stamped with its arrival time; reception time is a different quantity from
  measurement time, and nothing downstream could separate them afterwards.

## In InfluxDB

The collector tags points `BLE_` + the name in `dev[16]`, so the series key is the sampler name this
firmware chose. That name only has to be unique within one board, but the **database is shared** by
every board reporting into it — hence the NVS board prefix (`board-prefix` on the console,
`src/board_prefix.h`). Two boards running this firmware both published `BLE_TMP117` before it
existed.

Transport is UDP to a relay on the rpi (`influxdb-udp-relay.service`), which batches and forwards
over HTTP. UDP means a gap reads as "not delivered", never as "unchanged" — which is why the aux
series has a heartbeat rather than being change-only.
