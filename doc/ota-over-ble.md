# OTA over BLE

Push firmware to a running shunt over BLE — no WiFi, no USB cable, no unplugging the device from
whatever rig it is wired into.

The receiver is the shared [`esp-ota-ble`](../../esp-ota-ble/README.md) module, also used by
`fugu-mppt-firmware`. This file covers what is specific to the shunt.

## Pushing

```
# stop the collector first -- only one central may hold the link
python3 ../pwr-metering/smart-shunt-ota-ble.py

pio run -e esp32s3                                   # build first, if you have not
python3 ../pwr-metering/smart-shunt-ota-ble.py .pio/build/esp32s3/firmware.bin --name smart-shunt
```

Expect `OTAB READY`, an upload bar, a flush bar, then the device disconnecting and re-advertising.
A full image takes roughly a minute.

## GATT

Both characteristics live on the existing telemetry service
`e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb`:

| UUID | Properties | Role |
|---|---|---|
| `b0e0d1a4-7f52-4a3e-9c61-2d8f5b3ae741` | write, notify | control in, `OTAB …` status out |
| `b0e0d1a5-7f52-4a3e-9c61-2d8f5b3ae741` | write-no-response | firmware bytes |

Commands are bare lines — `begin <size> <sha256hex>`, `end`, `abort` — and the protocol is the one
documented in the module README, including the credit window and the requirement to wait for
`OTAB PROG <size>/<size>` before sending `end`.

**After any change to this GATT layout, macOS will keep serving the cached attribute table** and
clients fail to find characteristics. Bust it with `blueutil -p 0 && blueutil -p 1`, or forget the
device in System Settings. This bites `smart-shunt-ble-client.py` too, not just the OTA tool.

## What the device does during a push

- **Sampling stops.** The OTA quiesce hook sets `g_samplingHalted`, and `realTimeTask` yields
  instead of sampling. Flash erase/write disables the CPU cache and stalls the other core; a tight
  sampling loop there achieves nothing but I2C timeouts and a starved transfer.
- **Telemetry indications stop**, so they do not compete with the status notifications for NimBLE's
  small mbuf pool.
- **The connection interval drops** to 7.5–15 ms for the duration and returns to 30–60 ms
  afterwards. At the relaxed interval a full image would take minutes.
- **Deep sleep is inhibited** — sleeping mid-transfer would leave a half-written slot.

Measurement is therefore interrupted for about a minute. That is the trade; there is no way to write
flash and keep a real-time sampling loop fed at the same time.

## Partitioning

`parts.csv` carries two 0x1B0000 app slots. `nvs` deliberately keeps its original offset and size,
because **calibration lives there** — `settings.h` goes through `<EEPROM.h>`, which on
arduino-esp32 3.x is an NVS blob rather than a partition of its own.

So when reflashing over the wire: `pio run -e esp32s3 -t upload`, which writes bootloader, partition
table and app only. **Never `esptool.py erase_flash`** — it silently destroys every calibration on
the device, and nothing afterwards looks wrong.

Check headroom after any sizeable change; `pio run` prints it, and `begin` rejects an image that does
not fit before erasing anything.

## Rollback

The bootloader has `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`, so a freshly pushed image boots as
`PENDING_VERIFY` and **the next reset reverts it** unless the firmware confirms itself.

Two pieces in `main.cpp` make that a safety net rather than a trap:

- `markOtaValidIfHealthy()` confirms the image only after ~60 s of uptime *and* evidence that the
  sample count is still increasing — it takes two observations, because a cumulative counter that is
  merely non-zero is also satisfied by a build that sampled once and then wedged.
- `bootWatchdogArm()` starts a 30 s one-shot timer at the top of `setup()`, disarmed once the tasks
  are running. An image that hangs inside `setup()` otherwise never resets, so the bootloader never
  gets its chance to roll back — that is exactly how two fugu units were bricked into needing a
  serial cable.

`bootinfo` on the console prints the running slot and its verify state. It is the only way to tell a
rollback from a normal boot: after a bad push the device comes back looking perfectly healthy, just
running the previous image.

## When it goes wrong

| Symptom | Cause |
|---|---|
| `characteristic not found` | stale macOS GATT cache — see above |
| first write after connecting fails, ATT 17 "insufficient resources" | NimBLE ACL buffers exhausted by repeated connect/disconnect cycles; reboot the device for a fresh BLE stack |
| `OTAB FAIL incomplete` / `sha-mismatch` | the image did not arrive intact; the boot partition is untouched, just retry |
| device never re-advertises | it may have rolled back, or be wedged — serial console and `bootinfo` |
| no device found | `smart-shunt-ble-client.py` is probably still holding the link |
