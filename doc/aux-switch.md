# Aux switch

One general-purpose switched output on **GPIO 10**, driven from BLE or the serial console. The shunt
otherwise only measures — this is the one thing it can actuate.

## Hardware

| | |
|---|---|
| Pin | GPIO 10 |
| Polarity | **active-high** — high = on |
| Drive | logic level only; needs an external FET/driver for any real load |
| Required | **a pulldown on the driver gate** |

GPIO 10 was chosen because it is free (undeclared in `settings.h`), not a strapping pin, not USB,
not flash, and is **RTC-capable** — that last property is what lets it hold its level through deep
sleep. Confirm the pin is actually broken out before wiring: `hw/` documents the INA228 sensor board,
whose MCU sheet is an ESP-12F, not the ESP32-S3 module the firmware runs on. Nothing in this repo
records the module's pinout or how it is wired to the sensor board.

**The pulldown is not optional.** From reset until `auxBegin()` runs, the pin is a floating input —
no firmware can cover that window, so the hardware has to hold the load off through boot.

## Controlling it

**Serial console:**

```
aux            # status
aux on
aux off
aux toggle
```

**Over BLE** — characteristic `b952dad5-9541-4852-bab6-96b9cbc9131a` on the existing service
`e8308d3d-…`, `READ | WRITE | NOTIFY`. The value is a single ASCII byte `'0'` or `'1'`, so a read or
a notification needs no parsing. Writes also accept raw `0x00`/`0x01` and the words `on`, `off` and
`toggle`, so a generic BLE app can work the switch too. An unparseable write is ignored and the
characteristic is re-published, so its value never shows something the pin is not doing.

**From the host:** only one BLE central may hold the link, and `smart-shunt-ble-client.py` normally
does. So aux control goes *through* the collector: `pwr-metering/smart-shunt-aux.py` edits a small
JSON control file (`aux-state.json`), and the collector pushes any change over the connection it
already has.

```
./smart-shunt-aux.py on                       # every shunt
./smart-shunt-aux.py off --device AA:BB:CC:…  # just one
./smart-shunt-aux.py status
```

The collector reads the device's own state before writing, so a reconnect costs no flash write.

A missing, unreadable or nonsense control file makes the collector command **nothing at all**. It
does not fall back to off: a file going away is not evidence about what the switch should do, and
dropping a load because of it would be a change nobody asked for. Covered by
`pwr-metering/aux_control_selftest.py`.

## In InfluxDB

Measurement **`smart_shunt_aux`**, tagged `address` / `name` / `local_name`:

| Field | Meaning |
|---|---|
| `state` | what the **device reports** the pin is doing (0/1) |
| `wanted` | what the control file asked for (0/1) |

They are separate on purpose. `state` is evidence; `wanted` is intent. A window where they disagree
is the interesting event — a write that was refused, a device that came back from a reset before the
collector re-asserted, or a load that is not doing what you think.

`state` is **omitted** whenever the collector does not actually know it (right after issuing a write,
before the device has confirmed). A gap is honest; publishing the commanded value there would turn
an assumption into a recorded measurement.

Points are written **immediately on any change** and again as a **heartbeat** every 30 s. Both are
needed: the transport is UDP, so a change-only series would lose a transition to a dropped datagram
and then show the wrong level indefinitely, while a heartbeat means a gap reads as "not delivered"
rather than "unchanged".

## State across resets

The commanded state is persisted in NVS (raw EEPROM bytes 152–153, clear of the calibration slots)
and restored at the top of `setup()`.

A **magic byte** distinguishes "never written" from "written off". A virgin NVS image reads back as
zeros, so without it the two would be indistinguishable — and any encoding where a blank device could
read as *on* would energise a load on first boot. If the magic does not match, the switch comes up
off.

NVS is written **only when the value actually changes**: `EEPROM.commit()` rewrites the whole
256-byte blob, so re-asserting an already-stored value would burn a flash cycle for nothing.

## Deep sleep

The device deep-sleeps after an hour idle and wakes every 15 minutes through a **full reset**. Before
sleeping, `auxArmDeepSleepHold()` latches the pad (`gpio_hold_en` + `gpio_deep_sleep_hold_en`) so the
level carries across the sleep and the wake. `auxBegin()` then configures the pin *before* calling
`gpio_hold_dis()`, so the pad goes straight from held-old to driven-new with no blink in between.

Without that, a switch left on would drop out for the whole boot interval every 15 minutes, and
nothing would say so.

The switch deliberately does **not** inhibit sleep — it latches instead, and the hold makes sleeping
while on harmless.

`ESP.restart()` (an OTA reboot, say) is *not* a deep sleep, so the hold is not armed for it. Whether
the load drops across a firmware push is worth measuring on the bench rather than assuming; the
pulldown means it fails off if it does.
