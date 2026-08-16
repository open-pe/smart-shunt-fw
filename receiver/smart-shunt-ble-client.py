"""
- todo use bluetoothctl to get real-time rssi
"""

import asyncio
import collections
import json
import os
import struct
import time
import traceback
from collections import defaultdict
from typing import Dict, Optional

import bluek.shadow  # noqa: F401 — redirect import bleak -> bluek (no D-Bus, multi-central)
import bleak
from bleak import AdvertisementData, BLEDevice, BleakGATTCharacteristic, BleakError, BleakClient

from util import write_point, crc_modbus, remove_nan

WIRE_SAMPLE_CHAR = "df51a73d-0b60-43a5-bc86-a043f3841152"
AUX_CHAR = "b952dad5-9541-4852-bab6-96b9cbc9131a"

# Desired aux-switch state per device, edited by smart-shunt-aux.py. We push changes over the link
# we already hold, because only one central may be connected at a time -- a separate CLI that
# connected itself would be unusable whenever this collector runs.
AUX_STATE_FILE = os.environ.get(
    'SMART_SHUNT_AUX_FILE',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'aux-state.json'))

# after a failed connect (or a dropped client) wait a bit before trying that address again,
# otherwise we hammer BlueZ with a device object it has already thrown away
RECONNECT_COOLDOWN = 5.0
RECONNECT_COOLDOWN_MAX = 60.0

# no frame for this long -> the link is stale, tear it down and reconnect
RX_TIMEOUT = 10.0

# BlueZ can wedge inside disconnect().  Unbounded, that hangs the client's loop task
# forever, so is_good() stays True, the client is never popped and nothing ever
# reconnects -- the failure is invisible because the supervisor thinks all is well.
DISCONNECT_TIMEOUT = 10.0

# the loop task itself is stuck somewhere (not just the link): last resort, cancel it
STALL_HARD_TIMEOUT = 30.0

# no advertisement for this long -> the scanner callback is not going to fire for this
# address, so the supervisor retries the connection itself instead of waiting forever
ADV_STALE = 20.0

# a known device missing this long with the scanner silent -> restart the scanner, in
# case BlueZ discovery has gone deaf rather than the device having gone away
SCANNER_RESTART_AFTER = 120.0

# Adapter liveness probe.
#
# The main scanner filters on the smart-shunt service UUID, so its callback CANNOT
# fire for anything else. That makes silence ambiguous in the worst possible way:
# "no peripheral is advertising" and "this adapter has stopped scanning entirely"
# look identical, and the log reports both as "no smart-shunt seen yet" -- an
# absence of evidence printed as evidence of absence.
#
# Observed 2026-08-14: BlueZ went deaf after a client restart. `bluetoothctl scan le`
# returned ZERO devices of any kind -- never true in a home -- while this client
# happily reported "scanning for 272s" as though the bench were empty. Restarting
# bluetooth.service fixed it; the client had no way to know that was the fix.
#
# The probe removes the ambiguity by scanning UNFILTERED for a few seconds. Any
# advertisement at all, from any device, proves the adapter works and the
# peripherals are genuinely absent. Zero proves the adapter is deaf.
SCANNER_PROBE_AFTER = 90.0       # total silence this long -> go find out which it is
SCANNER_PROBE_SECONDS = 8.0      # how long to listen; a quiet flat still yields several
SCANNER_PROBE_COOLDOWN = 240.0   # min seconds between probes (each costs a scan restart)

# The device clock, in the two shapes it can arrive in.
#
# WireSample.t is gettimeofday() MILLISECONDS on the device (firmware: adc/sampling.h calls
# getTimeStamp(&u_time, 3)). A board that reached timeSync() therefore sends real Unix ms, while a
# BLE-only board -- no WiFi, so SNTP never runs -- has never had its clock set and sends
# milliseconds-since-boot instead. Both are 64-bit ms and NOTHING IN THE FRAME SAYS WHICH, so the
# shape has to be inferred from magnitude, and both have to be handled -- including the switch from
# one to the other partway through a link, which is what happens the moment a device gets WiFi.
EPOCH_MS_MIN = 1_000_000_000_000  # 2001-09-09. Below it: ~31 years of uptime, which no board has.

# Uptime mode only. The boot instant in host time is estimated as min(host_now - t) across frames:
# every observation is the true offset PLUS the link's transport delay, which is non-negative, so
# the smallest observation is the least contaminated one and the estimator can only improve.
CLOCK_SETTLE_FRAMES = 5      # consecutive frames without a better estimate -> start writing
CLOCK_BUF_MAX = 4096         # points held while the estimate settles; ~8 SPS x 4 samplers x 2 min

# An observation this far ABOVE the current offset cannot be transport delay any more, so the
# premise the offset rests on is gone -- see _resolve_timestamp.
CLOCK_REANCHOR_MS = 30_000
CLOCK_DRIFT_WARN_MS = 1_000

# how often to say out loud that we are still waiting (and record it in influx)
STATUS_INTERVAL = 30.0

# How often the aux pin state is REPUBLISHED when nothing has changed. A change is always written
# immediately; this is only the keep-alive cadence for an unchanged state.
#
# SPLIT OUT OF STATUS_INTERVAL, which it used to share. They are different jobs -- one is a human
# progress message about a device we are still waiting for, the other is the evidence a consumer
# reads to decide whether the fan was running -- and tying them together meant the evidence rate
# could not be raised without making the waiting log six times noisier.
#
# 5 s, from the consumer side. ate/fan.confirm_interval requires at least one aux point INSIDE the
# interval it verifies, so the heartbeat is a hard floor on how short a dwell can be and still be
# confirmable. At 30 s it cost ~35 s of every rung; measured 2026-08-12, a 19 s dwell landed
# between two bursts (21:35:47 and 21:36:17) and aborted the run UNVERIFIED.
#
# IT IS CHEAP BECAUSE IT IS NOT A BLE POLL. The aux state is already re-read every
# AUX_CHECK_INTERVAL (1 s) and this only decides how often an UNCHANGED value is written to
# InfluxDB. The cost is database rows -- 12/min per device instead of 2 -- not radio traffic on a
# shared adapter, which matters on a bench where EMI already corrupts shunt samples at load.
AUX_HEARTBEAT_INTERVAL = 5.0

# how often a client re-checks the aux control file / device state
AUX_CHECK_INTERVAL = 1.0

# BlueZ wedge recovery: when ALL connect attempts fail with "Operation already in progress"
# or "br-connection-canceled" for this many consecutive attempts across all devices, the
# controller is stuck — per-device backoff never succeeds. Restarting the HCI bridge
# service + btmgmt power cycle resets it.
# See memory: rpi-bluez-recovery.md.
BLUEZ_WEDGE_THRESHOLD = 5
BLUEZ_WEDGE_COOLDOWN = 120.0  # min seconds between recovery attempts

# The systemd service that drives the USB BLE HCI bridge (esp32-h3 on /dev/ttyACM0).
# Restarted during wedge recovery to get a clean controller. Override via env so the
# service unit can name its own dependency without hardcoding it here.
HCI_BRIDGE_SERVICE = os.environ.get('HCI_BRIDGE_SERVICE', 'esp32-hci.service')

# conn-params debugfs paths — lost on btmgmt power cycle, must be re-applied
CONN_PARAM_FILES = {
    'conn_max_interval': '160',
    'conn_min_interval': '80',
    'supervision_timeout': '400',
}

# A heartbeat is evidence only while the underlying observation is fresh. Without this bound a
# wedged notification subscription leaves the collector republishing the last state forever, so a
# stale pin level is indistinguishable from a live one.
#
# DELIBERATELY NOT RETIED to AUX_HEARTBEAT_INTERVAL when that was split out, and this is the whole
# reason the split was worth making. It used to be `2 * STATUS_INTERVAL`, so speeding up the
# publish cadence would have silently dragged the staleness bound from 60 s down to 10 s and
# started omitting `state` on a link that was merely bursty -- tightening a guard as a side effect
# of an unrelated speed change, with the consumer then aborting runs UNVERIFIED.
#
# The two quantities are independent: how often we WRITE an unchanged value, versus how stale the
# underlying reading may be before we stop vouching for it. 60 s is the latter, kept absolute.
AUX_STATE_MAX_AGE = 60.0


class AuxWanted:
    """Reads the aux control file, cached on mtime so the 100ms client loop doesn't re-parse JSON.

    Every failure path here returns None, meaning 'no opinion' -- never a state. A missing or
    unparseable file is not evidence that the user wants the switch off, and driving a load because
    a file went away (or because someone was mid-write when we read it) would be a change nobody
    asked for. When in doubt, leave the device exactly as it is.

    File format: {"<address>": true, "*": false}.  "*" applies to any device without its own entry.
    """

    def __init__(self, path: str):
        self.path = path
        self._data: dict = {}
        self._last_logged = None  # only announce a genuine change in content
        self._complained = None   # last complaint, so a persistent problem logs once, not per poll

    def _complain(self, msg: str):
        if self._complained != msg:
            print('aux:', msg)
            self._complained = msg

    def _reload(self):
        # Read every time rather than caching on mtime. Caching on mtime was wrong: file timestamp
        # granularity is the kernel tick on the bench rpi, so two edits inside ~10ms share an mtime
        # and the second one is never seen -- the cache serves stale state with nothing to show for
        # it. This is called at most once per second per device on a file of a few hundred bytes,
        # so there was nothing to optimise in the first place.
        try:
            with open(self.path) as f:
                data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError(f'expected an object, got {type(data).__name__}')
        except FileNotFoundError:
            if self._data:
                self._complain(f'{self.path} disappeared; leaving every switch as it is')
            self._data = {}
            self._last_logged = None
            return
        except Exception as e:
            self._data = {}
            self._last_logged = None
            self._complain(f'cannot read {self.path} ({e}); leaving every switch as it is')
            return
        self._data = data
        self._complained = None
        if data != self._last_logged:
            self._last_logged = dict(data)
            print('aux: loaded', self.path, '->', data)

    def for_device(self, address: str, name: Optional[str]) -> Optional[bool]:
        self._reload()
        for key in (address, name, '*'):
            if key is None or key not in self._data:
                continue
            v = self._data[key]
            if isinstance(v, bool):
                return v
            # A present-but-nonsense value is a typo, not an instruction. Say so and do nothing,
            # rather than coercing "maybe"/1/"on" into a state the user may not have meant.
            self._complain(f'{self.path}: value for {key!r} is {v!r}, not true/false; ignoring')
            return None
        return None


aux_wanted = AuxWanted(AUX_STATE_FILE)


async def find_dev_with_server():
    device: BLEDevice = None
    advertisement: AdvertisementData = None
    stop_event = asyncio.Event()

    def callback(device_: BLEDevice, advertising_data: AdvertisementData):
        nonlocal device, advertisement
        device = device_
        advertisement = advertising_data
        print('found server', device.name, advertising_data.local_name, 'tx_pow=', advertisement.tx_power, 'RSSI=',
              advertisement.rssi)
        stop_event.set()

    async with bleak.BleakScanner(callback, service_uuids=["e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb"]) as scanner:
        await stop_event.wait()

    return device, advertisement


async def wire_sample_callback(characteristic: BleakGATTCharacteristic, data: bytearray):
    print(data)


async def start_scanner(callback):
    scan = bleak.BleakScanner(callback, service_uuids=["e8308d3d-c3b4-45ff-ba58-9c0fb99d0ecb"])
    await scan.start()
    return scan


async def probe_adapter(seconds=SCANNER_PROBE_SECONDS):
    """Count DISTINCT BLE addresses seen in an unfiltered scan.

    Returns the count, or None if the probe could not be carried out at all.

    The None is the whole point and must be preserved by every caller: a probe that
    threw has NOT shown that the adapter is deaf, and treating it as 0 would trigger
    a controller reset on the strength of a failed measurement. Nor may it be read as
    "fine" -- it is neither. Three states, all distinguishable:

        n > 0   adapter scans; the peripherals are genuinely not advertising
        n == 0  adapter is deaf; the peripherals tell us nothing about it
        None    unknown; we learned nothing and must not act as if we did

    The caller must stop the main scanner first: BlueZ does not usefully run two
    discovery sessions with different filters at once, and the second one silently
    inherits the first one's filter -- which would make this probe agree with the
    thing it exists to cross-check.
    """
    seen = set()

    def cb(device, advertising_data):
        seen.add(device.address)

    try:
        async with bleak.BleakScanner(cb):   # NO service_uuids: that is the point
            await asyncio.sleep(seconds)
    except Exception as e:
        print('  adapter probe could not run: %s: %s' % (type(e).__name__, e))
        return None
    return len(seen)


class ClientState:
    def __init__(self, client: BleakClient, char: BleakGATTCharacteristic, advertisement: AdvertisementData):
        self.client: BleakClient = client
        self.char = char
        self.advertisement = advertisement
        self.last_idx = defaultdict(lambda: -1)
        # Device clock. `clock_mode` is None until the first frame classifies it; `clock_offset_ms`
        # is meaningful in 'uptime' mode only and is None until anchored. See _resolve_timestamp.
        self.clock_mode: Optional[str] = None
        self.clock_offset_ms: Optional[int] = None
        self.clock_settled = 0
        self.n_frames_no_time = 0
        self.n_points_dropped = 0
        self.t_last_rx = time.time()
        self.adc_device_names = set()
        self.n_updates = 0
        self.buf = bytearray()
        self.use_read = False
        self.frame_size = None  # auto-detected: 56 or 64, from first CRC-valid frame

        # Points held while the uptime offset is still settling, so they can be stamped with the
        # final estimate rather than the first guess. BOUNDED: it used to be unbounded and, for a
        # time-synced device, unreachable by the flush condition -- so it grew for the life of the
        # process while not one point was ever written.
        self.points_buf = collections.deque(maxlen=CLOCK_BUF_MAX)
        self._loop_task:asyncio.Task= None

        # Aux switch. `aux_actual` is what the device last told us; it is the source of truth, so we
        # only write when the wanted state differs from it. That keeps a reconnect from costing an
        # NVS write on the device (each one rewrites its whole 256-byte EEPROM blob).
        self.aux_actual: Optional[bool] = None
        self.aux_actual_at = 0.0
        self.aux_wanted: Optional[bool] = None
        self.aux_char: Optional[BleakGATTCharacteristic] = None
        self.t_aux_check = 0.0
        self.t_aux_point = 0.0
        self._aux_reported = object()  # sentinel: nothing published yet

    def _notify_callback(self, char: BleakGATTCharacteristic, b: bytearray):
        if char.uuid != WIRE_SAMPLE_CHAR:
            print('unknown char UUID notify callback', char, b)
        else:
            #print('notify callback',len(b))
            self.buf.extend(b)

    def _decode_frame(self, d: bytearray):
        ver, idx, dev = struct.unpack('BB16s', d[:18])
        # The firmware fills this fixed 16-byte field with std::string::copy(), which
        # appends NO terminator, so a sampler name of exactly 16 characters arrives
        # with no NUL at all. `dev[:dev.find(0)]` would then see find() == -1 and slice
        # to dev[:-1] -- silently dropping the LAST CHARACTER and inventing a device
        # name one character short of the real one. That is worse than an error: the
        # points keep flowing, under a tag that looks plausible and is wrong, and
        # InfluxDB indexes it as a separate series forever.
        nul = dev.find(0)
        dev = (dev if nul < 0 else dev[:nul]).decode('utf-8')
        assert ver == 1
        u, i, p, e, t, temp = struct.unpack('ffffQf', d[18:18 + 28])
        if len(d) >= 64:
            # Sample grew by 4 bytes (diag field), shifting everything after it
            i_max, u_max = struct.unpack('ff', d[18 + 32:18 + 32 + 8])
            diag = struct.unpack('I', d[18 + 32 + 8:18 + 32 + 8 + 4])[0]
            crc = struct.unpack('H', d[18 + 32 + 8 + 4:])[0]
            crc_comp = crc_modbus(d[:18 + 32 + 8 + 4])
        else:
            i_max, u_max = struct.unpack('ff', d[18 + 28:18 + 28 + 8])
            diag = 0
            crc = struct.unpack('H', d[18 + 28 + 8:])[0]
            crc_comp = crc_modbus(d[:18 + 28 + 8])
        assert crc_comp == crc, "checksum error %s vs %s" % (crc_comp, crc)

        if idx == self.last_idx[dev]:
            return

        # idx is a uint8 in the frame, so it wraps: without the mask, every 256th sample of every
        # sampler reported a gap that had not happened.
        if idx != (self.last_idx[dev] + 1) & 0xFF and self.last_idx[dev] != -1:
            print('missed sample', self.last_idx[dev], idx)
        self.last_idx[dev] = idx

        self.t_last_rx = time.time()

        self.adc_device_names.add(dev)

        now_ms = int(time.time() * 1000)
        ts_ms, settled = self._resolve_timestamp(t, now_ms)

        point = dict(
            measurement='smart_shunt',
            tags=dict(device='BLE_' + dev),
            values=remove_nan(dict(I=i, U=u, I_max=i_max, U_max=u_max, P=p, E=e, T=temp, diag=diag or None)),
            t=t,
        )

        if ts_ms is None:
            # Not placeable on the host timeline. DROP it. Stamping it with time.time() would
            # record the moment of reception as the moment of measurement, which is a different
            # (and unmeasured) quantity, and nothing downstream could tell the two apart.
            self.n_points_dropped += 1
            if self.n_points_dropped in (1, 10) or self.n_points_dropped % 1000 == 0:
                print(self.client.address, 'no usable timestamp, dropped %d point(s) so far'
                      % self.n_points_dropped)
        elif settled:
            self._flush_points()
            write_point(point['measurement'], point['tags'], point['values'], timestamp_ms=ts_ms)
        else:
            if len(self.points_buf) == self.points_buf.maxlen:
                # deque(maxlen) discards the oldest silently; count it, or a clock that never
                # settles looks exactly like a clock that settled instantly.
                self.n_points_dropped += 1
            self.points_buf.append(point)

        print('%20s %14s %3i  u=%.4fV   i=%.4fA   t=%.1f°C    rssi=%.0f tx_pow=%s, ' % (
            dev, self._fmt_ts(ts_ms, settled), idx, u, i, temp,
            self.advertisement.rssi, self.advertisement.tx_power))

    def _fmt_ts(self, ts_ms, settled):
        """'*' = reconstructed from device uptime, '?' = held pending a settled offset."""
        if ts_ms is None:
            return '-'
        return '%d%s%s' % (ts_ms, '' if self.clock_mode == 'epoch' else '*', '' if settled else '?')

    def _stamp(self, t: int) -> Optional[int]:
        """Device timestamp -> Unix ms under the CURRENT clock estimate, or None if there is none."""
        if self.clock_mode == 'epoch':
            return t
        if self.clock_mode == 'uptime' and self.clock_offset_ms is not None:
            return t + self.clock_offset_ms
        return None

    def _flush_points(self, offset_ms: Optional[int] = None, mode: Optional[str] = None):
        """Write everything held, stamped with the best offset we ever reached for those frames.

        `offset_ms`/`mode` override the current estimate, for the case where the buffered frames
        belong to a clock mode the device has since left -- their raw `t` means something different
        now, so re-stamping them with the NEW estimate would be wrong by a whole epoch.
        """
        while self.points_buf:
            p = self.points_buf.popleft()
            if mode is None:
                ts = self._stamp(p['t'])
            elif mode == 'epoch':
                ts = p['t']
            else:
                ts = None if offset_ms is None else p['t'] + offset_ms
            if ts is None:
                self.n_points_dropped += 1
                continue
            write_point(p['measurement'], p['tags'], p['values'], timestamp_ms=ts)

    def _resolve_timestamp(self, t: int, now_ms: int):
        """Map a device timestamp onto Unix ms. Returns (timestamp_ms, settled).

        timestamp_ms is None when the frame cannot be placed on the host timeline AT ALL. That is a
        third state, distinct from both a good stamp and a stale one, and the caller must drop such
        a point rather than invent a plausible time for it.

        `settled` is False while the uptime offset is still being estimated. Those points are worth
        holding rather than dropping, because a strictly better offset is due within a few frames.
        """
        if t == 0:
            # A window summary published with no timestamp in it -- nothing to anchor to.
            self.n_frames_no_time += 1
            return None, True

        mode = 'epoch' if t >= EPOCH_MS_MIN else 'uptime'
        if mode != self.clock_mode:
            if self.clock_mode is not None:
                # This is what happens when a device finally reaches timeSync() mid-link. The old
                # code did not consider it: the offset stayed applied to what were now epoch
                # milliseconds, putting the points somewhere around the year 2081 -- except it
                # never got that far, because the drift check below it read a variable that is
                # assigned only on the uptime path and raised UnboundLocalError, tearing down the
                # link on every single frame.
                print(self.client.address, 'device clock switched %s -> %s' % (self.clock_mode, mode))
                self._flush_points(self.clock_offset_ms, self.clock_mode)
            self.clock_mode = mode
            self.clock_offset_ms = None
            self.clock_settled = 0

        if mode == 'epoch':
            # The device knows the wall clock, so its own stamp IS the answer and there is no
            # offset to estimate or settle. The previous code had no branch for this case at all:
            # a time-synced device buffered every point and wrote none, forever.
            drift = t - now_ms
            if abs(drift) > CLOCK_DRIFT_WARN_MS:
                print(self.client.address, 'device clock is %+.1fs off the host clock' % (drift / 1000))
            return t, True

        obs = now_ms - t  # the boot instant in host time, plus this frame's transport delay
        if self.clock_offset_ms is None:
            print(self.client.address, 'device has no time sync, started %.1fs ago' % (t / 1000))
            self.clock_offset_ms = obs
            self.clock_settled = 0
        elif obs < self.clock_offset_ms:
            # A lower-latency frame: strictly a better estimate of the boot instant.
            self.clock_offset_ms = obs
            self.clock_settled = 0
        elif obs - self.clock_offset_ms > CLOCK_REANCHOR_MS:
            # Too large to be transport delay, so whatever the old offset was measuring is gone:
            # either the HOST clock stepped forward -- an rpi with no RTC does exactly this the
            # first time chrony syncs, and this collector runs on one -- or the device restarted
            # its uptime under a link that stayed up. Keeping the old offset would go on recording
            # every later point at the pre-step time, silently and indefinitely, so re-anchor.
            #
            # Only on a LARGE excess. Chasing small ones would ratchet the estimate upward frame by
            # frame and turn the min-estimator into a random walk.
            print(self.client.address, 'clock re-anchored: %+.1fs jump, too large for link latency '
                                       '(host clock step, or the device restarted)'
                  % ((obs - self.clock_offset_ms) / 1000))
            self.clock_offset_ms = obs
            self.clock_settled = 0
        else:
            self.clock_settled += 1

        return t + self.clock_offset_ms, self.clock_settled >= CLOCK_SETTLE_FRAMES

    async def _disconnect(self):
        try:
            if self.client.is_connected:
                # bounded: a wedged BlueZ disconnect would otherwise park the loop task
                # forever, and a loop task that never ends is a client that is never
                # popped and never reconnected
                await asyncio.wait_for(self.client.disconnect(), timeout=DISCONNECT_TIMEOUT)
        except asyncio.TimeoutError:
            print(self.client.address, 'disconnect did not return within %.0fs, abandoning the link'
                  % DISCONNECT_TIMEOUT)
        except Exception as e:
            # BlueZ may already have dropped the device; nothing left to do here
            print(self.client.address, 'error during disconnect:', type(e).__name__, e)

    def _aux_point(self):
        """Publish the switch state.

        Written immediately whenever it changes AND as a periodic heartbeat, because the transport
        is UDP: a change-only series would lose a transition to a dropped datagram and then show the
        wrong level indefinitely, while a gap in a heartbeat series reads as "not delivered" rather
        than "unchanged". `state` is what the device reports the pin is doing; `wanted` is what the
        control file asked for. They should agree -- when they do not, that is the interesting event.
        """
        if self.aux_actual is None and self.aux_wanted is None:
            return  # nothing observed and nothing commanded: no claim to make
        now = time.time()
        actual = (self.aux_actual if self.aux_actual is not None and
                  0 <= now - self.aux_actual_at <= AUX_STATE_MAX_AGE else None)
        cur = (actual, self.aux_wanted)
        if cur == self._aux_reported and now - self.t_aux_point < AUX_HEARTBEAT_INTERVAL:
            return
        self.t_aux_point = now
        self._aux_reported = cur
        write_point(
            'smart_shunt_aux',
            dict(address=self.client.address,
                 name=self.client.name,
                 local_name=self.advertisement.local_name),
            remove_nan(dict(
                state=None if actual is None else int(actual),
                wanted=None if self.aux_wanted is None else int(self.aux_wanted),
            )),
            timestamp_ms=int(now * 1000),
        )

    def _aux_notify(self, _char, b: bytearray):
        if b:
            self.aux_actual = (bytes(b)[:1] == b'1')
            self.aux_actual_at = time.time()
            self._aux_point()  # a transition is exactly what must not wait for the next poll

    async def _aux_reconcile(self):
        """Push the wanted aux state, if any, to the device. Cheap and idempotent."""
        if self.aux_char is None:
            return
        now = time.time()
        if now - self.t_aux_check < AUX_CHECK_INTERVAL:
            return
        self.t_aux_check = now

        wanted = aux_wanted.for_device(self.client.address, self.client.name)
        self.aux_wanted = wanted

        if (self.aux_actual is not None and
                now - self.aux_actual_at > AUX_STATE_MAX_AGE):
            self.aux_actual = None
            self.aux_actual_at = 0.0
            self._aux_point()  # stale state is explicitly omitted before read-back

        if self.aux_actual is None:
            # Never heard the device's own state; read it rather than assuming, so we don't write
            # (and burn a flash cycle) just to set what is already set.
            try:
                v = await self.client.read_gatt_char(self.aux_char)
                self.aux_actual = (bytes(v)[:1] == b'1')
                self.aux_actual_at = time.time()
                self._aux_point()
            except (BleakError, TimeoutError, EOFError):
                # Link-level death. Let it propagate: _loop treats these as end-of-life for this
                # client so the supervisor pops it and reconnects. Swallowing them here kept a dead
                # client "good" forever, retrying against a closed connection while the supervisor
                # believed all was well -- exactly the invisible failure the timeouts above exist
                # to prevent.
                raise
            except Exception as e:
                print(self.client.address, 'aux read failed:', type(e).__name__, e)
                return

        if wanted is None:
            self._aux_point()  # fresh read-back, but no command -- leave the device alone
            return

        if wanted == self.aux_actual:
            self._aux_point()  # heartbeat: "still on" must keep being evidenced
            return
        try:
            await self.client.write_gatt_char(self.aux_char, b'1' if wanted else b'0', response=True)
            print(self.client.address, 'aux ->', 'ON' if wanted else 'off')
            # Do not assume it took: the device notifies its real state, and a read-back on the next
            # pass corrects us if the write was refused. Until then `state` is genuinely unknown and
            # the series says so by omitting it, rather than claiming the commanded value.
            self.aux_actual = None
            self.aux_actual_at = 0.0
            self._aux_point()
        except (BleakError, TimeoutError, EOFError):
            raise  # link death -- see the read path above
        except Exception as e:
            print(self.client.address, 'aux write failed:', type(e).__name__, e)

    async def update(self):
        await self._aux_reconcile()

        if time.time() - self.t_last_rx > RX_TIMEOUT:
            print(self.client.name, self.client.address, 'havent received anything for a while, re-connecting')
            await self._disconnect()
            raise TimeoutError("no data")

        if self.use_read:
            d = await self.client.read_gatt_char(self.char)
            if not d:
                if len(self.last_idx):
                    print('received empty frame after start')
                return
        else:
            if len(self.buf) == 0:
                return
            d = self.buf.copy()
            self.buf.clear()

        if self.frame_size is None:
            for candidate in (64, 56):
                if len(d) < candidate:
                    continue
                crc_off = candidate - 2
                crc = struct.unpack('H', d[crc_off:candidate])[0]
                if crc_modbus(d[:crc_off]) == crc:
                    self.frame_size = candidate
                    break
            if self.frame_size is None:
                self.buf.extend(d)
                return

        FS = self.frame_size
        leftover = len(d) % FS
        if leftover:
            self.buf.extend(d[-leftover:])
            d = d[:-leftover]
            if len(d) == 0:
                return
        # print('received bytes', len(d), 'num frames', len(d) // FS)
        for i in range(len(d) // FS):
            self._decode_frame(d[i * FS:(i + 1) * FS])

        if self.n_updates % 20 == 0:
            # supply voltage, rssi, mcu temp
            write_point(
                'smart_shunt_meta',
                dict(address=self.client.address,
                     name=self.client.name,
                     local_name=self.advertisement.local_name,
                     adcs=','.join(sorted(self.adc_device_names))),
                remove_nan(dict(
                    rssi=self.advertisement.rssi,
                    # vcc=math.nan,  # todo
                )),
                timestamp_ms=int(time.time() * 1000),
            )

        self.n_updates += 1

    async def _loop(self):
        try:
            while True:
                try:
                    await self.update()
                    await asyncio.sleep(.1)
                except KeyboardInterrupt:
                    break
                except (BleakError, TimeoutError, EOFError) as e:
                    # expected end-of-life for this client: link dropped, wedged or stalled.
                    # end the task quietly, main() will pop us and the scanner re-connects.
                    print(self.client.address, 'ending client:', type(e).__name__, e)
                    break
        except asyncio.CancelledError:
            raise
        except Exception:
            traceback.print_exc()
        finally:
            # Whatever is still held gets written with the best offset we ever reached. A link that
            # dropped before the estimate settled still measured something real, and the estimate
            # in hand is the one those points would have been stamped with anyway -- discarding
            # them would lose data to a clock detail, which is not a reason to lose data.
            held = len(self.points_buf)
            self._flush_points()
            if held:
                print(self.client.address, 'flushed %d held point(s) at shutdown' % held)
            if self.n_points_dropped or self.n_frames_no_time:
                print(self.client.address, 'dropped %d point(s), %d frame(s) carried no timestamp'
                      % (self.n_points_dropped, self.n_frames_no_time))
            await self._disconnect()
            print(self.client.address, 'loop task ends')

    async def start_task(self):
        if not self.use_read:
            await self.client.start_notify(self.char, self._notify_callback)
        if self.aux_char is not None:
            try:
                await self.client.start_notify(self.aux_char, self._aux_notify)
            except Exception as e:
                # Not fatal: _aux_reconcile falls back to reading the characteristic.
                print(self.client.address, 'aux subscribe failed:', type(e).__name__, e)
        self._loop_task = asyncio.create_task(self._loop())

    def is_good(self):
        return self._loop_task and not self._loop_task.done()

    def cancel(self):
        if self._loop_task:
            self._loop_task.cancel()


async def main():
    print('finding device..')

    clients: Dict[str, ClientState] = {}
    connecting: set = set()
    retry_after: Dict[str, float] = {}
    fail_count: Dict[str, int] = defaultdict(int)
    connect_tasks: set = set()

    # every address the scanner has ever shown us, so we can go looking for it again
    # without depending on the scanner producing another callback for it
    known: Dict[str, BLEDevice] = {}
    last_adv: Dict[str, AdvertisementData] = {}
    t_last_adv: Dict[str, float] = {}
    gone_since: Dict[str, float] = {}
    t_start = time.time()
    t_last_status = 0.0
    t_last_scan_restart = t_start

    # BlueZ wedge tracking: count consecutive connect failures with wedge signatures
    # across all devices. Reset on any successful connect. When it hits the threshold,
    # run btmgmt power off/on + re-apply conn params.
    bluez_consecutive_failures = 0
    t_last_bluez_recovery = 0.0
    t_last_probe = 0.0

    WEDGE_SIGNATURES = ('Operation already in progress', 'br-connection-canceled')

    def is_wedge_error(exc: Exception) -> bool:
        s = str(exc)
        return any(sig in s for sig in WEDGE_SIGNATURES)

    async def recover_bluez(scan=None, restart_bluetoothd=False):
        """Reset the BLE controller and HCI bridge when BlueZ wedges.

        hci0 is an ESP32-S3 USB HCI bridge (esp32-hci.service, btattach on /dev/ttyACM0).
        btmgmt power off/on alone toggles the Bluetooth stack but does not reset the
        ESP32 bridge, which can stay in a bad state. Restarting esp32-hci.service
        re-runs btattach and gives a clean controller. Requires passwordless sudo.
        """
        nonlocal t_last_bluez_recovery
        # Say which fault triggered this, not a fixed sentence: called from the probe
        # path the connect-failure count is zero, and reporting "0 consecutive connect
        # failures" would describe evidence that was never gathered.
        if restart_bluetoothd:
            print('*** Discovery fault: adapter saw no BLE devices at all — running recovery ***')
        else:
            print('*** BlueZ wedge detected: %d consecutive connect failures — running recovery ***'
                  % bluez_consecutive_failures)
        try:
            if scan is not None:
                try:
                    await scan.stop()
                except Exception:
                    pass
            # Restart bluetoothd when the fault is DISCOVERY rather than connecting.
            # btmgmt power-cycles the controller but leaves the daemon's discovery
            # state intact, and it is the daemon that goes deaf: measured 2026-08-14,
            # `bluetoothctl scan le` returned zero devices until bluetooth.service was
            # restarted, after which the identical scan saw 8. Not done on the
            # connect-failure path, where the controller is the thing that is stuck.
            if restart_bluetoothd:
                print('  restarting bluetooth.service (discovery fault)')
                proc = await asyncio.create_subprocess_exec(
                    'sudo', 'systemctl', 'restart', 'bluetooth.service',
                    stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                await proc.communicate()
                await asyncio.sleep(5)

            # Power off the controller first
            proc = await asyncio.create_subprocess_exec(
                'sudo', 'btmgmt', '--index', '0', 'power', 'off',
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            await proc.communicate()
            await asyncio.sleep(2)
            # Restart the HCI bridge service — this re-attaches the controller
            # and gives us a clean hci0, which btmgmt alone cannot do.
            proc = await asyncio.create_subprocess_exec(
                'sudo', 'systemctl', 'restart', HCI_BRIDGE_SERVICE,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            await proc.communicate()
            await asyncio.sleep(8)
            # Power on and re-apply conn params lost on reset
            proc = await asyncio.create_subprocess_exec(
                'sudo', 'btmgmt', '--index', '0', 'power', 'on',
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            await proc.communicate()
            await asyncio.sleep(3)
            for path, val in CONN_PARAM_FILES.items():
                full = '/sys/kernel/debug/bluetooth/hci0/' + path
                try:
                    proc = await asyncio.create_subprocess_exec(
                        'sudo', 'sh', '-c', f'echo {val} > {full}',
                        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                    await proc.communicate()
                except Exception as e:
                    print('  conn-param restore %s failed: %s' % (path, e))
            if scan is not None:
                try:
                    await asyncio.sleep(2)
                    await scan.start()
                except Exception as e:
                    print('  scanner restart after recovery failed: %s' % e)
            print('*** BlueZ recovery complete, resuming connect attempts ***')
        except Exception as e:
            print('*** BlueZ recovery FAILED: %s: %s — manual intervention needed ***'
                  % (type(e).__name__, e))
        t_last_bluez_recovery = time.time()

    def backoff(address: str):
        fail_count[address] += 1
        delay = min(RECONNECT_COOLDOWN * fail_count[address], RECONNECT_COOLDOWN_MAX)
        retry_after[address] = time.time() + delay
        print('retrying', address, 'in %.0fs' % delay)

    async def connect_device(address: str, target, advertisement: AdvertisementData):
        nonlocal bluez_consecutive_failures
        client = None
        try:
            print('connecting to', address, getattr(target, 'name', None),
                  advertisement.local_name if advertisement else None,
                  '(by address)' if isinstance(target, str) else '')
            client = bleak.BleakClient(target, timeout=20)
            if not client.is_connected:
                await client.connect(timeout=20)
            for s in client.services:
                for c in s.characteristics:
                    print('char', s.uuid, c.uuid, c.properties)
            char = client.services.get_characteristic(WIRE_SAMPLE_CHAR)
            if not char:
                print('char', WIRE_SAMPLE_CHAR, 'not found')
                backoff(address)
                return
            state = ClientState(client, char, advertisement)
            # Optional: older firmware has no aux characteristic, and that must not stop telemetry.
            state.aux_char = client.services.get_characteristic(AUX_CHAR)
            if state.aux_char is None:
                print(address, 'no aux characteristic (pre-aux firmware)')
            await state.start_task()
            clients[address] = state
            gone_since.pop(address, None)
            fail_count.pop(address, None)
            retry_after.pop(address, None)
            bluez_consecutive_failures = 0  # successful connect — BlueZ is healthy
        except asyncio.CancelledError:
            raise
        except Exception as e:
            # bleak raises asyncio.TimeoutError (not BleakError) when the connect times out,
            # and BleakError("device ... not found") when BlueZ dropped the stale device object
            print('error with', address, advertisement.local_name if advertisement else None,
                  ':', type(e).__name__, e)
            if is_wedge_error(e):
                bluez_consecutive_failures += 1
            backoff(address)
            if client is not None:
                try:
                    await asyncio.wait_for(client.disconnect(), timeout=DISCONNECT_TIMEOUT)
                except Exception:
                    pass
        finally:
            connecting.discard(address)

    def try_connect(address: str, target, advertisement: AdvertisementData):
        if address in clients or address in connecting:
            return
        if time.time() < retry_after.get(address, 0):
            return
        # connect in its own task so a 20s connect timeout doesn't stall the scanner callback
        connecting.add(address)
        task = asyncio.create_task(connect_device(address, target, advertisement))
        connect_tasks.add(task)
        task.add_done_callback(connect_tasks.discard)

    async def on_new_device(device: BLEDevice, advertisement: AdvertisementData):
        # record every sighting, connected or not: this is what tells the supervisor
        # apart "the device is gone" from "our scanner has gone deaf"
        known[device.address] = device
        last_adv[device.address] = advertisement
        t_last_adv[device.address] = time.time()

        if device.address in clients:
            # update advertisement data for rssi
            clients[device.address].advertisement = advertisement
            return

        try_connect(device.address, device, advertisement)

    scan = await start_scanner(on_new_device)

    try:
        while True:
            await asyncio.sleep(2)
            now = time.time()

            for client in list(clients.values()):
                address = client.client.address
                if client.is_good():
                    # the loop task should have ended itself via the RX_TIMEOUT in update();
                    # if it has not, it is stuck somewhere else and only cancellation frees us
                    if now - client.t_last_rx > STALL_HARD_TIMEOUT:
                        print(address, 'loop task still alive %.0fs after the last frame, cancelling'
                              % (now - client.t_last_rx))
                        client.cancel()
                    continue
                print('pop', address, 'client not good')
                clients.pop(address)
                gone_since.setdefault(address, now)
                # give BlueZ time to clean up the old link before we re-connect
                retry_after[address] = now + RECONNECT_COOLDOWN

            missing = [a for a in known if a not in clients]

            for address in missing:
                gone_since.setdefault(address, now)
                # while advertisements keep arriving, on_new_device drives the reconnect.
                # once they stop, that path is dead and this is the only way back.
                if now - t_last_adv.get(address, 0) > ADV_STALE:
                    try_connect(address, address, last_adv[address])

            # BlueZ discovery can go deaf while the process stays up. Restarting the
            # scanner is cheap; not restarting it costs every device at once.
            stale = [a for a in missing if now - t_last_adv.get(a, 0) > SCANNER_RESTART_AFTER]
            if stale and now - t_last_scan_restart > SCANNER_RESTART_AFTER:
                t_last_scan_restart = now
                print('no advertisement from', ','.join(stale), 'for %.0fs, restarting the scanner'
                      % SCANNER_RESTART_AFTER)
                try:
                    await scan.stop()
                    await asyncio.sleep(.5)
                    await scan.start()
                except Exception as e:
                    print('scanner restart failed:', type(e).__name__, e)

            # BlueZ wedge recovery: when every connect fails with controller-level errors,
            # per-device backoff is pointless — the controller itself is stuck. Power-cycle
            # it via btmgmt and re-apply conn params, then let the normal retry loop resume.
            if (bluez_consecutive_failures >= BLUEZ_WEDGE_THRESHOLD
                    and not clients  # no device is connected -- globally stuck, not one flaky device
                    and now - t_last_bluez_recovery > BLUEZ_WEDGE_COOLDOWN):
                await recover_bluez(scan)
                bluez_consecutive_failures = 0
                # reset per-device backoff so all devices retry immediately after recovery
                for a in list(fail_count.keys()):
                    fail_count[a] = 0
                    retry_after[a] = time.time() + 2

            # Total silence: nothing from ANY smart-shunt for a long time. That is the
            # ambiguous state -- the filtered scanner cannot tell an empty bench from a
            # deaf adapter -- so go and settle it instead of printing "still scanning"
            # forever. See SCANNER_PROBE_AFTER.
            last_any_adv = max(t_last_adv.values(), default=t_start)
            if (not clients
                    and now - last_any_adv > SCANNER_PROBE_AFTER
                    and now - t_last_probe > SCANNER_PROBE_COOLDOWN):
                t_last_probe = now
                print('nothing heard for %.0fs — probing the adapter itself' % (now - last_any_adv))
                try:
                    await scan.stop()
                except Exception:
                    pass
                n_seen = await probe_adapter()
                try:
                    await scan.start()
                except Exception as e:
                    print('  scanner restart after probe failed: %s: %s' % (type(e).__name__, e))

                # -1 and None are deliberately distinct in the series: "probe says deaf"
                # and "probe could not run" are different facts and must not average
                # together into a number that looks like a measurement.
                write_point(
                    'smart_shunt_meta',
                    dict(address='adapter', name='scanner'),
                    dict(scanner_devices_seen=(-1 if n_seen is None else n_seen),
                         scanner_ok=(None if n_seen is None else int(n_seen > 0))),
                    timestamp_ms=int(now * 1000),
                )

                if n_seen is None:
                    print('  probe INCONCLUSIVE — cannot tell whether the adapter scans; '
                          'not resetting anything on the strength of a failed measurement')
                elif n_seen == 0:
                    print('*** SCANNER FAULT: adapter saw ZERO BLE devices in %.0fs of '
                          'unfiltered scanning. This is the adapter, NOT the peripherals. ***'
                          % SCANNER_PROBE_SECONDS)
                    if now - t_last_bluez_recovery > BLUEZ_WEDGE_COOLDOWN:
                        await recover_bluez(scan, restart_bluetoothd=True)
                else:
                    print('  adapter is healthy (%d devices seen) — the smart-shunt '
                          'peripherals are genuinely not advertising' % n_seen)

            # say out loud that we are waiting. without this the process goes silent on a
            # dead peripheral and looks identical to a wedged client.
            if not known and now - t_last_status > STATUS_INTERVAL:
                t_last_status = now
                print('no smart-shunt seen yet, scanning for %.0fs' % (now - t_start))

            if missing and now - t_last_status > STATUS_INTERVAL:
                t_last_status = now
                for address in missing:
                    gone = now - gone_since.get(address, now)
                    adv_age = now - t_last_adv.get(address, 0)
                    print('waiting for %s (%s): no data for %.0fs, last advertisement %.0fs ago'
                          % (address, (known[address].name or '?'), gone, adv_age))
                    write_point(
                        'smart_shunt_meta',
                        dict(address=address, name=known[address].name),
                        dict(connected=0, gone_s=round(gone, 1), adv_age_s=round(adv_age, 1)),
                        timestamp_ms=int(now * 1000),
                    )
    except KeyboardInterrupt:
        pass
    finally:
        for client in list(clients.values()):
            try:
                await asyncio.wait_for(client.client.disconnect(), timeout=DISCONNECT_TIMEOUT)
            except:
                pass


# Guarded so the module can be imported (the aux control-file logic has tests); running it as a
# script is unchanged.
if __name__ == "__main__":
    asyncio.run(main())
