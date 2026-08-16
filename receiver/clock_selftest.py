#!/usr/bin/env python3
"""Regression test for the receiver's device-clock handling.

WireSample.t is gettimeofday() milliseconds on the device (firmware: adc/sampling.h). It arrives as
either real Unix ms (the board reached timeSync()) or ms-since-boot (BLE-only board, SNTP never
ran), and nothing in the frame says which. Everything here is about getting a sample onto the host
timeline without inventing a time for it.

Cases 2, 3, 4, 7 and 8 are failures that were REAL in the receiver up to 2026-08-16, each listed
with what it actually did, so this file doubles as the record of why the code looks the way it
does. Every one of them was watched failing against the previous revision before the fix landed --
a guard nobody has seen fire is not a guard -- and it can be watched again:

    python3 receiver/clock_selftest.py
    SMART_SHUNT_CLIENT=/path/to/older-client.py python3 receiver/clock_selftest.py

No BLE, no InfluxDB, no network: frames are synthesised with real CRCs and write_point is captured.
"""
import collections
import contextlib
import importlib.util
import io
import os
import struct
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
CLIENT = os.environ.get('SMART_SHUNT_CLIENT', os.path.join(HERE, 'smart-shunt-ble-client.py'))

written = []


def _load():
    """Import the client with its BLE stack stubbed out and write_point captured."""
    for name in ('bluek', 'bluek.shadow', 'bleak'):
        sys.modules.setdefault(name, types.ModuleType(name))
    for attr in ('AdvertisementData', 'BLEDevice', 'BleakGATTCharacteristic',
                 'BleakClient', 'BleakScanner'):
        setattr(sys.modules['bleak'], attr, object)
    sys.modules['bleak'].BleakError = type('BleakError', (Exception,), {})

    sys.path.insert(0, HERE)
    import util
    real_write, util.write_point = util.write_point, \
        lambda m, tags, values, timestamp_ms: written.append(timestamp_ms)
    del real_write  # never send UDP from a test

    spec = importlib.util.spec_from_file_location('smart_shunt_client', CLIENT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, util


MOD, UTIL = _load()

BOOT = 1_770_000_000_000  # host wall-clock at the instant the device booted
DEV = b'ftr_t17_48'


def frame(idx, t):
    """A CRC-valid 64-byte WireSample, laid out as the C struct is (note the alignment pad)."""
    body = struct.pack('BB16s', 1, idx, DEV.ljust(16, b'\0'))
    body += struct.pack('ffffQf', 12.0, 1.5, 18.0, 3.0, t, 25.0)  # u i p e t temp
    body += b'\0' * 4                                             # struct alignment
    body += struct.pack('ff', 1.6, 12.1)                          # i_max u_max
    body += struct.pack('I', 0)                                   # diag
    assert len(body) == 62, len(body)
    return bytearray(body + struct.pack('H', UTIL.crc_modbus(body)))


BUF_MAX = getattr(MOD, 'CLOCK_BUF_MAX', 4096)


def _init_clock_state(st):
    """Seed the clock state ClientState.__init__ would have set.

    Sets BOTH the current field names and the ones they replaced, so this file can also be pointed
    at an older client (SMART_SHUNT_CLIENT=...) and the failures below watched happening. A test
    that can only run against the fixed code cannot show that it fixed anything.
    """
    st.points_buf = collections.deque(maxlen=BUF_MAX)
    st.clock_mode = None
    st.clock_offset_ms = None
    st.clock_settled = 0
    st.n_frames_no_time = 0
    st.n_points_dropped = 0
    st.remote_boot_time = 0          # pre-2026-08-16 field names
    st.remote_boot_time_const_cnt = 0


def offset_of(st):
    """The estimated boot instant, under whichever field name this revision uses."""
    if getattr(st, 'clock_mode', None) is not None:
        return st.clock_offset_ms
    return st.remote_boot_time or None


def feed(frames):
    """Run frames through the real _decode_frame. Returns (stamps, state, exception-or-None, log)."""
    written.clear()
    clock = {'now': 0.0}
    MOD.time = types.SimpleNamespace(time=lambda: clock['now'] / 1000.0)

    st = MOD.ClientState.__new__(MOD.ClientState)
    st.client = types.SimpleNamespace(address='TEST')
    st.advertisement = types.SimpleNamespace(rssi=-50, tx_power=0)
    st.last_idx = collections.defaultdict(lambda: -1)
    st.adc_device_names = set()
    st.t_last_rx = 0
    _init_clock_state(st)

    err = None
    log = io.StringIO()
    with contextlib.redirect_stdout(log):
        for i, (t, now) in enumerate(frames):
            clock['now'] = now
            try:
                st._decode_frame(frame(i & 0xFF, t))
            except Exception as e:                       # noqa: BLE001 -- the test IS the handler
                err = '%s: %s' % (type(e).__name__, e)
                break
    return list(written), st, err, log.getvalue()


def check(name, cond, detail=''):
    print('  %-4s %s%s' % ('ok' if cond else 'FAIL', name, (' -- ' + detail) if detail else ''))
    if not cond:
        check.failed += 1


check.failed = 0


# --------------------------------------------------------------------------------------------
print('\n1. unsynced device (ms since boot), jittery link latency')
# The estimator takes min(host_now - t): every observation is the true offset plus a non-negative
# transport delay, so the smallest is the least contaminated and can only improve.
lat = [40, 300, 12, 90, 55, 8, 33, 61, 20, 44, 77, 15]
ups = [1000 + k * 125 for k in range(12)]
stamps, st, err, _ = feed([(u, BOOT + u + l) for u, l in zip(ups, lat)])
check('no exception', err is None, err or '')
check('every point written', len(stamps) == len(ups), '%d of %d' % (len(stamps), len(ups)))
check('offset converges on the minimum latency', offset_of(st) == BOOT + min(lat))
check('no stamp off true time by more than the best latency',
      all(0 <= s - (BOOT + u) <= min(lat) for s, u in zip(stamps, ups)))

# --------------------------------------------------------------------------------------------
print('\n2. TIME-SYNCED device sends real epoch ms')
# WAS: `if t < 1e10` skipped the whole estimator, so the flush gate never opened. Not one point was
# ever written and points_buf grew for the life of the process. Measured on the old code: 40 frames
# in, 0 written, 40 buffered.
frames = [(BOOT + k * 1000, BOOT + k * 1000 + 30) for k in range(40)]
stamps, st, err, _ = feed(frames)
check('no exception', err is None, err or '')
check('every point written', len(stamps) == 40, '%d written, %d stuck in the buffer'
      % (len(stamps), len(st.points_buf)))
check('device stamp used as-is, no offset added', stamps == [f[0] for f in frames])
check('buffer drained', not st.points_buf)

# --------------------------------------------------------------------------------------------
print('\n3. device time-syncs partway through a live link')
# WAS: UnboundLocalError -- the drift check read `now`, which is assigned only on the uptime path.
# It tore the link down on every frame after the switch. Had it survived, the flush would have
# added the boot offset to an epoch timestamp and filed the points under the year 2082.
frames = [(1000 + k * 125, BOOT + 1000 + k * 125 + 20) for k in range(10)]
frames += [(BOOT + 2500 + k * 1000, BOOT + 2500 + k * 1000 + 20) for k in range(6)]
stamps, st, err, log = feed(frames)
check('no exception', err is None, err or '')
check('the switch is announced', 'clock switched uptime -> epoch' in log)
check('post-switch stamps are the device stamps', stamps[-6:] == [f[0] for f in frames[-6:]])
check('nothing lands in the far future', max(stamps) < BOOT + 3_600_000,
      'max stamp %d' % max(stamps))

# --------------------------------------------------------------------------------------------
print('\n4. host clock steps forward (rpi with no RTC, chrony first sync)')
# WAS: the estimator only ever adopted a LOWER observation, so a forward step was rejected as
# though it were latency and every later point was recorded an hour early -- silently, for the life
# of the link. Measured on the old code: last stamp 3600 s wrong.
STEP = 3_600_000
frames = [(1000 + k * 125, BOOT + 1000 + k * 125 + 20) for k in range(10)]
frames += [(1000 + k * 125, BOOT + STEP + 1000 + k * 125 + 20) for k in range(10, 30)]
stamps, st, err, log = feed(frames)
true_last = BOOT + STEP + 1000 + 29 * 125
check('no exception', err is None, err or '')
check('re-anchor is announced', 're-anchored' in log)
check('last stamp within 1s of true time', abs(stamps[-1] - true_last) < 1000,
      '%+.0f s off' % ((stamps[-1] - true_last) / 1000))

# --------------------------------------------------------------------------------------------
print('\n5. latency jitter below the threshold must NOT move the offset')
# The other side of case 4: chasing small excesses would ratchet the estimate upward frame by frame
# and turn a minimum-estimator into a random walk.
frames = [(1000 + k * 125, BOOT + 1000 + k * 125 + 5) for k in range(10)]
stalled = getattr(MOD, 'CLOCK_REANCHOR_MS', 30_000) - 10_000
frames += [(1000 + k * 125, BOOT + 1000 + k * 125 + stalled) for k in range(10, 60)]
stamps, st, err, log = feed(frames)
check('no exception', err is None, err or '')
check('offset unmoved by 50 late frames', offset_of(st) == BOOT + 5,
      '%+d ms' % (offset_of(st) - BOOT - 5))
check('no spurious re-anchor', 're-anchored' not in log)

# --------------------------------------------------------------------------------------------
print('\n6. a frame carrying no timestamp is dropped, not stamped with the time of arrival')
# Reception time is a different quantity from measurement time, and nothing downstream could tell
# them apart afterwards. Absence of a timestamp must stay absent.
stamps, st, err, _ = feed([(0, BOOT)])
check('no exception', err is None, err or '')
check('nothing written', not stamps)
check('counted as dropped', st.n_frames_no_time == 1 and st.n_points_dropped == 1)

# --------------------------------------------------------------------------------------------
print('\n7. an offset that never settles must not grow the buffer without bound')
# WAS: collections.deque() with no maxlen. Combined with case 2, that was an unbounded leak.
n = BUF_MAX + 500
stamps, st, err, _ = feed([(1000 + k, BOOT + 1000) for k in range(n)])  # every frame a new minimum
check('no exception', err is None, err or '')
check('buffer capped', len(st.points_buf) <= BUF_MAX,
      '%d held' % len(st.points_buf))
check('the discards are counted, not silent', st.n_points_dropped >= n - BUF_MAX)

# --------------------------------------------------------------------------------------------
print('\n8. idx wraps at 255 without reporting a gap that did not happen')
frames = [(1000 + k * 125, BOOT + 1000 + k * 125 + 20) for k in range(258)]
stamps, st, err, log = feed(frames)
check('no exception', err is None, err or '')
check('no missed-sample report over 258 contiguous frames', 'missed sample' not in log)

# --------------------------------------------------------------------------------------------
print('\n9. a real gap IS still reported')
# Case 8 must not have been bought by silencing the detector.
frames = [(1000 + k * 125, BOOT + 1000 + k * 125 + 20) for k in range(10)]
written.clear()
clock = {'now': BOOT}
MOD.time = types.SimpleNamespace(time=lambda: clock['now'] / 1000.0)
st = MOD.ClientState.__new__(MOD.ClientState)
st.client = types.SimpleNamespace(address='TEST')
st.advertisement = types.SimpleNamespace(rssi=-50, tx_power=0)
st.last_idx = collections.defaultdict(lambda: -1)
st.adc_device_names = set()
st.t_last_rx = 0
_init_clock_state(st)
log = io.StringIO()
with contextlib.redirect_stdout(log):
    for k, idx in enumerate([0, 1, 2, 7, 8]):  # 3..6 lost
        clock['now'] = BOOT + 1000 + k * 125 + 20
        st._decode_frame(frame(idx, 1000 + k * 125))
check('a dropped run of samples is reported', 'missed sample' in log.getvalue())

print('\n%s' % ('ALL PASS' if not check.failed else '%d CHECK(S) FAILED' % check.failed))
sys.exit(1 if check.failed else 0)
