import time

from dmm.hp3458a import HP3458A
from util import write_point


def rel_err(a, ref):
    return (a - ref) / abs(ref)


m = HP3458A()
m.connect()

print('err', m.read_error())

last_calib = m.read_calibration()
while True:
    m.acal_dcv(wait=True)
    calib = m.read_calibration()

    keys = list(calib.keys())
    cs = 20  # chunk size (for UDP transport)
    t = time.time() * 1000
    for ks in [keys[i:i + cs] for i in range(0, len(keys), cs)]:
        print('storing', ks)
        write_point('calib',
                    tags=dict(typ='dmm', device=m.name),
                    values={'cal' + str(k): calib[k] for k in ks},
                    timestamp_ms=t
                    )
        time.sleep(.1)
    print('stored calibration values in influxdb')

    for n in range(1, 254):
        if calib[n] != last_calib[n]:
            print("Param %3d changed : %f -> %f (%.2f ppm)" % (n, last_calib[n], calib[n],
                                                               float(rel_err(calib[n], last_calib[n])) * 1e6))

    last_calib = calib
    time.sleep(3600 * 2)
