"""

run `pyvisa-info`

brew install libusb
pip install pyusb pyvisa pyvisa-py

"On Unix system, one may have to modify udev rules to allow non-root access to the device you are trying to connect to. The following tutorial describes how to do it http://ask.xmodulo.com/change-usb-device-permission-linux.html."


macos:
python 32bit needed
https://stackoverflow.com/questions/5541096/ni-visa-pyvisa-on-mac-os-x-snow-leopard

not so

"""
import collections
import socket
import sys
import time

import pyvisa as visa
from DMM6500 import DMM6500
from DMM6500_SCPI import Function

from util import round_to_n_dec

resource_name = 'USB::0x05e6::0x6500::04577308::INSTR'
print(resource_name)

rm = visa.ResourceManager()
rsc = rm.open_resource(resource_name)
mm = DMM6500(rsc)

# nplc 12 is adc max integration time
# mm.nplc = 12
# TODO doesnt work

# mm.reset()
# mm.function = Function.DC_CURRENT  # ioreg -p IOUSB

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

avg = 1

# csr = 2e-3
csr = 0

if len(sys.argv) > 1:
    csr = 2.5e-3
    print('measuring current with current sense resistor = %.1f mΩ' % (csr * 1e3))

if csr:
    voltage = False
    print('measuring voltage across shunt R= %.3s mΩ (%.1fA/50mV)' % (csr * 1e3, 50e-3 / csr))
elif True:
    voltage = True
    print('measuring voltage')
else:
    voltage = False
    print('measuring current')

print('averaging = ', avg)


def write_point(measurement, tags, values, timestamp_ms):
    lp = "smart_shunt"
    for k, v in tags.items():
        lp += ',%s=%s' % (k, v)
    for k, v in values.items():
        lp += ' %s=%s' % (k, v)
    # print(lp)
    lp += ' ' + str(int(round(timestamp_ms)))
    sock.sendto(lp.encode(), ('127.0.0.1', 8086))


t_last_read = time.time()


def watchdog_loop():
    while time.time() - t_last_read < 5:
        time.sleep(1)
    print('Watchdog timeout, exit')
    # rsc.close()
    rm.close()
    sys.exit(1)


from threading import Thread

Thread(target=watchdog_loop, daemon=True).start()

acc = 0
n = 0

t0 = time.time()
n_tot = 0

win100 = collections.deque(maxlen=100)

while True:
    v = mm.measure()
    if v > 9e20:
        v = float('nan')

    if csr:
        v = v / csr  # I=U/R

    t_last_read = time.time()

    acc += v
    win100.append(v)

    n += 1
    n_tot += 1

    if n == avg:
        v_avg = acc / n
        n = 0
        acc = 0.

        v_sma = sum(win100) / len(win100) if win100 else float('nan')

        sps = n_tot / (time.time() - t0)

        sys.stdout.write('  %s=  %-10s (sma100=%-10s)  sps=%.1f        \r' % (
            'V' if voltage else 'A',
            round_to_n_dec(v_avg, 7), round_to_n_dec(v_sma, 7), sps))

        t = time.time() * 1000
        write_point('smart_shunt',
                    tags=dict(device='DMM6500'),
                    values=dict(I=v_avg) if not voltage else dict(U=v_avg),
                    timestamp_ms=t)

    time.sleep(.001)

    """
    mA
    DMM,INA
    0.65,-0.03
    268.8,266.5
    975.2,972.66
    1898,1895
    4342.2,4338.2,
    6.3980 A,6.3943 A
    7.5815 A,7.5786 A	
    9.7906 A,9.7904 A	
    
    """
