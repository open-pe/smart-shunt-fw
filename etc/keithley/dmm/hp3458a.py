"""
https://www.keysight.com/de/de/assets/9018-14517/quick-start-guides/9018-14517.pdf
https://github.com/dbbotkin/HP3245a_Calibration/blob/main/HP%203245A%20Command%20List.pdf
https://github.com/dbbotkin/HP3245a_Calibration/blob/main/HP3245_Calibrate.py
https://res.cloudinary.com/iwh/image/upload/q_auto,g_center/assets/1/7/Keysight_3458A_calibration_manual.pdf#page=91

U180 drop-in:
https://www.eevblog.com/forum/metrology/nu180-a-u180-drop-in-project-for-the-3468a-dvm/

SN18A:
http://literature.cdn.keysight.com/litweb/pdf/3458A-18A.pdf
https://web.archive.org/web/20190409075350/http://literature.cdn.keysight.com/litweb/pdf/3458A-18A.pdf
If the absolute value of C is > 0.43 ppm per day, then the A3 assembly in your instrument needs to be
replaced.  (C=cal72)

data:
https://www.eevblog.com/forum/metrology/3458a-worklog/
marcoreps/multiinstrumentalist


ACAL:
* run if temperature changed 1°C or every 24h
* wait at least 2h warm-up before ACAL
* wait 15min after ACAL DCV for relais thermal stabilisation
"""

import math
import sys
import time
from decimal import Decimal

import timeout_decorator
from pyvisa.resources import MessageBasedResource
from util import write_point, round_to_n_dec


class HP3458A:

    def __init__(self, name='HP3458A'):
        self.name = name
        self.inst: MessageBasedResource | None = None

    def connect(self):
        import pyvisa

        rm = pyvisa.ResourceManager()
        # res = ('USB0::1003::8293::GPIB_15_34235303432351F0D141::0::INSTR')
        res = ('USB0::1003::8293::HP3458A::0::INSTR')
        print('opening', res)
        inst: MessageBasedResource = rm.open_resource(res)
        inst.timeout = 2000000
        self.inst = inst

        self.wait_for_readiness()

        # inst.write("RESET")
        inst.write("END ALWAYS")
        inst.write("OFORMAT ASCII")

        dev_id = inst.query("ID?").strip()
        assert dev_id == "HP3458A", "its not a HP3458A or adapter buffer is lagging (restart)"

        #print('idn?' , inst.query("IDN?"))
        print('rev?' , inst.query("REV?"))

        # print ('IDN', inst.query('END ON;IDN?'))
        # print ('calnum', inst.query('CALNUM?'))
        # print ('calstr', inst.query('CALSTR?'))
        # print ('callen', inst.query('CALEN?'))

    def wait_for_readiness(self):
        print('waiting for readiness ..')
        while True:
            time.sleep(1)
            try:
                self.inst.query("").strip()
                break
            except Exception as e:
                if 'Operation timed out' in str(e):
                    continue
                raise

    def acal_dcv(self, what='DCV', wait=False):
        assert what in ('DCV', 'ACV', 'DCI', 'ACI', 'ALL')
        self.inst.query("").strip()
        print('starting ACAL', what)
        self.inst.write("ACAL " + what.upper())
        t0 = time.time()
        if wait:
            self.wait_for_readiness()
            print('acal dcv took', round(time.time() - t0, 1), 'seconds')

    def read_calibration(self, dtype=Decimal):
        """
        https://res.cloudinary.com/iwh/image/upload/q_auto,g_center/assets/1/7/Keysight_3458A_calibration_manual.pdf#page=91

         CAL? 59 for DCV and CAL? 60 for OHMS.

         The internal temperature of the 3458A under test must be within
5 degrees C of its temperature when last adjusted (CAL 0, CAL 10, and CAL
10K). These temperatures can be determined by executing the commands
CAL? 58, CAL? 59, CAL? 60.
        :param dtype:
        :return:
        """
        def _conv(val):
            try:
                return dtype(val)
            except Exception as e:
                print('input val was', repr(val))
                raise e

        inst = self.inst
        print('reading calibration')
        d = {n: _conv(self.inst.query("CAL? " + str(n)).strip()) for n in range(1, 254)}
        return d

    def read_error(self):
        s = self.inst.query("ERRSTR?").strip()
        print('errstr?', s)
        err_code = int(s[:s.index(',')])
        err_msg = s[s.index(',') + 1:].strip('"')
        return err_code, err_msg
    def auto_zero(self, on=True):
        self.inst.write("AZERO ON")

    def nplc(self, nplc):
        """
        To specify the most accuracy, highest resolution, and 80 dB of NMR for DC or
        ohms measurements (with the slowest measurement speed), send:
        OUTPUT 722;"NPLC 1000"
        """
        assert isinstance(nplc, int) and nplc >= 1
        self.inst.write("NPLC " + str(nplc))

    def write(self, s):
        self.inst.write(s)

    def temperature(self):
        return float(self.inst.query("TEMP?"))

    def read(self):
        @timeout_decorator.timeout(nplc / 50 * 2 + 5, use_signals=True)
        def read_timeout():
            # inst.write("TARM SGL,1")
            try:
                return float(inst.query("").strip())
            except ValueError as e:
                print(e)
                print('\n\n')
                return math.nan

        raise NotImplementedError()


def main():
    inst = HP3458A(sys.argv[1])
    inst.write("NDIG 9")
    inst.write("TARM AUTO")

    inst.auto_zero(True)
    inst.nplc(10)

    print('start reading in 1s..')
    time.sleep(1)

    def read_err():
        s = inst.query("ERRSTR?").strip()
        print('errstr?', s)
        err_code = int(s[:s.index(',')])
        err_msg = s[s.index(',') + 1:].strip('"')
        return err_code, err_msg

    while True:
        err_code, err_msg = read_err()
        if err_code:
            print('error register:', err_code, err_msg)
        else:
            assert err_msg == "NO ERROR", err_msg
            break

    read_timeout()
    err_code, err_msg = read_err()
    if err_code:
        raise ValueError(str(err_code) + ': ' + err_msg)

    while True:
        val = read_timeout()

        t = time.time() * 1000

        sys.stdout.write((' read   %12sV' % round_to_n_dec(val, 3 if abs(val) < 999e-9 else 7)) + '        \r')

        write_point('smart_shunt',
                    tags=dict(device=dev_id),
                    values=dict(I=val) if not voltage else dict(U=val),
                    timestamp_ms=t)

        # time.sleep(1)

    # dm = HP3458A()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
