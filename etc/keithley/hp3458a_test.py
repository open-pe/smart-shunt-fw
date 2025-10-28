"""
# https://www.keysight.com/de/de/assets/9018-14517/quick-start-guides/9018-14517.pdf
https://github.com/dbbotkin/HP3245a_Calibration/blob/main/HP%203245A%20Command%20List.pdf
https://github.com/dbbotkin/HP3245a_Calibration/blob/main/HP3245_Calibrate.py
"""

import math
import sys
import time

import timeout_decorator
from pyvisa.resources import MessageBasedResource

from util import write_point, round_to_n_dec


def main():
    import pyvisa

    rm = pyvisa.ResourceManager()
    # res = ('USB0::1003::8293::GPIB_15_34235303432351F0D141::0::INSTR')
    res = ('USB0::1003::8293::HP3458A::0::INSTR')
    print('opening', res)
    inst: MessageBasedResource = rm.open_resource(res)
    inst.timeout = 2000000
    # inst.write("RESET")
    inst.write("END ALWAYS")
    inst.write("OFORMAT ASCII")
    # inst.write("BEEP")

    voltage = True

    if voltage:
        pass
        # inst.write("DCV")
    else:
        raise ValueError()

    dev_id = inst.query("ID?").strip()
    assert dev_id == "HP3458A", "its not a HP3458A or adapter buffer is lagging (restart)"

    if False:
        print('performing ACAL..')
        inst.write("ACAL DCV")
        time.sleep(30)
        inst.query("")

    inst.write("NDIG 9")
    inst.write("TARM AUTO")
    inst.write("AZERO ON")
    print('ndig 9, tarm auto')

    """
    To specify the most accuracy, highest resolution, and 80 dB of NMR for DC or
    ohms measurements (with the slowest measurement speed), send:
    OUTPUT 722;"NPLC 1000"
    """
    nplc = 10  # 200
    inst.write("NPLC " + str(nplc))
    print('nplc <-', nplc)

    # print('ser:', inst.query("SER?"))

    # print("temp", inst.query("TEMP?"))

    @timeout_decorator.timeout(nplc / 50 * 2 + 5, use_signals=True)
    def read_timeout():
        # inst.write("TARM SGL,1")
        try:
            return float(inst.query("").strip())
        except ValueError as e:
            print(e)
            print('\n\n')
            return math.nan

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
