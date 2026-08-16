"""The three helpers smart-shunt-ble-client.py imports, and nothing else.

Extracted VERBATIM from pwr-metering/util.py, which is a 244-line grab-bag pulling in pandas,
pytz and dateutil for query and calibration helpers the receiver never touches. Copying the whole
thing would have made this directory unrunnable without that dependency tree, for no gain.

Keep the function bodies byte-identical to their originals. The import line in the client
(`from util import write_point, crc_modbus, remove_nan`) is deliberately the same in both repos so
the two copies of the client diff cleanly -- see README.md.
"""
import math
import os
import socket

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

# The rpi runs the collector against its own local influxd; anything else on the bench reaches it
# by name. Override for a different host.
influxdb_host = os.environ.get(
    'INFLUXDB_HOST', '127.0.0.1' if socket.gethostname() == 'rpi' else 'rpi.local')


def write_point(measurement, tags, values, timestamp_ms):
    # https://docs.influxdata.com/influxdb/v1/write_protocols/line_protocol_reference/
    lp = measurement
    for k, v in tags.items():
        if v is None:
            continue
        assert '\n' not in v
        lp += ',%s=%s' % (k, str(v).replace(',', '\\,').replace('=', '\\=').replace(' ', '\\ '))
    lp += ' '
    for k, v in values.items():
        lp += '%s=%s,' % (k, v)
    lp = lp[:-1]
    if timestamp_ms is not None:
        lp += ' ' + str(int(round(timestamp_ms)))
    lp += '\n'
    sock.sendto(lp.encode(), (influxdb_host, 8086))


def crc_modbus(data: bytearray) -> int:
    """Calculate CRC-16-CCITT MODBUS."""
    crc: int = 0xFFFF
    for i in data:
        crc ^= i & 0xFF
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc % 2 else (crc >> 1)
    return crc & 0xFFFF


def remove_nan(fields, int_to_float=False, rm_inf=False):
    for k, v in dict(fields).items():
        if v is None:
            del fields[k]
        elif isinstance(v, float):
            if math.isnan(v) or (rm_inf and not math.isfinite(v)):
                del fields[k]
        elif int_to_float and isinstance(v, int):
            fields[k] = float(v)
        elif isinstance(v, str):
            if not v:
                del fields[k]
    return fields
