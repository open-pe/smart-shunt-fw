import math
import socket


def round_to_n(x, n):
    # if isinstance(x, tuple):
    #    return '[%s]' % ', '.join(map(str, map(partial(round_to_n, n=n), x)))

    if not x or isinstance(x, str) or not math.isfinite(x):
        return x

    try:
        f = round(x, -int(math.floor(math.log10(abs(x)))) + (n - 1))
        if isinstance(f, float) and f.is_integer():
            return int(f)
        return f
    except ValueError as e:
        print('error', x, n, e)
        raise e


def num2str(x, n=None, strip_zeros=True):
    if n is not None:
        x = round_to_n(x, n)
    if strip_zeros or n is None:
        s = str(x)
        if s.endswith('.0'):
            s = s[:-2]
    else:
        s = str(x)  # TODO
    return s


def round_to_n_dec(x, n):
    x = round_to_n(x, n)

    if not x or isinstance(x, str) or not math.isfinite(x):
        return str(x)

    ax = abs(x)

    if ax < 1e-20:
        return '~0'
    elif ax < 999e-12:
        return num2str(x * 1e12, n) + 'p'
    elif ax < 999e-9:
        return num2str(x * 1e9, n) + 'n'
    elif ax < 999e-6:
        return num2str(x * 1e6, n) + 'µ'
    elif ax < 999e-3:
        return num2str(x * 1e3, n) + 'm'
    elif ax > 0.999e6:
        return num2str(x * 1e-6, n) + 'M'
    elif ax > 0.999e3:
        return num2str(x * 1e-3, n) + 'k'
    # elif x > 9
    else:
        return num2str(x, n)


sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP


def write_point(measurement, tags, values, timestamp_ms):
    lp = measurement
    for k, v in tags.items():
        lp += ',%s=%s' % (k, v)
    for k, v in values.items():
        lp += ' %s=%s' % (k, v)
    # print(lp)
    lp += ' ' + str(int(round(timestamp_ms))) + '\n'
    sock.sendto(lp.encode(), ('127.0.0.1', 8086))
