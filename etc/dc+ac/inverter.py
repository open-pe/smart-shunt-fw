"""

ina228:
the shortest conversion times are 50, 84 and 150µs (up to 4120µs).
at 50µs, noise Vpp is 50µV, which is 0.125% at 40mV FSR.
for power monitoring, we need I & U, so 100µs, which equals to 10 kHz SR

"""
import math

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

from notch import NotchIIR

sr = 1 / (2 * 84e-6)  # 2*50µs
ac_freq = 50
n = 2000

print('total time', round(n / sr, 3))

x = 11 + abs(np.sin(2 * math.pi * ac_freq * np.array(range(0, n)) / sr) * 8)

roll_win = int(round(sr / ac_freq))
r = pd.Series(x).rolling(roll_win).mean()

notch = NotchIIR(sr, ac_freq)

n = pd.Series(notch(x))

plt.plot(x, marker='.', label=f'mean={round(np.mean(x), 6)}')
r.plot(label=f'roll({roll_win}) mean={round(r.mean(), 6)}')
# n.plot(label=f'notch mean={round(n.mean(), 6)}')


plt.legend()
plt.show()
