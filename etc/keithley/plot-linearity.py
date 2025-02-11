from matplotlib import pyplot as plt

# 0.65,-0.03
measurements = """
268.8,266.5
975.2,972.66
1898,1895
4342.2,4338.2
6398.0,6394.3
7581.5,7578.6
9790.6,9790.4	
"""



d = list(map(lambda l: list(map(float, l.split(','))), measurements.strip().split("\n")))
print(d)

import pandas as pd

df = pd.DataFrame(d).set_index(0)
df.loc[:,1] = df.loc[:,1] / df.index
print(df)
df.plot(marker='.')
plt.show()