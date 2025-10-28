import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("https://raw.githubusercontent.com/marcoreps/3458a/refs/heads/main/csv/HP%203458A%20internal%20temp%20vs%20CAL%2072.csv")
df.index = pd.to_datetime(df.Time, unit='ms')
cal72 = df['CAL? 72 3458A']
cal72 = cal72[cal72 != 'undefined'].astype(float)
cal72.plot()


dy = cal72.iloc[-1] - cal72.iloc[0]
dt = cal72.index[-1] - cal72.index[0]

print(dy, dt) # 0.05ppm/day


plt.show()

