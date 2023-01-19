import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('vel.csv')

plt.rcParams["figure.autolayout"] = True
fig, (ax1) = plt.subplots(1, 1)
ax1.plot(df.Time, df.Velocity, label="Velocity")
ax1.plot(df.Time, df.Voltage, label="Voltage")
ax1.plot(df.Time, df.Error, label="Error")
ax1.set_xlabel('Time/s')
ax1.legend()
plt.show()