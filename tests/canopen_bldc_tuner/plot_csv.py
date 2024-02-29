import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('relay.csv')
df_peaks = pd.read_csv('relay_peaks.csv')

plt.rcParams["figure.autolayout"] = True
fig, (ax1,ax2,ax3) = plt.subplots(3, 1)

ax1.plot(df.Time, df.Period, label="Period")
ax1.set_xlabel('Time/s')

ax2.plot(df.Time, df.CurrentDemand, label="CurrentDemand")
ax2.plot(df.Time, df.CurrentActual, label="CurrentActual")
ax2.plot(df_peaks.Time, df_peaks.CurrentActual,color="red", marker='x', linestyle=" ")
ax2.set_xlabel('Time/s')
ax2.axhline(y=0.0, color='r', linestyle='-')

ax3.plot(df.Time, df.CurrentVelocity, label="CurrentVelocity")
ax3.plot(df_peaks.Time, df_peaks.CurrentVelocity,color="red", marker='x', linestyle=" ")
ax3.set_xlabel('Time/s')
ax3.axhline(y=0.0, color='r', linestyle='-')

ax1.legend()
ax2.legend()
ax3.legend()
plt.show()
