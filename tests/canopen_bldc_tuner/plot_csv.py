import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('relay.csv')
df_vel_peaks = pd.read_csv('relay_vel_peaks.csv')
df_cur_peaks = pd.read_csv('relay_cur_peaks.csv')
df_vel_valleys = pd.read_csv('relay_vel_valleys.csv')
df_cur_valleys = pd.read_csv('relay_cur_valleys.csv')

plt.rcParams["figure.autolayout"] = True
fig, (ax1,ax2) = plt.subplots(2, 1)

ax1.plot(df.Time, df.CurrentDemand, label="CurrentDemand")
ax1.plot(df.Time, df.CurrentActual, label="CurrentActual")
ax1.plot(df_cur_valleys.Time, df_cur_valleys.CurrentActual,color="green", marker='x', linestyle=" ", label ="Low")
ax1.plot(df_cur_peaks.Time, df_cur_peaks.CurrentActual,color="red", marker='x', linestyle=" ", label="High")
ax1.set_xlabel('Time/s')
ax1.axhline(y=0.0, color='r', linestyle='-')

ax2.plot(df.Time, df.CurrentVelocity, label="CurrentVelocity")
ax2.plot(df.Time, df.CurrentDemand*30, label="CurrentDemand")
ax2.plot(df_vel_peaks.Time, df_vel_peaks.CurrentVelocity,color="red", marker='x', linestyle=" ", label ="High")
ax2.plot(df_vel_valleys.Time, df_vel_valleys.CurrentVelocity,color="green", marker='x', linestyle=" ", label ="Low")
ax2.set_xlabel('Time/s')
ax2.axhline(y=0.0, color='r', linestyle='-')

ax1.legend()
ax2.legend()
plt.show()
