import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('vel.csv')

plt.rcParams["figure.autolayout"] = True
fig, (ax1,ax2,ax3) = plt.subplots(3, 1)
ax1.plot(df.Time, df.Velocity, label="Velocity")
ax1.plot(df.Time, df.VelError, label="Error")
ax1.set_xlabel('Time/s')

ax2.plot(df.Time, df.PWM, label="PWM Duty Cycle")
ax2.set_xlabel('Time/s')

ax3.plot(df.Time, df.Position, label="Position")
ax3.plot(df.Time, df.PosError, label="Error")
ax3.set_xlabel('Time/s')

ax1.legend()
ax2.legend()
plt.show()