import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('vel.csv')

plt.rcParams["figure.autolayout"] = True
fig, (ax1,ax2,ax3) = plt.subplots(3, 1)
ax1.plot(df.Time, df.Current, label="Current", marker="o")
ax1.plot(df.Time, df.Error, label="Error")
ax1.plot(df.Time, df.Target, label="Target")
ax1.plot(df.Time, df.Commanded, label="Commanded")
#ax1.plot(df.Time, df.Charge, label="Charge")
ax1.set_xlabel('Time/s')

ax2.plot(df.Time, df.PWM, label="PWM Duty Cycle")
ax2.set_xlabel('Time/s')

ax3.plot(df.Time, df.Velocity, label="Velocity")
ax3.plot(df.Time, df.VelocityError, label="Velocity Error")
ax3.plot(df.Time, df.VelocityTarget, label="Velocity Target")
ax3.set_xlabel('Time/s')


ax1.legend()
ax2.legend()
ax3.legend()
plt.show()
