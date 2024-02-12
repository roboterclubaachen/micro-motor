import pandas as pd
import matplotlib
matplotlib.use('GTK4Agg')
import matplotlib.pyplot as plt

df = pd.read_csv('sim_motor.csv')

plt.rcParams["figure.autolayout"] = True
fig, (ax1,ax2,ax3) = plt.subplots(3, 1)
ax1.plot(df.Time, df.v1, label="v1")
ax1.plot(df.Time, df.v2, label="v2")
ax1.plot(df.Time, df.v3, label="v3")
ax1.set_xlabel('Time/s')

ax2.plot(df.Time, df.i1, label="i1")
ax2.plot(df.Time, df.i2, label="i2")
ax2.plot(df.Time, df.i3, label="i3")
ax2.set_xlabel('Time/s')

ax3.plot(df.Time, df.theta, label="theta")
ax3.plot(df.Time, df.omega, label="theta")
ax3.set_xlabel('Time/s')


ax1.legend()
ax2.legend()
ax3.legend()
plt.show()
