import pandas as pd
import matplotlib
matplotlib.use('GTK4Agg')
import matplotlib.pyplot as plt

df = pd.read_csv('sim_motor.csv')

plt.rcParams["figure.autolayout"] = True
fig, (ax1,ax2,ax3,ax4) = plt.subplots(4, 1)
ax1.plot(df.Time, df.v1, label="v1")
ax1.plot(df.Time, df.v2, label="v2")
ax1.plot(df.Time, df.v3, label="v3")
ax1.plot(df.Time, df.e1, label="e1")
ax1.plot(df.Time, df.e2, label="e2")
ax1.plot(df.Time, df.e3, label="e3")
ax1.set_xlabel('Time/s')
ax1.legend()

ax2.plot(df.Time, df.i1, label="i1")
ax2.plot(df.Time, df.i2, label="i2")
ax2.plot(df.Time, df.i3, label="i3")
ax2.set_xlabel('Time/s')
ax2.legend()

ax3.plot(df.Time, df.theta, label="theta")
ax3.plot(df.Time, df.omega, label="omega")
ax3.set_xlabel('Time/s')
ax3.legend()

ax4.plot(df.Time, df.te, label="t_e")
ax4.plot(df.Time, df.tl, label="t_l")
ax4.plot(df.Time, df.tf, label="t_f")
ax4.set_xlabel('Time/s')
ax4.legend()

plt.show()
