import pandas as pd
import matplotlib
matplotlib.use('GTK4Agg')
import matplotlib.pyplot as plt

df = pd.read_csv('sim_motor.csv')

plt.rcParams["figure.autolayout"] = True
fig, (ax1,ax2,ax3,ax4,ax5) = plt.subplots(5, 1)
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

ax4.plot(df.Time, df.pwm, label="pwm")
ax4.set_xlabel('Time/s')
ax4.legend()


ax5.plot(df.Time, df.te, label="t_e")
ax5.plot(df.Time, df.tl, label="t_l")
ax5.plot(df.Time, df.tf, label="t_f")
ax5.set_xlabel('Time/s')
ax5.legend()

plt.show()
