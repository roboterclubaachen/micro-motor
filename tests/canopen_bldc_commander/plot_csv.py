import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("vel.csv")

plt.rcParams["figure.autolayout"] = True
fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1)
ax1.plot(df.Time, df.Current, label="Current")
ax1.plot(df.Time, df.Commanded, label="Commanded")
ax1.set_xlabel("Time/s")
ax1.axhline(y=0.0, color="r", linestyle="-")

ax2.plot(df.Time, df.Velocity, label="Velocity")
ax2.plot(df.Time, df.VelocityTarget, label="Velocity Target")
ax2.set_xlabel("Time/s")
ax2.axhline(y=0.0, color="r", linestyle="-")

ax3.plot(df.Time, df.Position, label="Position")
ax3.plot(df.Time, df.PositionTarget, label="Position Target")
ax3.set_xlabel("Time/s")
ax3.axhline(y=0.0, color="r", linestyle="-")

ax4.plot(df.Time, df.PWM, label="PWM")
ax4.set_xlabel("Time/s")
ax4.axhline(y=0.0, color="r", linestyle="-")

ax5.plot(df.Time, df.Mode, label="Mode")
ax5.set_xlabel("Time/s")
ax5.axhline(y=0.0, color="r", linestyle="-")

ax6.plot(df.Time, df.Charge, label="Charge")
ax6.axhline(y=0.1, color="r", linestyle="-")
ax6.set_xlabel("Time/s")

# ax6.axhline(y=0.0, color='r', linestyle='-')

ax1.legend()
ax2.legend()
ax3.legend()
ax4.legend()
ax5.legend()
plt.show()
