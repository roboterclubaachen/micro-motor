import pandas as pd
import math
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("relay.csv")
time_np_raw = df[["Time"]].to_numpy()


plt.rcParams["figure.autolayout"] = True
fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

ax1.plot(df.Time, df.CurrentDemand, label="CurrentDemand")
ax1.plot(df.Time, df.CurrentActual, label="CurrentActual")

ax1.set_xlabel("Time/s")
ax1.axhline(y=0.0, color="r", linestyle="-")

ax2.plot(df.Time, df.CurrentVelocity, label="CurrentVelocity")
ax2.plot(df.Time, df.CurrentDemand * 25, label="CurrentDemand Scaled")

ax2.set_xlabel("Time/s")
ax2.axhline(y=0.0, color="r", linestyle="-")

ax3.plot(df.Time, df.CurrentPosition, label="CurrentPosition")
ax3.plot(df.Time, df.CurrentDemand * 25, label="CurrentDemand Scaled")

ax3.set_xlabel("Time/s")
ax3.axhline(y=0.0, color="r", linestyle="-")

try:
    df_cur_valleys = pd.read_csv("relay_cur_valleys.csv")
    ax1.plot(
        df_cur_valleys.Time,
        df_cur_valleys.CurrentActual,
        color="green",
        marker="x",
        linestyle=" ",
        label="Low",
    )
except:
    print("No Cur valleys")

try:
    df_cur_peaks = pd.read_csv("relay_cur_peaks.csv")
    ax1.plot(
        df_cur_peaks.Time,
        df_cur_peaks.CurrentActual,
        color="red",
        marker="x",
        linestyle=" ",
        label="High",
    )
except:
    print("No Cur peaks")

try:
    df_vel_peaks = pd.read_csv("relay_vel_peaks.csv")
    time_to = df_vel_peaks["Time"][1]
    time_to_index = np.argwhere(time_np_raw == time_to)[0, 0]
    ax2.plot(
        df_vel_peaks.Time,
        df_vel_peaks.CurrentVelocity,
        color="red",
        marker="x",
        linestyle=" ",
        label="High",
    )
except:
    time_to_index = time_np_raw.shape[0]

try:
    df_vel_valleys = pd.read_csv("relay_vel_valleys.csv")
    time_from = df_vel_valleys["Time"][0]
    time_from_index = np.argwhere(time_np_raw == time_from)[0, 0]
    ax2.plot(
        df_vel_valleys.Time,
        df_vel_valleys.CurrentVelocity,
        color="green",
        marker="x",
        linestyle=" ",
        label="Low",
    )

except:
    time_from_index = 0

ax1.legend()
ax2.legend()
plt.show()

time_np_incomp = df[["Time"]].to_numpy()[time_from_index:]
current_np_incomp = df[["CurrentActual"]].to_numpy()[time_from_index:]
velocity_np_incomp = df[["CurrentVelocity"]].to_numpy()[time_from_index:]
demand_np_incomp = df[["CurrentDemand"]].to_numpy()[time_from_index:]

time_np = np.array([time_np_incomp[0] / 1000])
current_np = np.array([current_np_incomp[0]])
velocity_np = np.array([velocity_np_incomp[0]])
demand_np = np.array([demand_np_incomp[0]])

for i in range(1, time_np_incomp.shape[0]):
    timeDiff = (time_np_incomp[i][0] - time_np_incomp[i - 1][0]) / 1000
    n = timeDiff / 0.001
    lastTime = time_np[-1]
    currDiff = (current_np_incomp[i] - current_np[-1]) / n
    velDiff = (velocity_np_incomp[i] - velocity_np[-1]) / n
    demDiff = (demand_np_incomp[i] - demand_np[-1]) / n
    timeDiff = 0.001

    for j in range(0, math.floor(n)):
        time_np = np.append(time_np, [time_np[-1] + timeDiff])
        current_np = np.append(current_np, [current_np[-1] + currDiff])
        velocity_np = np.append(velocity_np, [velocity_np[-1] + velDiff])
        demand_np = np.append(demand_np, [demand_np[-1] + demDiff])


fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

cfft = np.fft.rfft(current_np, axis=0)
cfft = np.absolute(cfft)
vfft = np.fft.rfft(velocity_np, axis=0)
vfft = np.absolute(vfft)
dfft = np.fft.rfft(demand_np, axis=0)
dfft = np.absolute(dfft)

fftfreq = np.fft.rfftfreq(current_np.shape[0], 0.001)

ax1.plot(fftfreq, cfft, label="Current FFT")
ax1.set_xlabel("Frequency/Hz")
ax1.legend()
ax2.plot(fftfreq, vfft, label="Velocity FFT")
ax2.set_xlabel("Frequency/Hz")
ax2.legend()
ax3.plot(fftfreq, dfft, label="Demand FFT")
ax3.set_xlabel("Frequency/Hz")
ax3.legend()
plt.show()
