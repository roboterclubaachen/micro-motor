
from functools import partial
import pandas as pd
import matplotlib
matplotlib.use("GTK4Agg")
import signal 
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def read_csvs(i,axs):
    df_4 = pd.read_csv("sim_motor_4.csv")
    df_6 = pd.read_csv("sim_motor_6.csv")
    df_8 = pd.read_csv("sim_motor_8.csv")
    df_10 = pd.read_csv("sim_motor_10.csv")
    df_12 = pd.read_csv("sim_motor_12.csv")
    df_16 = pd.read_csv("sim_motor_16.csv")
    df_18 = pd.read_csv("sim_motor_18.csv")
    datas = [df_4,df_6,df_8,df_10,df_12,df_16,df_18]
    names = ["4","6","8","10","12","16","18"]
    for k,(ax,data,name) in enumerate(zip(axs.flat,datas,names)):
        try:
            ax.get_lines()[0].set_data(data.Time,data.omega)
        except:
            ax.plot(data.Time,data.omega)
        ax.set_xlabel('Time', fontsize=12)
        ax.set_ylabel('Omega', fontsize=12)
        ax.set_title("Motor {}".format(name), fontsize=14)
        factor = 1.2
        seconds = 20
        x_max = data.Time.max()+seconds*200
        x_min = x_max - seconds*1000
        y_lim = max(10,abs(data.omega.max()),abs(data.omega.min()))*factor
        ax.set_xlim(x_min,x_max)
        ax.set_ylim(-y_lim,y_lim)


def on_close(event):
    sys.exit(0)
    
def signal_handler(sig, frame):
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    plt.ion()
    plt.style.use("fivethirtyeight")
    fig, axs = plt.subplots(nrows=4, ncols=2, constrained_layout=True)
    fig.canvas.mpl_connect('close_event', on_close)
    while True:
        csv_func = partial(read_csvs,axs=axs)
        ani = FuncAnimation(fig, csv_func, interval=1000)

        plt.legend()
        plt.tight_layout()
        plt.pause(0.2)


if __name__ == "__main__":
    main()
