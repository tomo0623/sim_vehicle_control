import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv('./build/simout.csv', header=None)
data = df.to_numpy()

# 走行軌道
fig1, ax1 = plt.subplots()
ax1.plot(data[:,5], data[:,6], label="ref", linestyle='dashed', linewidth=3.0)
ax1.plot(data[:,1], data[:,2], label="act")
ax1.set_title("Vehicle Trajectory")
ax1.legend()
ax1.grid()
ax1.set_ylabel('Y [m]')
ax1.set_xlabel('X [m]')

# 状態・入力
fig, ax = plt.subplots(2, 2, figsize=(15,8), sharex="all", layout="tight")
# fig, ax = plt.subplots(2, 2, figsize=(15,8), sharex="all", layout="constrained")
ax[0,0].plot(data[:,0], data[:,1])
ax[0,0].set_ylabel("posx [m]")
ax[0,0].set_xlabel("time [s]")
ax[0,1].plot(data[:,0], data[:,2])
ax[0,1].set_ylabel("posy [m]")
ax[0,1].set_xlabel("time [s]")
ax[1,0].plot(data[:,0], data[:,3]*np.pi)
ax[1,0].set_ylabel("theta [deg]")
ax[1,0].set_xlabel("time [s]")
ax[1,1].plot(data[:,0], data[:,4]*np.pi)
ax[1,1].plot([data[0,0], data[-1,0]], [30, 30], linestyle='dashed', color='r')
ax[1,1].plot([data[0,0], data[-1,0]], [-30, -30], linestyle='dashed', color='r')
ax[1,1].set_ylabel("steer [deg]")
ax[1,1].set_xlabel("time [s]")

for i in range(2):
    for j in range(2):
        ax[i,j].legend()
        ax[i,j].grid()
        # ax[i,j].set_xlim([x_st,x_ed])
        ax[i,j].minorticks_on()
        ax[i,j].grid(which='minor', lw=0.4)


# 制御誤差
fig, ax = plt.subplots(2, 2, figsize=(15,8), sharex="all", layout="tight")
# fig, ax = plt.subplots(2, 2, figsize=(15,8), sharex="all", layout="constrained")
ax[0,0].plot(data[:,0], data[:,7])
ax[0,0].set_ylabel("tracking error [m]")
ax[0,0].set_xlabel("time [s]")
ax[0,1].plot(data[:,0], data[:,12])
ax[0,1].set_ylabel("vy [m/s]")
ax[0,1].set_xlabel("time [s]")
ax[1,0].plot(data[:,0], data[:,13]*np.pi)
ax[1,0].set_ylabel("gamma [deg/s]")
ax[1,0].set_xlabel("time [s]")
ax[1,1].plot(data[:,0], data[:,4]*np.pi)
ax[1,1].plot([data[0,0], data[-1,0]], [30, 30], linestyle='dashed', color='r')
ax[1,1].plot([data[0,0], data[-1,0]], [-30, -30], linestyle='dashed', color='r')
ax[1,1].set_ylabel("steer [deg]")
ax[1,1].set_xlabel("time [s]")

for i in range(2):
    for j in range(2):
        ax[i,j].legend()
        ax[i,j].grid()
        # ax[i,j].set_xlim([x_st,x_ed])
        ax[i,j].minorticks_on()
        ax[i,j].grid(which='minor', lw=0.4)
        
        
plt.show()