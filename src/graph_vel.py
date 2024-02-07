from matplotlib import pyplot as plt
import numpy as np


measured_vel = [] # add vals
cmd_vel = [] # add vals
time = [] # add vals

# shift all time values to start from 0
time = [t - time[0] for t in time]

# plt.plot(time, measured_vel, label="measured")
# plt.plot(time, cmd_vel, label="cmd")
# plt.legend()
# plt.xlabel("time (ms)")
# plt.ylabel("velocity (m/s)")
# plt.show()

diffs = []
measured_vel = np.array(measured_vel)
cmd_vel = np.array(cmd_vel)
# TODO: change this based on the approx phase length observed
approx_phase_length = 12
for i in range(0, len(time), 12):
    peak_cmd = np.argmax(cmd_vel[i : min(i + 10, len(cmd_vel))]) + i
    peak_measured = np.argmax(measured_vel[i : min(i + 10, len(measured_vel))]) + i
    diffs.append((peak_measured - peak_cmd) * 50)
    print("Peak cmd: ", peak_cmd, cmd_vel[peak_cmd])
    print("Peak measured: ", peak_measured, measured_vel[peak_measured])

print("Average difference: ", np.mean(diffs))
