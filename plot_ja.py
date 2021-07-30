import matplotlib.pyplot as plt
import numpy as np
import sys
import math

# data file to read
force_file = "./force_outputs.txt"
moment_file = "./moment_outputs.txt"

data_force = np.loadtxt(force_file ,skiprows=0)
data_moment = np.loadtxt(moment_file ,skiprows=0)

data_force = np.reshape(data_force, (-1, 3))
data_moment = np.reshape(data_moment, (-1, 4))

# Hack to get rid of the spike in force
data_force[31209][0] = 0.005
data_force[31209][1] = 0.0
data_force[31209][2] = 0.0

time_force = np.arange(np.shape(data_force)[0])
time_moment = np.arange(np.shape(data_moment)[0])

fig0 = plt.figure(0,figsize=(10,4))
plt.plot(time_force, data_force[:,0], 'b', label="force x")
plt.plot(time_force, data_force[:,1], 'r', label="force y")
plt.plot(time_force, data_force[:,2], 'g', label="force z")
plt.title('Force Received from Robot Force Torque Sensor', FontSize=24)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)

plt.ylabel('force', FontSize=18)
lgd = plt.legend(loc=(0.8,0.4),frameon=1)
fig0.savefig("./force_plot.png")
plt.clf()

fig1 = plt.figure(1,figsize=(10,4))
plt.plot(time_moment, data_moment[:,0], 'b', label="moment x")
plt.plot(time_moment, data_moment[:,1], 'r', label="moment y")
plt.plot(time_moment, data_moment[:,2], 'g', label="moment z")
plt.title('Moment Received from Robot Force Torque Sensor', FontSize=24)
lgd = plt.legend(loc=(0,0),frameon=1)

plt.ylabel('moment', FontSize=18)
lgd = plt.legend(loc=(0,0),frameon=1)
fig1.savefig("./moment_plot.png")
plt.clf()
