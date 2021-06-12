import redis
#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

r = redis.Redis(host='127.0.0.1',port=6379)

value = r.get('sai2::optoforceSensor::3Dsensor::force')
start_time = time.time()
t = []
force_x = []
force_y = []
force_z = []

figure, (ax1) = plt.subplots(1,1,figsize=(8,6))
line_force_x, = ax1.plot(t, force_x, label='force x')
line_force_y, = ax1.plot(t, force_y, label='force y')
line_force_z, = ax1.plot(t, force_z, label='force z')


ax1.set_xlabel('time')
ax1.set_ylabel('force')
ax1.legend(loc=3)


figure.suptitle("Real Time Haptic Force Exerted")

def func_animate(i):
    value = r.get('sai2::optoforceSensor::6Dsensor::force')
    value = value.decode().strip('][').split(',')
    
    t.append(float(time.time() - start_time))
    force_x.append(float(value[0]))
    force_y.append(float(value[1]))
    force_z.append(float(value[2]))

    line_force_x.set_data(t, force_x)
    line_force_y.set_data(t, force_y)
    line_force_z.set_data(t, force_z)
    #ax = plt.gca()
    ax1.relim()
    ax1.autoscale_view()

ani = FuncAnimation(figure, func_animate, frames=10, interval=200)

plt.show()
