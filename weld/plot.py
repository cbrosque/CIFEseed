import redis
#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

r = redis.Redis(host='127.0.0.1',port=6379)

value = r.get('sai2::optoforceSensor::6Dsensor::force')
value2 = r.get('sai2::optoforceSensor::3Dsensor::force')

start_time = time.time()
t = []
force_x = []
force_y = []
force_z = []
moment_x = []
moment_y = []
moment_z = []
force_x2 = []
force_y2 = []
force_z2 = []

figure, (ax1,ax2,ax3) = plt.subplots(3,1,figsize=(8,6))
line_force_x, = ax1.plot(t, force_x, label='force x')
line_force_y, = ax1.plot(t, force_y, label='force y')
line_force_z, = ax1.plot(t, force_z, label='force z')

line_moment_x, = ax2.plot(t, moment_x, label='moment x')
line_moment_y, = ax2.plot(t, moment_y, label='moment y')
line_moment_z, = ax2.plot(t, moment_z, label='moment z')

line_force_x2, = ax3.plot(t, force_x2, label='force x')
line_force_y2, = ax3.plot(t, force_y2, label='force y')
line_force_z2, = ax3.plot(t, force_z2, label='force z')

ax1.set_xlabel('time')
ax1.set_ylabel('force')
ax1.title.set_text('Force Received')
ax1.legend(loc=3)

ax2.set_xlabel('time')
ax2.set_ylabel('moment')
ax2.title.set_text('Moment Received')
ax2.legend(loc=3)

ax3.set_xlabel('time')
ax3.set_ylabel('force')
ax3.title.set_text('Force Exerted')
ax3.legend(loc=3)

figure.suptitle("Real Time Haptic Feedback")

def func_animate(i):
    value = r.get('sai2::optoforceSensor::6Dsensor::force')
    value = value.decode().strip('][').split(',')
    value2 = r.get('sai2::optoforceSensor::3Dsensor::force')
    value2 = value2.decode().strip('][').split(',')
    t.append(float(time.time() - start_time))
    force_x.append(float(value[0]))
    force_y.append(float(value[1]))
    force_z.append(float(value[2]))
    moment_x.append(float(value[3]))
    moment_y.append(float(value[4]))
    moment_z.append(float(value[5]))
    force_x2.append(float(value2[0]))
    force_y2.append(float(value2[1]))
    force_z2.append(float(value2[2]))

    line_force_x.set_data(t, force_x)
    line_force_y.set_data(t, force_y)
    line_force_z.set_data(t, force_z)
    line_moment_x.set_data(t, moment_x)
    line_moment_y.set_data(t, moment_y)
    line_moment_z.set_data(t, moment_z)
    line_force_x2.set_data(t, force_x2)
    line_force_y2.set_data(t, force_y2)
    line_force_z2.set_data(t, force_z2)
    #ax = plt.gca()
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()
    ax3.relim()
    ax3.autoscale_view()

ani = FuncAnimation(figure, func_animate, frames=10, interval=200)

plt.show()
