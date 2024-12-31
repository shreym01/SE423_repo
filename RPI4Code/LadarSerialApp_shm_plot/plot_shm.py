import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
import struct
from ctypes import *
so_file = "/home/pi/LadarSerialApp_shm_plot/plot_sem.so"
fcn = CDLL(so_file)
fcn.my_sem_open()
fig = plt.figure()
ax = fig.subplots(subplot_kw={'projection': 'polar'}) # use polar plot
line, = ax.plot([],[],'.')
#initialize the angle of the polar plot
ladar_dist_pong = np.zeros((228))
for i in range(228):
    ladar_dist_pong[i] = ((3*i+44)*0.3515625-135)*math.pi/180 #calculate the angle based on datasheet
#initialize the distances
shm_dist = [0]*228 

def animate(i):
    try:
        if (fcn.my_sem_trywait() == 0):
            shm_file = open('/dev/shm/posix-shared-mem-ladar-dist','rb') #open the shared memory
            plot_data = shm_file.read()
            for i in range(228):
                #unpack the binary data to float, need to divide by 1000 to get reading in meters
                shm_dist[i] = struct.unpack('f',plot_data[4*i:4*i+4])[0]/1000 
            shm_file.close()
    except:
        pass
    r = []
    for dist in shm_dist:
        r.append(float(dist))
    if(len(r)) == 228:
        line.set_ydata(r) #ydata is the distance
    return line,


def init():
    ax.set_rmax(6) #set max distance to 4 meters
    ax.set_rticks([1, 2, 3, 4, 5, 6])  # Less radial ticks
    ax.set_rlabel_position(-22.5)  # Move radial labels away from plotted line
    ax.grid(True) # grid on
    ax.set_theta_zero_location("N") # set 0 at top
    line.set_xdata(ladar_dist_pong) # xdata is the angle
    return line,

ani = animation.FuncAnimation(fig,animate,init_func = init,interval=50, blit=True)
plt.show()
