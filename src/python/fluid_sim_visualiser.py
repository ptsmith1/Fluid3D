import matplotlib.pyplot as plt
import matplotlib.axes
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
import time
from itertools import product, combinations
import math

paused = False

def load_data():
    meta_data = pd.read_csv('C:\\Users\\melti\\source\\repos\\Fluid3D\\build\\fluid_data.csv',nrows=1)
    num_particles = int(meta_data.iloc[0]['sim_size'])
    time_steps = int(meta_data.iloc[0]['timesteps'])
    box_size = meta_data.iloc[0]['box_size']
    cols=[0]
    for i in range(1,int(num_particles*6+1),6):
        cols.append(i)
        cols.append(i+1)
        cols.append(i+2)
    positions = pd.read_csv('C:\\Users\\melti\\source\\repos\\Fluid3D\\build\\fluid_data.csv',skiprows=2,dtype=float, header=0, index_col=0, usecols=cols)
    return positions, num_particles, time_steps, box_size

def plot(positions, num_particles, time_steps, box_size):
    labels = []
    lines = []
    plots = []
    fig = plt.figure(figsize=(10,7))
    ax = plt.subplot(111,projection='3d')
    ax.set_axis_off()
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.view_init(elev=0,azim=-90)

    # draw cube
    r = [-box_size, box_size]
    for s, e in combinations(np.array(list(product(r, r, r))), 2):
        if np.sum(np.abs(s - e)) == r[1] - r[0]:
            ax.plot3D(*zip(s, e), color="b")


    timestamp = ax.text(.03, .94, 0.94, s='Timestep: ', color='b', transform=ax.transAxes, fontsize='x-large')
    ax.set_xlim3d(-box_size, box_size)
    ax.set_ylim3d(-box_size, box_size)
    ax.set_zlim3d(-box_size, box_size)
    for i in range(num_particles):
        label = ax.text(positions.iloc[0,0], positions.iloc[0,1], positions.iloc[0,2],s='%d'%(i),zorder=0)
        plot, = ax.plot(positions.iloc[0,0], positions.iloc[0,1], positions.iloc[0,2], color="red", marker='o', markersize='11')
        line, = ax.plot(positions.iloc[0,[0]], positions.iloc[1,[1]], positions.iloc[2,[0]], color="black", linewidth=1)
        labels.append(label)
        plots.append(plot)
        lines.append(line)

    positions = positions.to_numpy(dtype='float32')
    global ani
    ani = animation.FuncAnimation(fig, animate, repeat=True, frames=time_steps,
                                  fargs=(positions,plots,lines,labels,timestamp,), blit=False, interval=0 )
    #ani.save('C:/Users/melti/Desktop/animation' + str(time.time()) + '.gif', writer='imagemagick', fps=60) #saves in 2d
    fig.canvas.mpl_connect('key_press_event', toggle_pause)
    plt.show()

def toggle_pause(*args, **kwargs):
    global paused
    if paused:
        ani.resume()
    else:
        ani.pause()
    paused = not paused

def animate(i, positions, plots, lines, labels, timestamp):
    plot_cutoff=25
    plot_points = list(range(0,i+1))
    if len(plot_points) >= plot_cutoff:
        plot_points = list(range(i-plot_cutoff-1, i+1))
    for j in range(len(plots)):
        labels[j].set_position([positions[i][j*3],positions[i][j*3+1],positions[i][j*3+2]])
        labels[j].set_3d_properties(positions[i][j*3 + 2])
        plots[j].set_data([positions[i][j*3],positions[i][j*3+1]])
        plots[j].set_3d_properties(positions[i][j*3+2])
        lines[j].set_data([positions[plot_points,j*3],positions[plot_points, j*3 + 1]])
        lines[j].set_3d_properties(positions[plot_points,j*3+2])
    timestamp.set_text('Timestep: ' + str(i))
    return plots + lines + labels + [timestamp]

positions, num_particles, timesteps, box_size = load_data()
x = positions.iloc[:,[0]]
plot(positions, num_particles, timesteps, box_size)
