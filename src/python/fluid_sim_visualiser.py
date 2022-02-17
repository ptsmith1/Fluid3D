import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy
import pandas as pd

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
    lines = []
    plots = []
    fig, (ax) = plt.subplots(nrows=1, ncols=1)
    ax.set_xlim(-box_size/2, box_size/2)
    ax.set_ylim(-box_size/2, box_size/2)
    for i in range(num_particles):
        plot = ax.scatter([], [], color="red", s=5)
        line, = ax.plot([], [], color="black", linewidth=1)
        plots.append(plot)
        lines.append(line)
    ani = animation.FuncAnimation(fig, animate, repeat=True, frames=time_steps,
                                  fargs=(positions,plots,lines,), blit=True, interval=0 )
    plt.show()

def animate(i, positions, plots, lines):
    plot_points = list(range(0,i+1))
    if len(plot_points) >= 20:
        plot_points = list(range(i-19, i+1))
    for j in range(len(plots)):
        plots[j].set_offsets([positions.iloc[i][j],positions.iloc[i][j+1]])
        lines[j].set_xdata(positions.iloc[plot_points,[j]])
        lines[j].set_ydata(positions.iloc[plot_points,[j+1]])

    return plots + lines

positions, num_particles, timesteps, box_size = load_data()
x = positions.iloc[:,[0]]
plot(positions, num_particles, timesteps, box_size)
