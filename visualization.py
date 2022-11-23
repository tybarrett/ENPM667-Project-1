"""visualization.py - visualizes data generated during main simulation loop"""

from mpl_toolkits import mplot3d
import numpy
import matplotlib.pyplot as plt

figure = plt.figure()
axes = plt.axes(projection="3d")
axes.set_xlabel("X")
axes.set_ylabel("Y")
axes.set_zlabel("Z")
axes.view_init(15, 45)

def plot_desired_point(point):
    axes.scatter(point[0, 0], point[1, 0], point[2, 0], c="blue", s=4)

def plot_actual_point(point):
    axes.scatter(point[0, 0], point[1, 0], point[2, 0], c="red", s=4)

def show_plot():
    plt.show()
