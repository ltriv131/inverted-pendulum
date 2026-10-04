import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray
import math
class Visualizer:
    def __init__(self, bounds):

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(bounds[0]-5, bounds[1]+5)
        self.ax.set_ylim(0, 20)
        self.ax.set_aspect("equal")
        self.ax.set_autoscale_on(False)
        self.connector_point = np.array([0, 0])
        self.pendulum_point = np.array([0,0])
        (self.line,) = self.ax.plot([self.connector_point[0], self.pendulum_point[0]], [self.connector_point[1], self.pendulum_point[1]], "ro-")
        (self.line_rail,) = self.ax.plot([bounds[0], bounds[1]], [10, 10], "bo-")
        

    def update(self, state: NDArray, r: float):
        self.connector_point = [state[0], 10]
        self.pendulum_point = r* np.array([np.cos(state[2]+math.pi/2.0), np.sin(state[2]+math.pi/2)])+self.connector_point
        self.line.set_data([self.connector_point[0], self.pendulum_point[0]], [self.connector_point[1], self.pendulum_point[1]])
        plt.draw()
        plt.pause(0.0001)
        