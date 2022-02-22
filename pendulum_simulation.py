"""
This Python script is responsible for performing Inverted Pendulum simulations.
"""

# include numpy and the Pendulum model
import numpy as np
from model import Pendulum


"""
Setup the system and perform the simulation
"""
t_interval = [0, 3.5]  # the interval over which to numerically integrate

pend = Pendulum()  # object containing system parameters and member functions for doing simulation-related things
# initial_conditions = np.random.rand(pend.dim_n)  # random position
# initial_conditions = (np.pi / 2 - 0.5) * np.ones(pend.dim_n)  # upper-right position (nearly nominally stable)
# initial_conditions = (3 * np.pi / 2 + 0.5) * np.ones(pend.dim_n)  # upper-left position (nearly nominally stable)
# initial_conditions = np.pi * np.ones(pend.dim_n)  # down position
# initial_conditions = np.array([3 * np.pi / 4, -4.0])  # bottom-right position
initial_conditions = np.array([5 * np.pi / 4, 0.0])  # bottom-left position
goal = np.zeros(pend.dim_n)  # down position
pend.integrate(initial_conditions, goal, t_interval)  # member function for performing the numerical simulation and storing solution to members


"""
Visualize the numerical solution!
"""
sol_graph = pend.graph()
anim_graph = pend.animate()
sol_graph.show()
