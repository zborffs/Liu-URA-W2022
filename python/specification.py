"""
1.) Check the Matthew Kelly Solver + Julia solver
-> specifically, check how the linearization works (is it linear at each time step or just one linearization)
2.) Define the "robustness measure" (function mapping specs to real numbers)
"""

import mtl
import numpy as np
import matplotlib.pyplot as plt

a, b = mtl.parse('a'), mtl.parse('b')
phi0 = ~a
phi1 = a & b
phi2 = a | b
phi3 = a ^ b
phi4 = a.iff(b)
phi5 = a.implies(b)

# Assumes piece wise constant interpolation.
data = {
    'a': [(0, 100), (1, -1), (3, -2)],
    'b': [(0, 20), (0.2, 2), (4, -10)]
}

phi = mtl.parse('F(a | b)')
print(phi(data))
# output: 100

# Evaluate at t=3
print(phi(data, time=3))
# output: 2

# Evaluate with discrete time
phi = mtl.parse('X b')
print(phi(data, dt=0.2))
# output: 2


def min_tilde(a : np.ndarray, k1):
    """
    as outlined in the paper
    :param a:
    :param k1:
    :return:
    """
    return -1 / k1 * np.log(np.exp(-k1 * a).sum())


def max_tilde(a: np.ndarray, k2):
    """

    :param a:
    :param k2:
    :return:
    """
    t = np.exp(k2 * a)
    return np.dot(a, t) / t.sum()


a1 = np.array([-1.0, 1.0, 3.1415, 2.123, -0.12351])
max_a1 = np.max(a1)
min_a1 = np.min(a1)
max_tilde_a1 = max_tilde(a1, 10.0)  # 10.0 is tuneable parameter
min_tilde_a1 = min_tilde(a1, 1.0)


p = plt.plot(a1, 'o-', color='black')
p = plt.plot(np.ones(a1.shape) * max_a1, 'o--', color='red')
p = plt.plot(np.ones(a1.shape) * max_tilde_a1, 'o', color='blue')
plt.show()

p = plt.plot(a1, 'o-', color='black')
p = plt.plot(np.ones(a1.shape) * min_a1, 'o--', color='red')
p = plt.plot(np.ones(a1.shape) * min_tilde_a1, 'o', color='blue')
plt.show()

