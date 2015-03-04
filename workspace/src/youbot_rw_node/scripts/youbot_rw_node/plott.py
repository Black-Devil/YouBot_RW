#!/usr/bin/env python2

import kinematics_nummeric
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

kin=kinematics_nummeric.Kinematics_num()
sol = kin.search_all_solutions(np.array([0,0.1,0]),10)


def randrange(n, vmin, vmax):
    return (vmax-vmin)*np.random.rand(n) + vmin

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
n = 1000

for i in sol:
    xs = i[1]
    ys = i[2]
    zs = i[3]
    ax.scatter(xs, ys, zs, c='r', marker='o')


ax.set_xlabel('Joint 2')
ax.set_ylabel('Joint 3')
ax.set_zlabel('Joint 4')

plt.show()
