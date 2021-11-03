"""
Rear suspension model
"""
from dynamics import Vector, CoordSys
from sympy import ImmutableMatrix
import components as cp
import numpy as np
import matplotlib.pyplot as plt


# rear suspension hardpoints relative to shock tab at ride height
h_shock = Vector(ImmutableMatrix([0.00000000, 0.00000000, 0.00000000]))
h_lca_f = Vector(ImmutableMatrix([0.23765217, -0.16213265, -0.20740690]))
h_uca_f = Vector(ImmutableMatrix([0.24653557, -0.16214752, -0.04346145]))
h_rocker = Vector(ImmutableMatrix([0.09861381, -0.10861618, 0.1169589]))

# lca hardpoints in local coordinates
h_lca_pr = Vector(ImmutableMatrix([-0.03668432, -0.26110929, 0.02540000]))
h_lca_up = Vector(ImmutableMatrix([-0.04728585, -0.33656820, 0.00000000]))
h_lca_r = Vector(ImmutableMatrix([-0.23469622, 0.00000000, 0.00000000]))

# uca hardpoints in local coordinates
h_uca_up = Vector(ImmutableMatrix([-0.08154261, -0.31675547, 0.00000000]))
h_uca_r = Vector(ImmutableMatrix([-0.27024884, 0.00000000, 0.00000000]))

# rear rocker hardpoints in local coordinates
h_r_arb = Vector(ImmutableMatrix([0.01387778, -0.01653890, 0.00000000]))
h_r_pr = Vector(ImmutableMatrix([0.05080000, 0.00000000, 0.00000000]))
h_r_shock = Vector(ImmutableMatrix([-0.02606194, 0.07160458, 0.00000000]))

# rotation matrices
lca_transform = ImmutableMatrix([[0.99218671, 0.12473952, -0.00236156],
                                 [-0.12473987, 0.99218948, 0.00000000],
                                 [0.00234312, 0.00029458, 0.99999721]])
rocker_transform = ImmutableMatrix([[0.51559649, -0.16262435, -0.84125714],
                                    [-0.53113573, 0.70976961, -0.46273311],
                                    [0.67235043, 0.68540529, 0.27957912]])

# coordinate systems
global_sys = CoordSys(h_shock)
lca_sys = CoordSys(h_lca_f, p=global_sys, r=lca_transform)
rocker_sys = CoordSys(h_rocker, p=global_sys, r=rocker_transform)

# components
rear_lca = cp.ControlArm(h_lca_r, h_lca_up, lca_sys, side='r', prod=h_lca_pr)
rear_rocker = cp.Rocker(h_r_arb, h_r_pr, h_r_shock, rocker_sys)

# longitudinal acceleration
static_force = 865.2889167210222  # compression in N
k = 31429.79901991958  # in N/m
rear_susp = cp.LongAccel(rear_lca, rear_rocker, k, static_force)

# calculate shock and pushrod forces at different angles
start_angle = -4.7  # in degrees
end_angle = 4.7  # in degrees
rock_angle = rear_susp.rotate_ca(start_angle)
angles_ca = np.linspace(start_angle, end_angle, 200)
rotate = angles_ca[1] - angles_ca[0]
angles_rocker = np.zeros(angles_ca.size)
shock_forces = np.zeros(angles_ca.size)
prod_forces = np.zeros(angles_ca.size)
angles_rocker[0] = rock_angle
shock_forces[0], prod_forces[0] = rear_susp.get_forces()
for i in range(1, angles_ca.size):
    angles_rocker[i] = angles_rocker[i-1] + rear_susp.rotate_ca(rotate)
    shock_forces[i], prod_forces[i] = rear_susp.get_forces()
angles_rocker = np.rad2deg(angles_rocker)
# plot shock forces vs. prod forces
plt.plot(angles_ca, angles_rocker)
plt.grid(True)
plt.xlabel("CA Angle (deg)")
plt.ylabel("Rocker Angle (deg)")
plt.show()