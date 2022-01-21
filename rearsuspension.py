"""
Rear quarter suspension model
"""
from dynamics import Vector, RefFrame
from sympy import ImmutableMatrix
import components as cp
import numpy as np
import matplotlib.pyplot as plt


# rear suspension hardpoints relative to shock tab at ride height
h_shock = Vector(ImmutableMatrix([0.00000000, 0.00000000, 0.00000000]))
h_lca_f = Vector(ImmutableMatrix([0.23765217, -0.16213265, -0.20740690]))
h_uca_f = Vector(ImmutableMatrix([0.24653557, -0.16214752, -0.04346145]))
h_rocker = Vector(ImmutableMatrix([0.09861381, -0.10861618, 0.1169589]))
h_arb = Vector(ImmutableMatrix([0.12307855, -0.12668519, 0.01522077]))

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

# rear arb hardpoints in local coordinates
h_arb_link = Vector(ImmutableMatrix([0.03022600, 0.00000000, 0.00000000]))

# rotation matrices
lca_transform = ImmutableMatrix([[0.99218671, 0.12473952, -0.00236156],
                                 [-0.12473987, 0.99218948, 0.00000000],
                                 [0.00234312, 0.00029458, 0.99999721]])
uca_transform = ImmutableMatrix([[0.99265843, 0.12088246, 0.00408251],
                                 [-0.12095137, 0.99208629, 0.03369796],
                                 [0.00002329, -0.03394435, 0.99942372]])
rocker_transform = ImmutableMatrix([[0.51559649, -0.16262435, -0.84125714],
                                    [-0.53113573, 0.70976961, -0.46273311],
                                    [0.67235043, 0.68540529, 0.27957912]])
arb_transform = ImmutableMatrix([[-0.98917348, 0.00000000, -0.14675087],
                                 [0.00000000, 1.00000000, 0.00000000],
                                 [0.14675087, 0.00000000, -0.98917348]])

# reference frames
global_frame = RefFrame()
lca_frame = RefFrame(h_lca_f, p=global_frame, r=lca_transform)
uca_frame = RefFrame(h_uca_f, p=global_frame, r=uca_transform)
rocker_frame = RefFrame(h_rocker, p=global_frame, r=rocker_transform)
arb_frame = RefFrame(h_arb, p=global_frame, r=arb_transform)

# suspension components
stiffness = 728.4537866  # in Nm
rear_lca = cp.ControlArm(h_lca_r, h_lca_up, lca_frame, prod=h_lca_pr, side='r')
rear_uca = cp.ControlArm(h_uca_r, h_uca_up, uca_frame, side='r')
rear_rocker = cp.Rocker(h_r_arb, h_r_pr, h_r_shock, rocker_frame, side='r')
rear_arb = cp.Arb(h_arb_link, arb_frame, stiffness, side='r')

# quarter suspension
static_force = 865.2889167210222  # compression in N
k = 31429.79901991958  # in N/m
rear_susp = cp.QuarterSusp(rear_lca, rear_uca, rear_rocker, rear_arb, k=k,
                           f_prod=static_force)
# calculate angles and forces
start_angle = -4.7  # in degrees
end_angle = 4.7  # in degrees
num_angles = 200
increment = (end_angle - start_angle) / (num_angles - 1)
angles_lca = np.linspace(start_angle, end_angle, num_angles)
angles_uca = np.zeros(num_angles)
angles_rocker = np.zeros(num_angles)
angles_arb = np.zeros(num_angles)
f_s_long = np.zeros(num_angles)
f_a_long = np.zeros(num_angles)
f_p_long = np.zeros(num_angles)
f_s_roll = np.zeros(num_angles)
f_a_roll = np.zeros(num_angles)
f_p_roll = np.zeros(num_angles)
rear_susp.rotate_lca(start_angle)
for i in range(num_angles):
    angles_uca[i] = rear_susp.uca.angle
    angles_rocker[i] = rear_susp.rocker.angle
    angles_arb[i] = rear_susp.arb.angle
    f_s_long[i], f_a_long[i], f_p_long[i] = rear_susp.get_forces()
    f_s_roll[i], f_a_roll[i], f_p_roll[i] = rear_susp.get_forces(False)
    rear_susp.rotate_lca(increment)

# plot angles and forces
plot1 = plt.figure(1)
plt.plot(angles_lca, angles_uca)
plt.grid(True)
plt.title('UCA Angle vs. LCA Angle')
plt.xlabel('LCA Angle (\u00b0)')
plt.ylabel('UCA Angle (\u00b0)')
plt.tight_layout()

plot2 = plt.figure(2)
plt.plot(angles_lca, angles_rocker)
plt.grid(True)
plt.title('Rocker Angle vs. LCA Angle')
plt.xlabel('LCA Angle (\u00b0)')
plt.ylabel('Rocker Angle (\u00b0)')
plt.tight_layout()

plot3 = plt.figure(3)
plt.plot(angles_lca, angles_arb)
plt.grid(True)
plt.title('ARB Angle vs. LCA Angle')
plt.xlabel('LCA Angle (\u00b0)')
plt.ylabel('ARB Angle (\u00b0)')
plt.tight_layout()

plot4 = plt.figure(4)
plt.plot(angles_rocker, angles_arb)
plt.grid(True)
plt.title('ARB Angle vs. Rocker Angle')
plt.xlabel('Rocker Angle (\u00b0)')
plt.ylabel('ARB Angle (\u00b0)')
plt.tight_layout()

plot5 = plt.figure(5)
plt.plot(angles_rocker, f_s_long, label='shock')
plt.plot(angles_rocker, f_a_long, label='arb')
plt.plot(angles_rocker, f_p_long, label='push/pullrod')
plt.grid(True)
plt.legend()
plt.title('Forces During Longitudinal Movement vs. Rocker Angle')
plt.xlabel('Rocker Angle (\u00b0)')
plt.ylabel('Compressive Force (N)')
plt.tight_layout()

plot6 = plt.figure(6)
plt.plot(angles_rocker, f_s_roll, label='shock')
plt.plot(angles_rocker, f_a_roll, label='arb')
plt.plot(angles_rocker, f_p_roll, label='push/pullrod')
plt.grid(True)
plt.legend()
plt.title('Forces During Roll vs. Rocker Angle')
plt.xlabel('Rocker Angle (\u00b0)')
plt.ylabel('Compressive Force (N)')
plt.tight_layout()

plot7 = plt.figure(7)
plt.plot(f_p_long, f_s_long)
plt.grid(True)
plt.title('Shock Force vs. Push/Pullrod Force During Longitudinal Movement')
plt.xlabel('Push/Pullrod Force (N)')
plt.ylabel('Shock Force (N)')
plt.tight_layout()

plot8 = plt.figure(8)
plt.plot(f_p_roll, f_s_roll)
plt.grid(True)
plt.title('Shock Force vs. Push/Pullrod Force During Roll')
plt.xlabel('Push/Pullrod Force (N)')
plt.ylabel('Shock Force (N)')
plt.tight_layout()

plot9 = plt.figure(9)
plt.plot(f_p_roll, f_a_roll)
plt.grid(True)
plt.title('ARB Force vs. Push/Pullrod Force During Roll')
plt.xlabel('Push/Pullrod Force (N)')
plt.ylabel('ARB Force (N)')
plt.tight_layout()

plot10 = plt.figure(10)
plt.plot(f_s_roll, f_a_roll)
plt.grid(True)
plt.title('ARB Force vs. Shock Force During Roll')
plt.xlabel('Shock Force (N)')
plt.ylabel('ARB Force (N)')
plt.tight_layout()

plot11 = plt.figure(11)
angles_lca = 39.3701 * np.deg2rad(angles_lca)
plt.plot(angles_lca / 2, f_a_roll / 2)
plt.grid(True)
plt.title('ARB Force vs. Wheel Travel')
plt.xlabel('Wheel Travel (in)')
plt.ylabel('ARB Force (N)')
plt.tight_layout()

plt.show()