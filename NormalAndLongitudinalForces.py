# Calculate peak longitudinal tire forces and corresponding normal forces
# Link to information:
# https://x-engineer.org/automotive-engineering/chassis/vehicle-dynamics/tire-model-for-longitudinal-forces/
import math
import numpy as np
import matplotlib.pyplot as plt


# tire model load dependent coefficients (constants)
cx1 = 1.685
dx1 = 1.75
dx2 = 0.0
ex1 = 0.344
ex2 = 0.095
ex3 = -0.02
ex4 = 0.0
kx1 = 21.51
kx2 = -0.163
kx3 = 0.254
hx1 = -0.002
hx2 = 0.002
vx1 = 0.0
vx2 = 0.0
epsilon = 0.000000001

# vehicle and environmental parameters
mass = 247.661  # in kg
g = 9.80665  # in m/s^2
rear_dis = 0.533734  # rear weight distribution in percent
fz0 = rear_dis / 2 * mass * g

# wheel slip and tire vertical load
slip = np.linspace(-1, 1, 400)
fz = np.linspace(fz0, fz0 + 500, 500)

# store maximum forces
max_force = np.zeros(len(fz))

# tire longitudinal calculation
for i in range(len(fz)):
    dfz = (fz[i] - fz0) / fz0
    mux = dx1 + dx2 * dfz
    shx = hx1 + hx2 * dfz
    svx = fz[i] * (vx1 + vx2 * dfz)
    kxk = fz[i] * (kx1 + kx2 * dfz) * math.exp(kx3 * dfz)
    dx = mux * fz[i]
    cx = cx1
    bx = kxk / (cx * dx + epsilon)
    kx = slip + shx
    ex = (ex1 + ex2 * dfz + ex3 * (dfz ** 2)) * (1 - ex4 * np.sign(slip))
    fx = dx * np.sin(cx * np.arctan(bx * kx - ex * (bx * kx - np.arctan(bx * kx)))) + svx
    max_force[i] = np.amax(fx)

# load transfer calculations
acceleration = np.linspace(0, 2, 200)
cg_height = 0.2921  # in m
wheelbase = 1.562354  # in m
rear_load = 0.5 * mass * g * (cg_height / wheelbase) * acceleration
normal_force = rear_load + fz0

# plot setup
max_force = 2 * max_force / (mass * g)
plt.plot(fz, max_force)
plt.plot(normal_force, acceleration)
plt.grid(True)
plt.xlabel('normal force, Fz (N)')
plt.ylabel("car acceleration, g's")
plt.show()
