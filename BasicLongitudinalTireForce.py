# Calculate peak longitudinal tire forces
# Link to information:
# https://x-engineer.org/automotive-engineering/chassis/vehicle-dynamics/tire-model-for-longitudinal-forces/
import math
import numpy as np
import matplotlib.pyplot as plt


# tire model constant coefficients
B = 10.0
C = 1.9
D = 1.0
E = 0.97

# vehicle and environmental parameters
mass = 247.661  # in kg
g = 9.80665  # in m/s^2
rear_dis = 0.533734  # rear weight distribution in percent
fz0 = rear_dis / 2 * mass * g

# wheel slip and tire vertical load
slip = np.linspace(-1, 1, 400)
fz = np.linspace(fz0, fz0 + 500, 10)

# store maximum forces
max_force = np.zeros(len(fz))

# tire longitudinal calculation
for i in range(len(fz)):
    fx = fz[i] * D * np.sin(C * np.arctan(B * slip - E * (B * slip - np.arctan(B * slip))))
    max_force[i] = np.amax(fx)
    plt.plot(slip, fx)

# plot setup
print(max_force)
plt.grid(True)
plt.xlabel('tire slip')
plt.ylabel('tire longitudinal force, Fx (N)')
plt.legend(fz)
plt.show()
