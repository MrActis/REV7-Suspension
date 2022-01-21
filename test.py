from functions import calc_susp_forces
from dynamics import Vector
from sympy import ImmutableMatrix


"""ONLY EDIT BELOW HERE"""
# position vectors from tire-ground contact center point
r_lca = Vector(ImmutableMatrix([0.01054615, 0.02828674, 0.14469821]))
r_uca = Vector(ImmutableMatrix([-0.01092512, 0.05192928, 0.31960376]))
r_prod = Vector(ImmutableMatrix([0.02826612, 0.09813674, 0.17014013]))
r_tie = Vector(ImmutableMatrix([-0.04024216, 0.02803981, 0.14363697]))

# force direction vectors
d_lca_f = Vector(ImmutableMatrix([0.08889975, 0.32804100, 0.00020994]))
d_lca_r = Vector(ImmutableMatrix([-0.14396272, 0.35731698, -0.00033998]))
d_uca_f = Vector(ImmutableMatrix([0.11923413, 0.30438607, -0.01075016]))
d_uca_r = Vector(ImmutableMatrix([-0.14903065, 0.33707303, -0.01075645]))
d_prod = Vector(ImmutableMatrix([-0.06781108, 0.27812723, 0.34003992]))
d_tie = Vector(ImmutableMatrix([-0.11971362, 0.36090048, 0.00065860]))

# forces on tire
tire_force = Vector(ImmutableMatrix([960, 0, 800]))
"""DO NOT EDIT BELOW HERE"""

calc_susp_forces(d_lca_f, d_lca_r, d_uca_f, d_uca_r, d_prod, d_tie, r_lca,
                 r_uca, r_prod, r_tie, tire_force)