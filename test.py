from functions import calc_susp_forces
from dynamics import Vector
from sympy import ImmutableMatrix


"""ONLY EDIT BELOW HERE"""
"""
# rear at ride height
# position vectors from tire-ground contact center point
r_lca = Vector(ImmutableMatrix([0.01042325, 0.02834733, 0.14467144]))
r_uca = Vector(ImmutableMatrix([-0.01080416, 0.05206487, 0.31959660]))
r_prod = Vector(ImmutableMatrix([0.03029467, 0.10189444, 0.17011844]))
r_tie = Vector(ImmutableMatrix([-0.04036650, 0.0280998, 0.14368113]))

# force direction vectors
d_lca_f = Vector(ImmutableMatrix([0.08889975, 0.32804100, 0.00020994]))
d_lca_r = Vector(ImmutableMatrix([-0.14396272, 0.35731698, -0.00033998]))
d_uca_f = Vector(ImmutableMatrix([0.11899026, 0.30431106, -0.03718577]))
d_uca_r = Vector(ImmutableMatrix([-0.14927453, 0.33699803, -0.03718577]))
d_prod = Vector(ImmutableMatrix([-0.04381773, 0.28102866, 0.33328414]))
d_tie = Vector(ImmutableMatrix([-0.11971219, 0.36090107, 0.00058766]))
"""
"""
# rear in full bump
r_lca = Vector(ImmutableMatrix([0.01014186, 0.03086573, 0.14413012]))
r_uca = Vector(ImmutableMatrix([-0.01066116, 0.05759815, 0.31867108]))
r_prod = Vector(ImmutableMatrix([0.03024195, 0.10612173, 0.16372911]))
r_tie = Vector(ImmutableMatrix([-0.04064996, 0.03054675, 0.14327609]))

# force direction vectors
d_lca_f = Vector(ImmutableMatrix([0.08883748, 0.32706193, -0.02554422]))
d_lca_r = Vector(ImmutableMatrix([-0.1440249, 0.3563379, -0.02609414]))
d_uca_f = Vector(ImmutableMatrix([0.11850361, 0.3003171, -0.06255573]))
d_uca_r = Vector(ImmutableMatrix([-0.14976117, 0.33300407, -0.06255573]))
d_prod = Vector(ImmutableMatrix([-0.04839332, 0.29198816, 0.3230734]))
d_tie = Vector(ImmutableMatrix([-0.11977238, 0.35999345, -0.02530278]))
"""

"""
# front at ride height
r_lca = Vector(ImmutableMatrix([0.00288856, -0.04829161, 0.14734338]))
r_uca = Vector(ImmutableMatrix([-0.00288856, -0.06117111, 0.31596838]))
r_prod = Vector(ImmutableMatrix([-0.00760754, -0.08856308, 0.28854218]))
r_tie = Vector(ImmutableMatrix([0.061956067, -0.02648516, 0.18099239]))

# force direction vectors
d_lca_f = Vector(ImmutableMatrix([0.055372, -0.30120097, 0.00013328]))
d_lca_r = Vector(ImmutableMatrix([-0.10795, -0.30120097, 0.00013328]))
d_uca_f = Vector(ImmutableMatrix([0.09848712, -0.26241347, -0.04088212]))
d_uca_r = Vector(ImmutableMatrix([-0.10141088, -0.25072947, -0.04088212]))
d_prod = Vector(ImmutableMatrix([0.00148715, -0.19547971, -0.15816954]))
d_tie = Vector(ImmutableMatrix([0.06867094, -0.3267456, -0.0150622]))
"""


# front in full bump
r_lca = Vector(ImmutableMatrix([0.00304517, -0.05200627, 0.14607918]))
r_uca = Vector(ImmutableMatrix([-0.00304517, -0.06914986, 0.31431311]))
r_prod = Vector(ImmutableMatrix([-0.00759616, -0.09366771, 0.284264]))
r_tie = Vector(ImmutableMatrix([0.061956067, -0.02648516, 0.18099239]))

# force direction vectors
d_lca_f = Vector(ImmutableMatrix([0.055372, -0.30010618, -0.0256578]))
d_lca_r = Vector(ImmutableMatrix([-0.10795, -0.30010618, -0.0256578]))
d_uca_f = Vector(ImmutableMatrix([0.09880034, -0.25705459, -0.06628212]))
d_uca_r = Vector(ImmutableMatrix([-0.10109766, -0.24537059, -0.06628212]))
d_prod = Vector(ImmutableMatrix([-0.00159251, -0.18779359, -0.1672219]))
d_tie = Vector(ImmutableMatrix([0.06867094, -0.3267456, -0.0150622]))

# forces on tire
tire_force = Vector(ImmutableMatrix([-878, 0, 1136]))
"""DO NOT EDIT BELOW HERE"""

calc_susp_forces(d_lca_f, d_lca_r, d_uca_f, d_uca_r, d_prod, d_tie, r_lca,
                 r_uca, r_prod, r_tie, tire_force)