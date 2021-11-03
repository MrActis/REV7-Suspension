"""
Various functions for suspension calculations and visualization
"""
from dynamics import Vector
from sympy import ImmutableMatrix


def calc_susp_forces(d_lf: Vector, d_lr: Vector, d_uf: Vector, d_ur: Vector,
                     d_p: Vector, d_t: Vector, r_l: Vector, r_u: Vector,
                     r_p: Vector, r_t: Vector, f_t: Vector) -> ImmutableMatrix:
    """
    Calculates the suspension forces for a given suspension geometry and tire
    forces
    6 Unknown Forces: 2 lower CA rods (lca), 2 upper CA rods (uca),
                      push/pullrod (prod), tierod (tie)
    Order of variables in x for equation Ax = b:
        front rod of lower CA
        rear rod of lower CA
        front rod of upper CA
        rear rod of upper CA
        push/pullrod
        tierod
    :param d_lf: direction of lca front rod of type Vector
    :param d_lr: direction of lca rear rod of type Vector
    :param d_uf: direction of uca front rod of type Vector
    :param d_ur: direction of uca rear rod of type Vector
    :param d_p: direction of push/pullrod of type Vector
    :param d_t: direction of tierod of type Vector
    :param r_l: position of lca upright from tire contact of type Vector
    :param r_u: position of uca upright from tire contact of type Vector
    :param r_p: position of push/pullrod from tire contact of type Vector
    :param r_t: position of tierod from tire contact of type Vector
    :param f_t: tire forces of type Vector
    :return: force on each rod
    :rtype: ImmutableMatrix
    """
    # normalized force direction vectors
    n_lf = d_lf.unit()
    n_lr = d_lr.unit()
    n_uf = d_uf.unit()
    n_ur = d_ur.unit()
    n_p = d_p.unit()
    n_t = d_t.unit()

    # moments about tire-ground contact center point
    m_lf = r_l.cross(n_lf).get()
    m_lr = r_l.cross(n_lr).get()
    m_uf = r_u.cross(n_uf).get()
    m_ur = r_u.cross(n_ur).get()
    m_p = r_p.cross(n_p).get()
    m_t = r_t.cross(n_t).get()

    # system of linear equations matrix
    n_lf = n_lf.get()
    n_lr = n_lr.get()
    n_uf = n_uf.get()
    n_ur = n_ur.get()
    n_p = n_p.get()
    n_t = n_t.get()
    a = ImmutableMatrix([[n_lf[0], n_lr[0], n_uf[0], n_ur[0], n_p[0], n_t[0]],
                         [n_lf[1], n_lr[1], n_uf[1], n_ur[1], n_p[1], n_t[1]],
                         [n_lf[2], n_lr[2], n_uf[2], n_ur[2], n_p[2], n_t[2]],
                         [m_lf[0], m_lr[0], m_uf[0], m_ur[0], m_p[0], m_t[0]],
                         [m_lf[1], m_lr[1], m_uf[1], m_ur[1], m_p[1], m_t[1]],
                         [m_lf[2], m_lr[2], m_uf[2], m_ur[2], m_p[2], m_t[2]]])

    # solution in form (fx, fy, fz, mx, my, mz)
    f_t = -f_t.get()
    b = ImmutableMatrix([f_t[0], f_t[1], f_t[2], 0.0, 0.0, 0.0])

    # solve
    x = (a ** -1) * b
    print("F_lca_f: %s N" % x[0])
    print("F_lca_r: %s N" % x[1])
    print("F_uca_f: %s N" % x[2])
    print("F_uca_r: %s N" % x[3])
    print("F_prod: %s N" % x[4])
    print("F_tie: %s N" % x[5])
    return x