"""
Components module
Contains classes for suspension components
Classes: ControlArm, Rocker, Shock
"""
from dynamics import Vector, CoordSys, make_rotation
from sympy import ImmutableMatrix, Symbol, nsolve


class ControlArm:
    """
    ControlArm class
    Defines a suspension control arm with origin at front hardpoint, x-axis
    from rear hardpoint to front hardpoint, y-axis to the left, and z-axis
    upwards
    Attributes:
        f_l : Vector
            front hardpoint in local coordinates
        r_l : Vector
            rear hardpoint in local coordinates
        u_l : Vector
            upright hardpoint in local coordinates
        p_l : Vector
            push/pullrod hardpoint in local coordinates, if it exists
        sys : CoordSys
            local coordinate system of control arm
        side : str
            'r' if CA is on right side, 'l' if CA is on left side
    Methods:
        __init__(rear, upright, sys, prod)
        get_f_g()
        get_r_g()
        get_u_g()
        get_p_g()
        force_up(force_f, force_r)
        rotate(a, side, deg)
    """
    def __init__(self, rear: Vector, upright: Vector, sys: CoordSys,
                 side: str, prod: Vector = None):
        """
        ControlArm class constructor
        :param rear: rear hardpoint in local coordinates of type Vector
        :param upright: upright hardpoint in local coordinates of type Vector
        :param sys: local coordinate system of control arm
        :param side: 'r' if CA is on right side, 'l' if CA is on left side
        :param prod: push/pullrod hardpoint in local coordinates of type Vector
        :raises: :class:'ValueError': if side is invalid
        """
        si = side.lower()
        if si != 'r' and si != 'l':
            raise ValueError('invalid side')
        self.f_l: Vector = Vector(ImmutableMatrix([0.0, 0.0, 0.0]))
        self.r_l: Vector = rear
        self.u_l: Vector = upright
        self.p_l: Vector = prod
        self.sys: CoordSys = sys
        self.side: str = side

    def get_f_g(self) -> Vector:
        """
        Calculates front hardpoint in parent coordinates
        :return: front hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.sys.get_coords(self.f_l, self.sys.get_parent())

    def get_r_g(self) -> Vector:
        """
        Calculates rear hardpoint in parent coordinates
        :return: rear hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.sys.get_coords(self.r_l, self.sys.get_parent())

    def get_u_g(self):
        """
        Calculates upright hardpoint in parent coordinates
        :return: upright hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.sys.get_coords(self.u_l, self.sys.get_parent())

    def get_p_g(self):
        """
        Calculates push/pullrod hardpoint in parent coordinates
        :return: push/pullrod hardpoint in parent coordinates
        :rtype: Vector
        :raises: :class:'ValueError': if there is no push/pullrod hardpoint
        """
        if self.p_l is None:
            raise ValueError('no push/pullrod')
        else:
            return self.sys.get_coords(self.p_l, self.sys.get_parent())

    def force_up(self, force_f: float, force_r: float) -> ImmutableMatrix:
        """
        Calculates the equivalent force at the upright hardpoint
        :param force_f: tensile force on the front rod of type float
        :param force_r: tensile force on the rear rod of type float
        :return: equivalent force at the upright hardpoint in local coordinates
        :rtype: ImmutableMatrix
        """
        f_front = force_f * -self.u_l.unit().get()
        p_rear = self.r_l.subtract(self.u_l)
        f_rear = force_r * p_rear.unit().get()
        f_upright = f_front + f_rear
        print('F_x: %s N' % f_upright[0])
        print('F_y: %s N' % f_upright[1])
        print('F_z: %s N' % f_upright[2])
        return f_upright

    def rotate(self, a: float, deg: bool = True):
        """
        Updates the coordinate system after upwards rotation
        :param a: angle of upwards rotation
        :param deg: true if angle is in degrees, false if angle is in radians
        :return: updates coordinate system
        """
        if self.side == 'r':
            a = -a
        rotation_l_new = make_rotation(a, 'x', deg=deg)
        rotation_g_l = self.sys.get_rotation()
        self.sys.set_rotation(rotation_g_l * rotation_l_new)


class Rocker:
    """
    Rocker class
    Defines a suspension rocker with origin at pivot hardpoint, x-axis from
    pivot hardpoint to push/pullrod hardpoint, and z-axis upwards
    Attributes:
        pi_l : Vector
            pivot hardpoint in local coordinates
        a_l : Vector
            arb hardpoint in local coordinates
        pr_l : Vector
            push/pullrod hardpoint in local coordinates
        s_l : Vector
            shock hardpoint in local coordinates
        sys : CoordSys
            local coordinate system of rocker
    Methods:
        __init__(arb, prod, shock sys)
        get_pi_g()
        get_a_g()
        get_pr_g()
        get_s_g()
        rotate(a, deg)
    """
    def __init__(self, arb: Vector, prod: Vector, shock: Vector, sys: CoordSys):
        """
        Rocker class constructor
        :param arb: arb hardpoint in local coordinates of type Vector
        :param prod: push/pullrod hardpoint in local coordinates of type Vector
        :param shock: shock hardpoint in local coordinates of type Vector
        :param sys: local coordinate system of rocker
        """
        self.pi_l: Vector = Vector(ImmutableMatrix([0.0, 0.0, 0.0]))
        self.a_l: Vector = arb
        self.pr_l: Vector = prod
        self.s_l: Vector = shock
        self.sys: CoordSys = sys

    def get_pi_g(self) -> Vector:
        """
        Calculates pivot hardpoint in parent coordinates
        :return: pivot hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.sys.get_coords(self.pi_l, self.sys.get_parent())

    def get_a_g(self) -> Vector:
        """
        Calculates arb hardpoint in parent coordinates
        :return: arb hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.sys.get_coords(self.a_l, self.sys.get_parent())

    def get_pr_g(self) -> Vector:
        """
        Calculates push/pullrod hardpoint in parent coordinates
        :return: push/pullrod hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.sys.get_coords(self.pr_l, self.sys.get_parent())

    def get_s_g(self) -> Vector:
        """
        Calculates shock hardpoint in parent coordinates
        :return: shock hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.sys.get_coords(self.s_l, self.sys.get_parent())

    def rotate(self, a: float, deg: bool = True):
        """
        Updates the coordinate system after counterclockwise rotation
        :param a: angle of counterclockwise rotation
        :param deg: true if angle is in degrees, false if angle is in radians
        :return: updates coordinate system
        """
        rotation_l_new = make_rotation(a, 'z', deg=deg)
        rotation_g_l = self.sys.get_rotation()
        self.sys.set_rotation(rotation_g_l * rotation_l_new)


class Shock:
    """
    Shock class
    Defines a suspension shock
    Attributes:
        k : float
            spring constant in N/m
        len : float
            initial length at ride height in m
        force : float
            initial force at ride height in N
    Methods:
        __init__(constant, length, force)
        get_force(new_len)
    """
    def __init__(self, constant: float, length: float, force: float):
        """
        Shock class constructor
        :param constant: spring constant in N/m of type float
        :param length: initial length at ride height in m of type float
        :param force: initial force at ride height in N of type float
        """
        self.__k: float = constant
        self.__len: float = length
        self.__force: float = force

    def get_force(self, new_len: float) -> float:
        """
        Calculates spring force with new length
        :param new_len: new length of spring in m
        :return: new spring force
        :rtype: float
        """
        return self.__force + self.__k * (self.__len - new_len)


class LongAccel:
    """
    LongAccel class
    Defines relevant suspension geometry for longitudinal acceleration
    Attributes:
        ca : ControlArm
            control arm with push/pullrod hardpoint
        rocker : Rocker
            rocker
        shock : Shock
            shock
        prod : float
            length of push/pullrod in m
        side : str
            'r' if on right side, 'l' if on left side
    Methods:
        __init__(ca, r, k, f_prod)
        make_shock(constant, f_prod)
        rotate_ca(a, deg)
        get_forces()
    """
    def __init__(self, ca: ControlArm, r: Rocker, k: float, f_prod: float):
        """
        LongAccel class constructor
        :param ca: control arm of type ControlArm
        :param r: rocker of type Rocker
        :param k: spring constant in N/m of type float
        :param f_prod: initial force on push/pullrod at ride height in N of
                       type float
        """
        self.ca: ControlArm = ca
        self.rocker: Rocker = r
        self.shock: Shock = self.make_shock(k, f_prod)
        self.prod: float = r.get_pr_g().subtract(ca.get_p_g()).norm()
        self.side: str = ca.side

    def make_shock(self, constant: float, f_prod: float) -> Shock:
        """
        Make shock for LongAccel object
        :param constant: spring constant in N/m of type float
        :param f_prod: initial force on push/pullrod at ride height in N of
                       type float, positive is compression
        :return: shock for LongAccel object
        :rtype: Shock
        """
        # moment balance to find shock force
        length = self.rocker.get_s_g().norm()
        r_l_g = self.rocker.sys.get_rotation() ** -1
        l_prod = self.rocker.get_pr_g().subtract(self.ca.get_p_g())
        l_prod = r_l_g * l_prod.get()
        n_prod = Vector(l_prod).unit()
        l_shock = r_l_g * self.rocker.get_s_g().get()
        n_shock = Vector(l_shock).unit()
        m_prod = self.rocker.pr_l.cross(n_prod)
        m_shock = self.rocker.s_l.cross(n_shock)
        f_shock = -f_prod * m_prod.get()[2] / m_shock.get()[2]
        return Shock(constant, length, f_shock)

    def rotate_ca(self, a: float, deg: bool = True) -> float:
        """
        Rotates CA, finds corresponding rocker rotation, and rotates rocker
        :param a: angle of upwards rotation
        :param deg: true if angle is in degrees, false if angle is in radians
        :return: rotates CA and rocker, returns new rocker angle
        :rtype: float
        """
        x = Symbol('x')
        self.ca.rotate(a, deg)
        r_l_n = make_rotation(x, 'z', deg=False)
        r_g_l = self.rocker.sys.get_rotation()
        r_g_n = r_g_l * r_l_n
        pr_g = self.rocker.get_pi_g().get() + r_g_n * self.rocker.pr_l.get()
        pr_len = pr_g - self.ca.get_p_g().get()
        length = self.prod ** 2
        eqn = pr_len[0] ** 2 + pr_len[1] ** 2 + pr_len[2] ** 2 - length
        sol = float(nsolve(eqn, x, 0.0))
        self.rocker.rotate(sol, deg=False)
        return sol

    def get_forces(self):
        """
        Calculates forces on shock and push/pullrod
        :return: force on shock and force on push/pullrod
        :rtype: tuple
        """
        # moment balance to find force on push/pullrod
        pr_g = self.rocker.get_pr_g()
        s_g = self.rocker.get_s_g()
        r_l_g = self.rocker.sys.get_rotation() ** -1
        l_prod = r_l_g * (pr_g.get() - self.ca.get_p_g().get())
        n_prod = Vector(l_prod).unit()
        l_shock = r_l_g * s_g.get()
        n_shock = Vector(l_shock).unit()
        m_prod = self.rocker.pr_l.cross(n_prod)
        m_shock = self.rocker.s_l.cross(n_shock)
        length = s_g.norm()
        f_shock = self.shock.get_force(length)
        f_prod = -f_shock * m_shock.get()[2] / m_prod.get()[2]
        return f_shock, f_prod