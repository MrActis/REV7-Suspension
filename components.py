"""
Components module
Contains classes for suspension components
Classes: ControlArm, Rocker, Arb, Shock, QuarterSusp
"""
from dynamics import Vector, RefFrame, make_rotation
from sympy import ImmutableMatrix, Symbol, nsolve
from math import degrees, radians


class ControlArm:
    """
    ControlArm class
    Defines a suspension control arm with origin at front hardpoint, x-axis
    from rear hardpoint to front hardpoint, y-axis to the left, and z-axis
    upwards
    Attributes:
        f : Vector
            front hardpoint in local coordinates
        r : Vector
            rear hardpoint in local coordinates
        u : Vector
            upright hardpoint in local coordinates
        pr : Vector
            push/pullrod hardpoint in local coordinates, if it exists
        frame : RefFrame
            local reference frame of control arm
        side : str
            'r' if CA is on right side, 'l' if CA is on left side
        angle : float
            upwards angle from ride height in degrees
    Methods:
        __init__(rear, upright, frame, prod, side)
        axis()
        get_f_p()
        get_r_p()
        get_u_p()
        get_p_p()
        force_u(force_f, force_r)
        rotate(a, actual, deg)
    """
    def __init__(self, rear: Vector, upright: Vector, frame: RefFrame,
                 prod: Vector = None, side: str = ''):
        """
        ControlArm class constructor
        :param rear: rear hardpoint in local coordinates of type Vector
        :param upright: upright hardpoint in local coordinates of type Vector
        :param frame: local reference frame of control arm of type RefFrame
        :param prod: push/pullrod hardpoint in local coordinates of type Vector
        :param side: 'r' if CA is on right side, 'l' if CA is on left side
        :raises: :class:'ValueError': if side is invalid
        """
        s = side.lower()
        if s != 'r' and s != 'l':
            raise ValueError('invalid side')
        self.f = Vector(ImmutableMatrix([0.0, 0.0, 0.0]))
        self.r = rear
        self.u = upright
        self.frame = frame
        self.pr = prod
        self.side = s
        self.angle = 0.0

    @staticmethod
    def axis() -> str:
        """
        Return rotation axis of control arm
        :return: rotation axis of control arm
        :rtype: str
        """
        return 'x'

    def get_f_p(self) -> Vector:
        """
        Calculates front hardpoint in parent coordinates
        :return: front hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.f, self.frame.get_parent())

    def get_r_p(self) -> Vector:
        """
        Calculates rear hardpoint in parent coordinates
        :return: rear hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.r, self.frame.get_parent())

    def get_u_p(self):
        """
        Calculates upright hardpoint in parent coordinates
        :return: upright hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.u, self.frame.get_parent())

    def get_pr_p(self):
        """
        Calculates push/pullrod hardpoint in parent coordinates
        :return: push/pullrod hardpoint in parent coordinates
        :rtype: Vector
        :raises: :class:'ValueError': if there is no push/pullrod hardpoint
        """
        if self.pr is None:
            raise ValueError('no push/pullrod hardpoint')
        else:
            return self.frame.get_coords(self.pr, self.frame.get_parent())

    def force_u(self, force_f: float, force_r: float) -> ImmutableMatrix:
        """
        Calculates the equivalent force at the upright hardpoint
        :param force_f: tensile force on the front rod of type float
        :param force_r: tensile force on the rear rod of type float
        :return: equivalent force at the upright hardpoint in local coordinates
        :rtype: ImmutableMatrix
        """
        f_front = force_f * -self.u.unit().v
        f_rear = force_r * self.r.subtract(self.u).unit().v
        f_up = f_front + f_rear
        print('F_x: %s N' % f_up[0])
        print('F_y: %s N' % f_up[1])
        print('F_z: %s N' % f_up[2])
        return f_up

    def rotate(self, a: float, actual: bool = False, deg: bool = True):
        """
        Updates the reference frame after rotation
        :param a: angle of upwards rotation of type float
        :param actual: true if actual rotation, false if upwards rotation
        :param deg: true if angle is in degrees, false if angle is in radians
        :return: updates reference frame
        """
        if actual:
            if self.side == 'r':
                a_up = -a
            else:
                a_up = a
        else:
            a_up = a
            if self.side == 'r':
                a = -a
        if deg:
            self.angle = self.angle + a_up
        else:
            self.angle = self.angle + degrees(a_up)
        rotation_l_new = make_rotation(a, ControlArm.axis(), deg=deg)
        rotation_g_l = self.frame.get_rotation()
        self.frame.set_rotation(rotation_g_l * rotation_l_new)


class Rocker:
    """
    Rocker class
    Defines a suspension rocker with origin at pivot hardpoint, x-axis from
    pivot hardpoint to push/pullrod hardpoint, and z-axis upwards
    Attributes:
        pi : Vector
            pivot hardpoint in local coordinates
        a : Vector
            arb hardpoint in local coordinates
        pr : Vector
            push/pullrod hardpoint in local coordinates
        s : Vector
            shock hardpoint in local coordinates
        frame : RefFrame
            local reference frame of rocker
        side : str
            'r' if rocker is on right side, 'l' if rocker is on left side
        angle : float
            inwards angle from ride height in degrees
    Methods:
        __init__(arb, prod, shock, frame, side)
        axis()
        get_pi_p()
        get_a_p()
        get_pr_p()
        get_s_p()
        rotate(a, actual, deg)
    """
    def __init__(self, arb: Vector, prod: Vector, shock: Vector,
                 frame: RefFrame, side: str = ''):
        """
        Rocker class constructor
        :param arb: arb hardpoint in local coordinates of type Vector
        :param prod: push/pullrod hardpoint in local coordinates of type Vector
        :param shock: shock hardpoint in local coordinates of type Vector
        :param frame: local reference frame of rocker of type RefFrame
        :param side: 'r' if rocker is on right side, 'l' if rocker is on left
                     side
        :raises: :class:'ValueError': if side is invalid
        """
        s = side.lower()
        if s != 'r' and s != 'l':
            raise ValueError('invalid side')
        self.pi = Vector(ImmutableMatrix([0.0, 0.0, 0.0]))
        self.a = arb
        self.pr = prod
        self.s = shock
        self.frame = frame
        self.side = s
        self.angle = 0.0

    @staticmethod
    def axis() -> str:
        """
        Return rotation axis of rocker
        :return: rotation axis of rocker
        :rtype: str
        """
        return 'z'

    def get_pi_p(self) -> Vector:
        """
        Calculates pivot hardpoint in parent coordinates
        :return: pivot hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.pi, self.frame.get_parent())

    def get_a_p(self) -> Vector:
        """
        Calculates arb hardpoint in parent coordinates
        :return: arb hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.a, self.frame.get_parent())

    def get_pr_p(self) -> Vector:
        """
        Calculates push/pullrod hardpoint in parent coordinates
        :return: push/pullrod hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.pr, self.frame.get_parent())

    def get_s_p(self) -> Vector:
        """
        Calculates shock hardpoint in parent coordinates
        :return: shock hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.s, self.frame.get_parent())

    def rotate(self, a: float, actual: bool = False, deg: bool = True):
        """
        Updates the reference frame after rotation
        :param a: angle of rotation of type float
        :param actual: true if actual rotation, false if inwards rotation
        :param deg: true if angle is in degrees, false if angle is in radians
        :return: updates reference frame
        """
        if actual:
            if self.side == 'l':
                a_in = -a
            else:
                a_in = a
        else:
            a_in = a
            if self.side == 'l':
                a = -a
        if deg:
            self.angle = self.angle + a_in
        else:
            self.angle = self.angle + degrees(a_in)
        rotation_l_new = make_rotation(a, Rocker.axis(), deg=deg)
        rotation_g_l = self.frame.get_rotation()
        self.frame.set_rotation(rotation_g_l * rotation_l_new)


class Arb:
    """
    Arb class
    Defines a suspension arb with origin at pivot hardpoint, x-axis from pivot
    hardpoint to arb link hardpoint, and y-axis to left
    Attributes:
        pi : Vector
            pivot hardpoint in local coordinates
        link : Vector
            arb link hardpoint in local coordinates
        frame : RefFrame
            local reference frame of arb
        k : float
            torsion spring constant (GJ/L) in Nm
        side : str
            'f' if arb is in front, 'r' if arb is in rear
        angle : float
            upwards angle from ride height in degrees
    Methods:
        __init__(link, frame, k, side)
        axis()
        get_pi_p()
        get_link_p()
        get_torque()
        rotate(a, actual, deg)
    """
    def __init__(self, link: Vector, frame: RefFrame, k: float, side: str = ''):
        """
        Arb class constructor
        :param link: arb link hardpoint in local coordinates of type Vector
        :param frame: local reference frame of arb of type RefFrame
        :param k: torsion spring constant (GJ/L) in Nm
        :param side: 'f' if arb is in front, 'r' if arb is in rear
        :raises: :class:'ValueError': if side is invalid
        """
        s = side.lower()
        if s != 'f' and s != 'r':
            raise ValueError('invalid side')
        self.pi = Vector(ImmutableMatrix([0.0, 0.0, 0.0]))
        self.link = link
        self.frame = frame
        self.k = k
        self.side = s
        self.angle = 0.0

    @staticmethod
    def axis() -> str:
        """
        Return rotation axis of arb
        :return: rotation axis of arb
        :rtype: str
        """
        return 'y'

    def get_pi_p(self) -> Vector:
        """
        Calculates pivot hardpoint in parent coordinates
        :return: pivot hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.pi, self.frame.get_parent())

    def get_link_p(self) -> Vector:
        """
        Calculates link hardpoint in parent coordinates
        :return: link hardpoint in parent coordinates
        :rtype: Vector
        """
        return self.frame.get_coords(self.link, self.frame.get_parent())

    def get_torque(self) -> float:
        """
        Calculates "upwards" torque on arb
        :return: "upwards" torque
        :rtype: float
        """
        return self.k * radians(self.angle)

    def rotate(self, a: float, actual: bool = False, deg: bool = True):
        """
        Updates the reference frame after rotation
        :param a: angle of rotation
        :param actual: true if actual rotation, false if upwards rotation
        :param deg: true if angle is in degrees, false if angle is in radians
        :return: updates reference frame
        """
        if actual:
            if self.side == 'f':
                a_up = -a
            else:
                a_up = a
        else:
            a_up = a
            if self.side == 'f':
                a = -a
        if deg:
            self.angle = self.angle + a_up
        else:
            self.angle = self.angle + degrees(a_up)
        rotation_l_new = make_rotation(a, Arb.axis(), deg=deg)
        rotation_g_l = self.frame.get_rotation()
        self.frame.set_rotation(rotation_g_l * rotation_l_new)


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
        self.__k = constant
        self.__len = length
        self.__force = force

    def get_force(self, new_len: float) -> float:
        """
        Calculates spring force with new length
        :param new_len: new length of spring in m
        :return: new spring force
        :rtype: float
        """
        return self.__force + self.__k * (self.__len - new_len)


class QuarterSusp:
    """
    QuarterSusp class
    Defines suspension geometry for a quarter suspension model
    Attributes:
        lca : ControlArm
            lower control arm
        uca : ControlArm
            upper control arm
        p_ca : ControlArm
            control arm with push/pullrod hardpoint
        rocker : Rocker
            rocker
        arb : Arb
            arb
        shock : Shock
            shock
        p_len : float
            length of push/pullrod in m
        ca_len : float
            distance between control arm upright hardpoints in m
        link_len : float
            length of arb link in m
        side : str
            'r' if on right side, 'l' if on left side
    Methods:
        __init__(lca, uca, r, a, s, k, f_prod)
        __make_shock(constant, f_prod)
        rotate_lca(a, actual, deg)
        get_forces(longitudinal)
    """
    def __init__(self, lca: ControlArm, uca: ControlArm, r: Rocker, a: Arb,
                 s: Shock = None, k: float = 0.0, f_prod: float = 0.0):
        """
        QuarterSusp class constructor
        :param lca: lower control arm of type ControlArm
        :param uca: upper control arm of type ControlArm
        :param r: rocker of type Rocker
        :param a: arb of type Arb
        :param s: shock of type Shock
        :param k: spring constant in N/m of type float
        :param f_prod: initial force on push/pullrod at ride height in N of
                       type float
        """
        if lca.side != uca.side or uca.side != r.side:
            raise ValueError('side mismatch')
        if lca.pr is None and uca.pr is None:
            raise ValueError('missing prod hardpoint')
        self.lca = lca
        self.uca = uca
        self.rocker = r
        self.arb = a
        if lca.pr is None:
            self.p_ca = uca
            self.p_len = r.get_pr_p().subtract(uca.get_pr_p()).norm()
        else:
            self.p_ca = lca
            self.p_len = r.get_pr_p().subtract(lca.get_pr_p()).norm()
        if s is None:
            self.shock = self.__make_shock(k, f_prod)
        else:
            self.shock = s
        self.ca_len = lca.get_u_p().subtract(uca.get_u_p()).norm()
        self.link_len = r.get_a_p().subtract(a.get_link_p()).norm()
        self.side = lca.side

    def __make_shock(self, constant: float, f_prod: float) -> Shock:
        """
        Make shock for QuarterSusp object
        :param constant: spring constant in N/m of type float
        :param f_prod: initial force on push/pullrod at ride height in N of
                       type float, positive is compression
        :return: shock for QuarterSusp object
        :rtype: Shock
        """
        # moment balance to find shock force
        length = self.rocker.get_s_p().norm()
        r_l_p = self.rocker.frame.get_rotation() ** -1
        l_prod = r_l_p * self.rocker.get_pr_p().subtract(self.p_ca.get_pr_p()).v
        n_prod = Vector(l_prod).unit()
        l_shock = r_l_p * self.rocker.get_s_p().v
        n_shock = Vector(l_shock).unit()
        m_prod = self.rocker.pr.cross(n_prod)
        m_shock = self.rocker.s.cross(n_shock)
        f_shock = -f_prod * m_prod.v[2] / m_shock.v[2]
        return Shock(constant, length, f_shock)

    def rotate_lca(self, a: float, actual: bool = False, deg: bool = True):
        """
        Rotates quarter suspension by rotating the lca
        :param a: angle of rotation
        :param actual: true if actual rotation, false if upwards rotation
        :param deg: true if angle is in degrees, false if angle is in radians
        :return: rotates quarter suspension, returns angles of rotation in the
                 order lca, uca, rocker, arb
        """
        x = Symbol('x')
        self.lca.rotate(a, actual, deg)

        # solve for uca rotation
        uca_l_n = make_rotation(x, self.uca.axis(), deg=False)
        uca_p_l = self.uca.frame.get_rotation()
        uca_p_n = uca_p_l * uca_l_n
        uca_up_pos = self.uca.get_f_p().v + uca_p_n * self.uca.u.v
        ca_dist = uca_up_pos - self.lca.get_u_p().v
        ca_length = self.ca_len ** 2
        ca_eqn = ca_dist[0] ** 2 + ca_dist[1] ** 2 + ca_dist[2] ** 2 - ca_length
        uca_sol = float(nsolve(ca_eqn, x, 0.0))
        self.uca.rotate(uca_sol, actual=True, deg=False)

        # solve for rocker rotation
        rocker_l_n = make_rotation(x, self.rocker.axis(), deg=False)
        rocker_p_l = self.rocker.frame.get_rotation()
        rocker_p_n = rocker_p_l * rocker_l_n
        pr_pos = self.rocker.get_pi_p().v + rocker_p_n * self.rocker.pr.v
        pr_dist = pr_pos - self.p_ca.get_pr_p().v
        pr_length = self.p_len ** 2
        r_eqn = pr_dist[0] ** 2 + pr_dist[1] ** 2 + pr_dist[2] ** 2 - pr_length
        r_sol = float(nsolve(r_eqn, x, 0.0))
        self.rocker.rotate(r_sol, actual=True, deg=False)

        # solve for arb rotation
        arb_l_n = make_rotation(x, self.arb.axis(), deg=False)
        arb_p_l = self.arb.frame.get_rotation()
        arb_p_n = arb_p_l * arb_l_n
        link_pos = self.arb.get_pi_p().v + arb_p_n * self.arb.link.v
        l_dist = link_pos - self.rocker.get_a_p().v
        l_length = self.link_len ** 2
        l_eqn = l_dist[0] ** 2 + l_dist[1] ** 2 + l_dist[2] ** 2 - l_length
        l_sol = float(nsolve(l_eqn, x, 0.0))
        self.arb.rotate(l_sol, actual=True, deg=False)

    def get_forces(self, longitudinal: bool = True):
        """
        Calculates forces on rocker
        :param longitudinal: true if acceleration is longitudinal, false if
                             acceleration is lateral
        :return: compressive force on rocker in order shock, arb, prod
        :rtype: tuple
        """
        # find force on arb link, positive is tension, negative is compression
        if longitudinal:
            f_arb = 0.0
        else:
            r_arb = self.arb.link
            f_arb = self.rocker.frame.get_coords(self.rocker.a, self.arb.frame)
            d_arb = f_arb.subtract(r_arb).unit()
            t_arb = self.arb.get_torque()
            f_arb = t_arb / (r_arb.cross(d_arb).v[1])
        # moment balance to find forces
        r_l_p = self.rocker.frame.get_rotation() ** -1
        s_len = self.rocker.get_s_p().norm()
        f_shock = self.shock.get_force(s_len)
        l_shock = Vector(r_l_p * self.rocker.get_s_p().v)
        n_shock = l_shock.unit()
        l_arb = self.rocker.get_a_p().subtract(self.arb.get_link_p()).unit()
        n_arb = Vector(r_l_p * l_arb.v)
        l_prod = self.rocker.get_pr_p().subtract(self.p_ca.get_pr_p()).unit()
        n_prod = Vector(r_l_p * l_prod.v)
        m_shock = self.rocker.s.cross(n_shock)
        m_arb = self.rocker.a.cross(n_arb)
        m_prod = self.rocker.pr.cross(n_prod)
        f_prod = (f_arb * m_arb.v[2] - f_shock * m_shock.v[2]) / m_prod.v[2]
        return f_shock, -f_arb, f_prod