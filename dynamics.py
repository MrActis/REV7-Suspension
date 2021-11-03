"""
Dynamics module
Contains classes and functions for suspension dynamics
Classes: Vector, CoordSys
Functions: subtract_v, cross_v, make_rotation
"""
from sympy import ImmutableMatrix, Symbol, shape, sin, cos
from math import sqrt, radians


class Vector:
    """
    Vector class
    Used to store a 3D vector
    Attributes:
        v : ImmutableMatrix
            3D column vector
    Methods:
        __init__(v)
        get()
        norm()
        unit()
    """
    def __init__(self, v: ImmutableMatrix):
        """
        Vector class constructor
        :param v: 3D vector of type ImmutableMatrix
        :raises: :class:'ValueError': if not a 3D vector
        """
        if shape(v) != (3, 1):
            raise ValueError('not a 3D vector')
        self.__v = v

    def get(self) -> ImmutableMatrix:
        """
        Getter for Vector class
        :return: immutable 3D vector
        :rtype: ImmutableMatrix
        """
        return self.__v

    def norm(self) -> float:
        """
        Calculates vector length
        :return: vector norm
        :rtype: float
        """
        return sqrt(self.__v.dot(self.__v))

    def unit(self):
        """
        Calculates unit vector
        :return: unit vector
        :rtype: Vector
        """
        return Vector((1 / self.__v.norm()) * self.__v)

    def add(self, v):
        """
        Adds two Vectors
        :param v: vector to add of type Vector
        :return: self vector plus input vector
        :rtype: Vector
        """
        if not isinstance(v, Vector):
            raise ValueError('not a vector')
        else:
            return Vector(self.get() + v.get())

    def subtract(self, v):
        """
        Subtracts two Vectors
        :param v: vector to subtract of type Vector
        :return: self vector minus input vector
        :rtype: Vector
        """
        if not isinstance(v, Vector):
            raise ValueError('not a vector')
        else:
            return Vector(self.get() - v.get())

    def cross(self, v):
        """
        Cross product of two Vectors
        :param v: vector of type Vector
        :return: self vector cross input vector
        :rtype: Vector
        """
        if not isinstance(v, Vector):
            raise ValueError('not a vector')
        else:
            return Vector(self.get().cross(v.get()))


class CoordSys:
    """
    CoordSys class
    Defines a 3D coordinate system
    Attributes:
        origin : Vector
            origin of coordinate system with respect to parent's origin, in
            parent coordinates
        r_p_l : ImmutableMatrix
            rotation matrix from self's coordinate system to parent
        parent : CoordSys
            parent coordinate system
    Methods:
        __init__(o, p, r)
        get_origin()
        get_parent()
        get_rotation()
        set_rotation(r_new)
        get_coords(v, sys)
    """
    def __init__(self, o: Vector, p = None, r: ImmutableMatrix = None):
        """
        CoordSys class constructor
        :param o: origin in parent coordinates of type Vector
        :param p: parent of type CoordSys
        :param r: rotation matrix to parent of type ImmutableMatrix
        :raises: :class:'ValueError': if p is not of type CoordSys
        :raises: :class:'ValueError': if r is not a valid rotation matrix
        """
        if p is not None and not isinstance(p, CoordSys):
            raise ValueError('invalid parent')
        if p is not None and r is None:
            raise ValueError('invalid rotation matrix')
        if p is None:
            self.__origin = Vector(ImmutableMatrix([0.0, 0.0, 0.0]))
            self.__r_p_l = None
        else:
            self.__origin = o
            self.__r_p_l = r
        self.__parent = p

    def get_origin(self) -> Vector:
        """
        Getter for origin
        :return: origin of coordinate system
        :rtype: Vector
        """
        return self.__origin

    def get_parent(self):
        """
        Getter for parent
        :return: parent of coordinate system
        :rtype: CoordSys
        :raises: :class:'ValueError': if there is no parent
        """
        if self.__parent is None:
            raise ValueError('no parent')
        else:
            return self.__parent

    def get_rotation(self) -> ImmutableMatrix:
        """
        Getter for rotation matrix
        :return: rotation matrix from self's coordinate system to parent
        :rtype: ImmutableMatrix
        :raises: :class:'ValueError': if there is no rotation matrix
        """
        if self.__r_p_l is None:
            raise ValueError('no rotation matrix')
        else:
            return self.__r_p_l

    def set_rotation(self, r_new: ImmutableMatrix):
        """
        Setter for rotation matrix
        :param r_new: new rotation matrix from self's coordinate system to
                      parent of type ImmutableMatrix
        :return: updates the rotation matrix
        """
        self.__r_p_l = r_new

    def get_coords(self, v: Vector, sys) -> Vector:
        """
        Converts coordinate from self's coordinate system to an ancestor's
        coordinate system
        :param v: a coordinate in self's coordinate system of type Vector
        :param sys: coordinate system to convert to of type CoordSys
        :return: coordinate in new coordinate system
        :rtype: Vector
        :raises: :class:'ValueError': if sys is not an ancestor
        """
        if self.__parent is None and sys != self:
            raise ValueError('cannot transform coordinates')
        if sys == self:
            return v
        else:
            new_coords = self.__origin.get() + self.__r_p_l * v.get()
            return self.__parent.get_coords(Vector(new_coords), sys)


def make_rotation(a, axis: str, deg: bool = True) -> ImmutableMatrix:
    """
    Calculates the rotation matrix for a rotation around a given axis
    :param a: angle of rotation of type float or Symbol
    :param axis: axis of rotation of type str
    :param deg: true if angle is in degrees, false if angle is in radians
    :return: rotation matrix
    :rtype: ImmutableMatrix
    :raises: :class:'ValueError': a is not of type float or Symbol
    :raises: :class:'ValueError': if axis is invalid
    """
    if not isinstance(a, float) and not isinstance(a, Symbol):
        raise ValueError('invalid angle, try using float')
    if deg:
        theta = radians(a)
    else:
        theta = a
    ax = axis.lower()
    if ax == 'x':
        return ImmutableMatrix([[1.0, 0.0, 0.0],
                                [0.0, cos(theta), -sin(theta)],
                                [0.0, sin(theta), cos(theta)]])
    elif ax == 'y':
        return ImmutableMatrix([[cos(theta), 0.0, sin(theta)],
                                [0.0, 1.0, 0.0],
                                [-sin(theta), 0.0, cos(theta)]])
    elif ax == 'z':
        return ImmutableMatrix([[cos(theta), -sin(theta), 0.0],
                                [sin(theta), cos(theta), 0.0],
                                [0.0, 0.0, 1.0]])
    else:
        raise ValueError('invalid axis')