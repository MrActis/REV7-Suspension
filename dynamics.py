"""
Dynamics module
Contains classes and functions for suspension dynamics
Classes: Vector, RefFrame
Functions: make_rotation
"""
from sympy import ImmutableMatrix, Symbol, shape, sin, cos
from math import sqrt, acos, radians


class Vector:
    """
    Vector class
    Represents a 3D vector
    Attributes:
        v : ImmutableMatrix
            3D column vector
    Methods:
        __init__(vec)
        norm()
        unit()
        add(vec)
        subtract(vec)
        dot(vec)
        cross(vec)
        angle(vec)
    """
    def __init__(self, vec: ImmutableMatrix):
        """
        Vector class constructor
        :param v: 3D vector of type ImmutableMatrix
        :raises: :class:'ValueError': if not a 3D vector
        """
        if shape(vec) != (3, 1):
            raise ValueError('not a 3D vector')
        self.v = vec

    def norm(self) -> float:
        """
        Calculates vector length
        :return: vector norm
        :rtype: float
        """
        return sqrt(self.v.dot(self.v))

    def unit(self):
        """
        Calculates unit vector
        :return: unit vector
        :rtype: Vector
        """
        return Vector((1.0 / self.norm()) * self.v)

    def add(self, vec):
        """
        Adds two Vectors
        :param vec: vector to add of type Vector
        :return: self vector plus input vector
        :rtype: Vector
        :raises: :class:'ValueError': if input is not of type Vector
        """
        if not isinstance(vec, Vector):
            raise ValueError('input not a vector')
        else:
            return Vector(self.v + vec.v)

    def subtract(self, vec):
        """
        Subtracts two Vectors
        :param vec: vector to subtract of type Vector
        :return: self vector minus input vector
        :rtype: Vector
        :raises: :class:'ValueError': if input is not of type Vector
        """
        if not isinstance(vec, Vector):
            raise ValueError('input not a vector')
        else:
            return Vector(self.v - vec.v)

    def dot(self, vec) -> float:
        """
        Dot product of two Vectors
        :param vec: vector of type Vector
        :return: dot product of two vectors
        :rtype: float
        :raises: :class:'ValueError': if input is not of type Vector
        """
        if not isinstance(vec, Vector):
            raise ValueError('input not a vector')
        else:
            return self.v.dot(vec.v)

    def cross(self, vec):
        """
        Cross product of two Vectors
        :param vec: vector of type Vector
        :return: self vector cross input vector
        :rtype: Vector
        :raises: :class:'ValueError': if input is not of type Vector
        """
        if not isinstance(vec, Vector):
            raise ValueError('input not a vector')
        else:
            return Vector(self.v.cross(vec.v))

    def angle(self, vec) -> float:
        """
        Angle between two Vectors
        :param vec: vector of type Vector
        :return: angle between two vectors in radians
        :rtype: float
        :raises: :class:'ValueError': if input is not of type Vector
        """
        if not isinstance(vec, Vector):
            raise ValueError('input not a vector')
        else:
            return acos(self.dot(vec) / (self.norm() * vec.norm()))


class RefFrame:
    """
    RefFrame class
    Defines a reference frame
    Attributes:
        origin : Vector
            origin of reference frame with respect to parent's origin, in
            parent coordinates
        r_p_l : ImmutableMatrix
            rotation matrix from reference frame to parent
        parent : RefFrame
            parent reference frame
    Methods:
        __init__(o, p, r)
        get_origin()
        get_parent()
        get_rotation()
        set_rotation(r_new)
        get_coords(vec, frame)
    """
    def __init__(self, o: Vector, p = None, r: ImmutableMatrix = None):
        """
        RefFrame class constructor
        :param o: origin in parent coordinates of type Vector
        :param p: parent of type RefFrame
        :param r: rotation matrix to parent of type ImmutableMatrix
        :raises: :class:'ValueError': if p is not of type RefFrame
        :raises: :class:'ValueError': if r is not a valid rotation matrix
        """
        if p is not None and not isinstance(p, RefFrame):
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
        :return: origin of reference frame
        :rtype: Vector
        """
        return self.__origin

    def get_parent(self):
        """
        Getter for parent
        :return: parent reference frame
        :rtype: RefFrame
        :raises: :class:'ValueError': if there is no parent
        """
        if self.__parent is None:
            raise ValueError('no parent')
        else:
            return self.__parent

    def get_rotation(self) -> ImmutableMatrix:
        """
        Getter for rotation matrix
        :return: rotation matrix from reference frame to parent
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
        :param r_new: new rotation matrix from reference frame to parent of
                      type ImmutableMatrix
        :return: updates the rotation matrix
        """
        self.__r_p_l = r_new

    def get_coords(self, vec: Vector, frame) -> Vector:
        """
        Converts coordinate from reference frame to an ancestor's coordinate
        system
        :param vec: a coordinate in self's coordinate system of type Vector
        :param frame: coordinate system to convert to of type RefFrame
        :return: coordinate in new coordinate system
        :rtype: Vector
        :raises: :class:'ValueError': if frame is not an ancestor
        """
        if self.__parent is None and frame != self:
            raise ValueError('cannot transform coordinates')
        if frame == self:
            return vec
        else:
            new_coords = self.__origin.v + self.__r_p_l * vec.v
            return self.__parent.get_coords(Vector(new_coords), frame)


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