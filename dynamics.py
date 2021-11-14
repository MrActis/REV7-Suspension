"""
Dynamics module
Contains classes and functions for suspension dynamics
Classes: Vector, RefFrame
Functions: make_rotation
"""
from sympy import ImmutableMatrix, Symbol, shape, sin, cos
from math import sqrt, acos, radians, degrees
from collections import deque


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
        angle(vec, deg)
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

    def angle(self, vec, deg: bool = True) -> float:
        """
        Angle between two Vectors
        :param vec: vector of type Vector
        :param deg: true if angle in degrees, false if angle in radians
        :return: angle between two vectors
        :rtype: float
        :raises: :class:'ValueError': if input is not of type Vector
        """
        if not isinstance(vec, Vector):
            raise ValueError('input not a vector')
        else:
            angle = acos(self.dot(vec) / (self.norm() * vec.norm()))
            if deg:
                return degrees(angle)
            else:
                return angle


class RefFrame:
    """
    RefFrame class
    Defines a reference frame
    Attributes:
        vertex : int
            integer that represents the reference frame
        origin : Vector
            origin of reference frame with respect to parent's origin, in
            parent coordinates
        r_p_l : ImmutableMatrix
            rotation matrix from reference frame to parent
    Class Attributes:
        vertices : dict
            maps vertex to reference frame
        graph : dict
            a graph of the relationships between reference frames
        parents : dict
            stores the parents of reference frames
    Methods:
        __init__(o, p, r)
        get_key()
        get_origin()
        get_parent()
        get_rotation()
        set_rotation(r_new)
        find_path(frame)
        get_coords(vec, frame)
        get_vector(vec, frame)
    """
    __vertices : dict = {}
    __graph : dict = {}
    __parents : dict = {}

    def __init__(self, o: Vector = None, p = None, r: ImmutableMatrix = None):
        """
        RefFrame class constructor
        :param o: origin in parent coordinates of type Vector
        :param p: parent of type RefFrame
        :param r: rotation matrix to parent of type ImmutableMatrix
        :raises: :class:'ValueError': if p is not of type RefFrame
        :raises: :class:'ValueError': if p does not exist
        :raises: :class:'ValueError': if r is not a valid rotation matrix
        """
        if p is not None:
            if not isinstance(p, RefFrame):
                raise ValueError('invalid parent type')
            elif p.get_key() not in RefFrame.__graph:
                raise ValueError('parent does not exist')
            elif r is None:
                raise ValueError('invalid rotation matrix')
        self.__vertex = len(RefFrame.__vertices)
        if p is None:
            self.__origin = Vector(ImmutableMatrix([0.0, 0.0, 0.0]))
            self.__r_p_l = None
            RefFrame.__graph[self.__vertex] = []
            RefFrame.__parents[self.__vertex] = -1
        else:
            self.__origin = o
            self.__r_p_l = r
            k = p.get_key()
            RefFrame.__graph[self.__vertex] = [k]
            RefFrame.__graph.get(k).append(self.__vertex)
            RefFrame.__parents[self.__vertex] = k
        RefFrame.__vertices[self.__vertex] = self

    def get_key(self) -> int:
        """
        Getter for key/vertex
        :return: return key/vertex for reference frame
        """
        return self.__vertex

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
        parent = RefFrame.__parents.get(self.get_key())
        if parent == -1:
            raise ValueError('no parent')
        else:
            return RefFrame.__vertices.get(parent)

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

    def find_path(self, frame) -> list:
        """
        Determines shortest path between reference frames, if a path exists
        :param frame: target reference frame of type RefFrame
        :return: list of reference frames in path from self to frame
        :raises: :class:'ValueError': if frame is not of type RefFrame
        :raises: :class:'ValueError': if there is no path
        """
        if not isinstance(frame, RefFrame):
            raise ValueError('invalid reference frame')
        # BFS
        source = self.get_key()
        target = frame.get_key()
        discovered = [False] * len(self.__vertices)
        ancestor = [-1] * len(self.__vertices)
        q = deque()
        q.append(source)
        discovered[source] = True
        while len(q) != 0:
            v = q.popleft()
            for u in self.__graph.get(v):
                if not discovered[u]:
                    discovered[u] = True
                    q.append(u)
                    ancestor[u] = v
            if discovered[target]:
                break
        if not discovered[target]:
            raise ValueError('no path exists')
        else:
            path = deque()
            curr = target
            while curr != source:
                path.appendleft(curr)
                curr = ancestor[curr]
            path.appendleft(source)
            return list(path)

    def get_coords(self, vec: Vector, frame) -> Vector:
        """
        Converts coordinate from reference frame to other coordinate system
        :param vec: a coordinate in self's coordinate system of type Vector
        :param frame: coordinate system to convert to of type RefFrame
        :return: coordinate in other coordinate system
        :rtype: Vector
        :raises: :class:'ValueError': if frame is not of type RefFrame
        :raises: :class:'ValueError': if conversion is impossible
        """
        path = self.find_path(frame)
        coord = vec.v
        for i in range(len(path) - 1):
            curr = path[i]
            new = path[i + 1]
            if new == RefFrame.__parents.get(curr):
                f = RefFrame.__vertices.get(curr)
                coord = f.get_origin().v + f.get_rotation() * coord
            else:
                f = RefFrame.__vertices.get(new)
                coord = (f.get_rotation() ** -1) * (coord - f.get_origin().v)
        return Vector(coord)

    def get_vector(self, vec: Vector, frame) -> Vector:
        """
        Converts vector from reference frame to other frame
        :param vec: a vector in self's coordinate system of type Vector
        :param frame: frame to convert to of type RefFrame
        :return: vector in other frame
        :rtype: Vector
        :raises: :class:'ValueError': if frame is not of type RefFrame
        :raises: :class:'ValueError': if conversion is impossible
        """
        path = self.find_path(frame)
        vector = vec.v
        for i in range(len(path) - 1):
            curr = path[i]
            new = path[i + 1]
            if new == RefFrame.__parents.get(curr):
                f = RefFrame.__vertices.get(curr)
                vector = f.get_rotation() * vector
            else:
                f = RefFrame.__vertices.get(new)
                vector = (f.get_rotation() ** -1) * vector
        return Vector(vector)


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