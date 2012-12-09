# Tread - a track editor for Vamos Automotive Simulator
# Copyright (C) 2010  Sam Varner
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import math
import unittest

full_circle = 2.0 * math.pi

def distance (p1, p2):
    delta = p2 - p1
    return math.sqrt (delta.x**2 + delta.y**2)

class Point:
    def __init__ (self, x, y):
        self.x = x
        self.y = y
    def __add__ (self, other):
        return Point (self.x + other.x, self.y + other.y)
    def __sub__ (self, other):
        return Point (self.x - other.x, self.y - other.y)
    def __mul__ (self, number):
        return Point (self.x * number, self.y * number)
    def __div__ (self, number):
        return self * (1.0 / number)
    def __iadd__ (self, other):
        self = self + other;
        return self
    def __isub__ (self, other):
        self = self - other;
        return self
    def __abs__ (self):
        return distance (Point (0, 0), self)
    def __eq__ (self, other):
        return self.x == other.x and self.y == other.y

class Circle:
    def __init__ (self, center = Point (0, 0), edge = Point (0, 0)):
        self.center = center
        self.edge = edge
    def move_to (self, new_center):
        delta = new_center - self.center
        self.center += delta
        self.edge += delta
    def set_center (self, p):
        self.center = p
    def set_edge (self, p):
        self.edge = p
    def radius (self):
        return distance (self.center, self.edge)
    def distance (self, p):
        return distance (p, self.center)
    def edge_distance (self, p):
        return distance (p, self.edge)

### Tests

class Test_Point (unittest.TestCase):
    def setUp (self):
        self.p1 = Point (1, 2)
        self.p2 = Point (3, -3)

    def test_access (self):
        self.assertEqual (self.p1.x, 1)
        self.assertEqual (self.p1.y, 2)
        self.p1.x = 6
        self.p1.y = 7
        self.assertEqual (self.p1.x, 6)
        self.assertEqual (self.p1.y, 7)

    def test_add (self):
        p = self.p1 + self.p2
        self.assertEqual (p.x, 4)
        self.assertEqual (p.y, -1)
        p += self.p2
        self.assertEqual (p.x, 7)
        self.assertEqual (p.y, -4)

    def test_subtract (self):
        p = self.p1 - self.p2
        self.assertEqual (p.x, -2)
        self.assertEqual (p.y, 5)
        p -= self.p2
        self.assertEqual (p.x, -5)
        self.assertEqual (p.y, 8)

    def test_multiply (self):
        p = self.p1 * 2
        self.assertEqual (p.x, 2)
        self.assertEqual (p.y, 4)

    def test_divide (self):
        p = self.p1 / 2
        self.assertEqual (p.x, 0.5)
        self.assertEqual (p.y, 1)

    def test_abs (self):
        self.assertEqual (abs (self.p1), math.sqrt (5))

    def test_equality (self):
        self.assertFalse (self.p1 == self.p2)
        p3 = self.p1
        self.assertTrue (self.p1 == p3)

class Test_Circle (unittest.TestCase):
    def setUp (self):
        self.c = Circle (Point (5, 4), Point (2, 0))

    def test_access (self):
        self.assertEqual (self.c.center, Point (5, 4))
        self.assertEqual (self.c.edge, Point (2, 0))

    def test_move (self):
        p = Point (3, 2)
        c = self.c.move_to (p)
        self.assertEqual (self.c.center, p)
        self.assertEqual (self.c.edge, Point (0, -2))

    def test_set_center (self):
        p = Point (3, 2)
        c = self.c.set_center (p)
        self.assertEqual (self.c.center, p)
        self.assertEqual (self.c.edge, Point (2, 0))

    def test_set_edge (self):
        p = Point (3, 2)
        c = self.c.set_edge (p)
        self.assertEqual (self.c.center, Point (5, 4))
        self.assertEqual (self.c.edge, p)

    def test_radius (self):
        self.assertEqual (self.c.radius (), 5)

    def test_distance (self):
        self.assertEqual (self.c.distance (Point (-3, -2)), 10)
        self.assertEqual (self.c.edge_distance (Point (1, 1)), math.sqrt (2))


point_suite = unittest.TestLoader().loadTestsFromTestCase (Test_Point)
circle_suite = unittest.TestLoader().loadTestsFromTestCase (Test_Circle)
suite = unittest.TestSuite ([point_suite, circle_suite])
