
from vector_2d import Vector
from vector_2d import VectorPolar

from multipledispatch import dispatch
from math import * 

import sys

# Math functions
def sign(x): return 1 if x > 0 else -1

# Additional useful Vec2 functions
def vec2_round(a:Vector):           return Vector(round(a.x), round(a.y))
def vec2_dot(a:Vector, b:Vector):   return a.x * b.x + a.y * b.y
def vec2_mag_sqr(a:Vector):         return vec2_dot(a, a)
def vec2_max(a:Vector, b:Vector):   return Vector(x=max(a.x, b.x), y=max(a.y, b.y))
def vec2_min(a:Vector, b:Vector):   return Vector(x=min(a.x, b.x), y=min(a.y, b.y))
def vec2_cross(a:Vector, b:Vector): return a.x * b.y - b.x * a.y
def vec2_clamp(a:Vector, min_vec:Vector, max_vec:Vector): return vec2_max(vec2_min(a, max_vec), min_vec)
# Some helpers for representing 2D geometry

class Rect:
  def __init__(self, position: Vector, size: Vector, rotation = 0):
    '''
    Construct a 2D Rectangle from position and size vectors.
    '''
    self.min  = position - size * 0.5
    self.max  = position + size * 0.5
    self.rotation = rotation

  @staticmethod
  def smallest():
    rect = Rect(Vector(), Vector())
    min_val = -sys.float_info.max
    max_val = sys.float_info.max
    rect.max = Vector(min_val, min_val)
    rect.min = Vector(max_val, max_val)
    return rect

  def scale(self, factor):
    self.min = self.min * factor
    self.max = self.max * factor

  def size(self):
    return self.max - self.min

  def half_size(self):
    return self.size() * 0.5

  def closest_point(self, point:Vector):
    return vec2_clamp(point, self.min, self.max)

  def contains_point(self, point:Vector):
    return self.closest_point(point) == point

  def grow_to_contain(self, extents):
    self.min = vec2_min(extents.min, self.min)
    self.max = vec2_max(extents.max, self.max)

  def extents(self):
    return self

class Circle:
  def __init__(self, position:Vector, radius, rotation = 0):
    self.position = position
    self.rotation = rotation
    self.radius   = radius

  def rad_sqr(self):
    return self.radius * self.radius

  def closest_point(self, point:Vector):
    to_point = point - self.position
    len_sqr = vec2_mag_sqr(to_point)
    return self.position + sqrt(min(len_sqr, self.radius)) * to_point.unit()

  def contains_point(self, point:Vector):
    return vec2_mag_sqr(point - self.position) <= self.rad_sqr()

  def distance(self, point:Vector):
    return sqrt(self.distance_sqr(point))

  def distance_sqr(self, point:Vector):
    return vec2_mag_sqr(point - self.position)

  def extents(self):
    return Rect(self.position, Vector(self.radius * 2, self.radius * 2))

@dispatch(Rect, Rect)
def intersect(a, b):
  a_min = a.min()
  a_max = a.max()
  b_min = b.min()
  b_max = b.max()
  return a_max.x > b_min.x and a_max.y > b_min.y and b_max.y > a_min.y and b_max.x > a_min.x

@dispatch(Circle, Circle)
def intersect(a, b):
  return abs(a.position - b.position) <= a.radius + b.radius

@dispatch(Rect, Circle)
def intersect(a, b):
  return b.contains_point(a.closest_point(b.position))

@dispatch(Circle, Rect)
def intersect(a, b):
  return intersect(b, a)
