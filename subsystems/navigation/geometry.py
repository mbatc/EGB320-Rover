from numpy import square
from vector_2d import Vector
from vector_2d import VectorPolar

from multipledispatch import dispatch
from math import * 

import sys

EPS = 1e-7

# Math functions
def sign(x): return 1 if x > 0 else -1
def sq(x): return x * x

# Additional useful Vec2 functions
def vec2_round(a:Vector):           return Vector(round(a.x), round(a.y))
def vec2_dot(a:Vector, b:Vector):   return a.x * b.x + a.y * b.y
def vec2_mag_sqr(a:Vector):         return vec2_dot(a, a)
def vec2_max(a:Vector, b:Vector):   return Vector(x=max(a.x, b.x), y=max(a.y, b.y))
def vec2_min(a:Vector, b:Vector):   return Vector(x=min(a.x, b.x), y=min(a.y, b.y))
def vec2_cross(a:Vector, b:Vector): return a.x * b.y - b.x * a.y
def vec2_clamp(a:Vector, min_vec:Vector, max_vec:Vector): return vec2_max(vec2_min(a, max_vec), min_vec)
def vec2_angle(a:Vector, b:Vector): return acos(max(-1, min(1, vec2_dot(a, b) / (abs(a) * abs(b)))))

class Body:
  def transformed(self, translation, rotation):
    raise Exception("Not implemented")

  def scale(self, amount):
    raise Exception("Not implemented")

  def min(self):
    raise Exception("Not implemented")

  def max(self):
    raise Exception("Not implemented")

  def extents(self):
    return Rect(self.center(), self.max() - self.min())
    
  def center(self):
    return (self.min() + self.max()) * 0.5

class Rect(Body):
  def __init__(self, position: Vector, size: Vector, rotation = 0):
    '''
    Construct a 2D Rectangle from position and size vectors.
    '''
    self.__min      = position - size * 0.5
    self.__max      = position + size * 0.5
    self.rotation = rotation

  def extents(self):
    '''
    The extents of the Rect is just itself.
    '''
    return self

  def transformed(self, translation, rotation):
    '''
    Return the transformed rect. Currently we only use translation.
    '''
    new_rect = Rect()
    new_rect.__min = self.__min + translation
    new_rect.__max = self.__max + translation

  def position(self):
    return self.center()

  def scale(self, amount):
    return Rect(self.position(), self.size() * amount)

  def size(self):
    return self.max() - self.min()

  def min(self):
    return self.__min

  def max(self):
    return self.__max

  def half_size(self):
    return self.size() * 0.5

  def closest_point(self, point:Vector):
    return vec2_clamp(point, self.min(), self.max())

  def contains_point(self, point:Vector):
    return self.closest_point(point) == point

  def grow_to_contain(self, body:Body):
    self.__min = vec2_min(body.min(), self.__min)
    self.__max = vec2_max(body.max(), self.__max)

  def is_smallest(self):
    return self.__min.x > self.__max.x or self.__min.y > self.__max.y

  @staticmethod
  def smallest():
    rect     = Rect(Vector(), Vector())
    min_val  = -sys.float_info.max
    max_val  = sys.float_info.max
    rect.__max = Vector(min_val, min_val)
    rect.__min = Vector(max_val, max_val)
    return rect

class Circle(Body):
  def __init__(self, position:Vector, radius, rotation = 0):
    self.__position = position
    self.__radius   = radius

  def position(self):
    return self.__position

  def radius(self):
    return self.__radius

  def extents(self):
    return Rect(self.position(), Vector(self.radius() * 2, self.radius() * 2))

  def transformed(self, translation, rotation):
    return Circle(self.__position + translation, self.__radius, 0)

  def scale(self, amount):
    return Circle(self.__position, self.__radius * amount)

  def rad_sqr(self):
    return self.__radius * self.__radius

  def center(self):
    return self.__position

  def min(self):
    return self.__position - Vector(self.__radius, self.__radius)

  def max(self):
    return self.__position + Vector(self.__radius, self.__radius)

  def closest_point(self, point:Vector):
    to_point = point - self.position
    len_sqr = vec2_mag_sqr(to_point)
    return self.position + sqrt(min(len_sqr, self.__radius)) * to_point.unit()

  def contains_point(self, point:Vector):
    return vec2_mag_sqr(point - self.position) <= self.rad_sqr()

  def distance(self, point:Vector):
    return sqrt(self.distance_sqr(point))

  def distance_sqr(self, point:Vector):
    return vec2_mag_sqr(point - self.position)

class Line(Body):
  def __init__(self, start, end):
    self.__start = start
    self.__end   = end

  def extents(self):
    return Rect(self.position, Vector(self.radius * 2, self.radius * 2))

  def transformed(self, translation, rotation):
    return Line(self.__start + translation, self.__end + translation)

  def scale(self, amount):
    return Line(self.__start * amount, self.__end * amount)

  def start(self):
    return self.__start

  def end(self):
    return self.__end

  def point_at(self, t):
    return (1 - t) * self.__start + t * self.__end

  def length(self):
    return abs(self.__end - self.__start)

  def length_sqr(self):
    return vec2_mag_sqr(self.__end - self.__start)

  def min(self):
    return vec2_min(self.start(), self.end())

  def max(self):
    return vec2_max(self.start(), self.end())

@dispatch(Rect, Rect)
def intersect(a, b):
  a_min = a.min()
  a_max = a.max()
  b_min = b.min()
  b_max = b.max()
  return a_max.x > b_min.x and a_max.y > b_min.y and b_max.y > a_min.y and b_max.x > a_min.x

@dispatch(Rect, Circle)
def intersect(a, b):
  return b.contains_point(a.closest_point(b.position()))

@dispatch(Rect, Line)
def intersect(a, b):
  delta  = (b.end() - b.start())
  scaleX = 1.0 / delta.x
  scaleY = 1.0 / delta.y
  signX = sign(scaleX)
  signY = sign(scaleY)
  pos = a.position()
  half_size = a.half_size()
  nearTimeX = (pos.x - signX * half_size.x - pos.x) * scaleX
  nearTimeY = (pos.y - signY * half_size.y - pos.y) * scaleY
  farTimeX  = (pos.x + signX * half_size.x - pos.x) * scaleX
  farTimeY  = (pos.y + signY * half_size.y - pos.y) * scaleY

  if nearTimeX > farTimeY or nearTimeY > farTimeX:
    return False

  nearTime = nearTimeX if nearTimeX > nearTimeY else nearTimeY
  farTime  = farTimeX if farTimeX < farTimeY else farTimeY

  if nearTime >= 1 or farTime <= 0:
    return False
  return True

@dispatch(Circle, Circle)
def intersect(a, b):
  return abs(a.position() - b.position()) <= a.radius() + b.radius()

@dispatch(Circle, Rect)
def intersect(a, b):
  return intersect(b, a)

@dispatch(Circle, Line)
def intersect(a, b):
  enter_point, exit_point = calculate_intersections(b, a)
  return enter_point is not None or exit_point is not None

@dispatch(Line, Line)
def intersect(a, b):
  p0 = a.start()
  p1 = a.end()
  p2 = b.start()
  p3 = b.end()
  s1 = p1 - p0
  s2 = p3 - p1
  s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) / (-s2.x * s1.y + s1.x * s2.y)
  t = ( s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) / (-s2.x * s1.y + s1.x * s2.y)
  return s >= 0 and s <= 1 and t >= 0 and t <= 1

@dispatch(Line, Rect)
def intersect(a, b):
  return intersect(b, a)

@dispatch(Line, Circle)
def intersect(a, b):
  return intersect(b, a)

def calculate_intersections(segment, circle):
  f = segment.start() - circle.center()
  r = circle.radius()
  d = segment.end() - segment.start()

  if vec2_mag_sqr(d) < 0.0001:
    return None, None

  a = vec2_dot(d, d)
  b = 2 * vec2_dot(f, d)
  c = vec2_dot(f, f) - r * r

  discriminant = b * b - 4 * a * c
  if discriminant < 0:
    return None, None

  discriminant = sqrt(discriminant)
  t1 = (-b - discriminant) / (2 * a)
  t2 = (-b + discriminant) / (2 * a)

  enter = None
  exit  = None

  if t1 >= 0 and t1 <= 1:
    enter = segment.point_at(t1)
  if t2 >= 0 and t2 <= 1:
    exit  = segment.point_at(t2)

  return enter, exit
 