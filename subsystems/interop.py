from vector_2d import *
from enum import Enum
from math import radians
from math import degrees

class SCS_ACTION(Enum):
  NONE           = 0,
  DROP_SAMPLE    = 1,
  FLIP_ROCK      = 2,
  COLLECT_SAMPLE = 3

class ObjectType(Enum):
  ROCK     = 'rock',
  SAMPLE   = 'sample',
  WALL     = 'wall',
  OBSTACLE = 'obstacle',
  LANDER   = 'lander',
  ROVER    = 'rover',
  EXPLORE  = 'explore'

class DetectedObject:
  def __init__(self, type, heading, distance, angle):
    self.__type          = type
    self.__heading       = heading
    self.__distance      = distance
    self.__angle         = angle
    self.__best_distance = 10
    self.__best_heading  = radians(5)
    self.missing_time  = 0

  def type(self):
    return self.__type

  def distance(self):
    return self.__distance

  def calculate_heading(self, rover):
    return rover.angle() + self.__heading

  def calculate_angle(self, rover):
    return rover.angle() + self.__angle

  def calculate_position(self, rover):
    return rover.position() + VectorPolar(self.distance(), self.calculate_heading(rover)).to_cartesian()

  def get_confidence(self):
    dist_confidence = max(0, min(1, self.__best_distance / self.__distance))
    head_confidence = max(0, min(1, abs(self.__best_heading) / abs(self.__heading)))
    return dist_confidence * head_confidence

  def __str__(self):
    return '(type:{}, angle: {}d, dist: {})'.format(self.__type, degrees(self.__heading), self.__distance)
