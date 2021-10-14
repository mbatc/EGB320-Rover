from vector_2d import *
from enum import Enum
from math import radians
from math import degrees

class SCS_ACTION(Enum):
  NONE                = 0,
  DROP_SAMPLE         = 1,
  FLIP_ROCK_PREP      = 2,
  FLIP_ROCK           = 3,
  COLLECT_SAMPLE_PREP = 4,
  COLLECT_SAMPLE      = 5,

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
    self.type          = type
    self.heading       = heading
    self.distance      = distance
    self.angle         = angle
    self.last_detected = 0

  def __str__(self):
    return '(type:{}, angle: {}d, dist: {})'.format(self.type, degrees(self.heading), self.distance)
