from enum import *

meter_scale = 100

class EntityType(Enum):
  ROCK     = 'rock',
  SAMPLE   = 'sample',
  WALL     = 'wall',
  OBSTACLE = 'obstacle',
  LANDER   = 'lander',
  ROVER    = 'rover',
  EXPLORE  = 'explore'

class EntityProperties:
  def __init__(self, size, inflateAmount, collidable, colour):
    self.__size          = size
    self.__collidable    = collidable
    self.__colour        = colour
    self.__inflateAmount = inflateAmount

  def size(self):
    return self.__size

  def inflation(self):
    return self.__inflateAmount

  def collidable(self):
    return self.__collidable

  def colour(self):
    return self.__colour

entity_info = {
  EntityType.LANDER:   EntityProperties( 60, 1.5, True,  (  1,   1, 0.3, 1)),
  EntityType.SAMPLE:   EntityProperties(  5,   4, True,  (  1, 0.5, 0.1, 1)),
  EntityType.ROCK:     EntityProperties( 15,   2, True,  (0.3, 0.3,   1, 1)),
  EntityType.ROVER:    EntityProperties( 20,   1, True,  (  1,   1,   1, 1)),
  EntityType.OBSTACLE: EntityProperties( 30,   1, True,  (0.3,   1, 0.3, 1)),
  EntityType.WALL:     EntityProperties(200,   1, True,  (  1,   1,   1, 1)),
  EntityType.EXPLORE:  EntityProperties( 10,   1, False, (  1,   0,   0, 1)),
}