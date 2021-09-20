from enum import *

meter_scale = 100

class EntityType(Enum):
  ROCK     = 'rock',
  SAMPLE   = 'sample',
  WALL     = 'wall',
  OBSTACLE = 'obstacle',
  LANDER   = 'lander',
  ROVER    = 'rover',

class EntityProperties:
  def __init__(self, size, collidable, colour):
    self.__size       = size
    self.__collidable = collidable
    self.__colour     = colour

  def size(self):
    return self.__size

  def collidable(self):
    return self.__collidable

  def colour(self):
    return self.__colour

entity_info = {
  EntityType.LANDER:   EntityProperties(70,  True, (1,     1, 0.3, 1)),
  EntityType.SAMPLE:   EntityProperties(5,   True, (1,   0.5, 0.1, 1)),
  EntityType.ROCK:     EntityProperties(15,  True, (0.3, 0.3,   1, 1)),
  EntityType.ROVER:    EntityProperties(30,  True, (1,     1,   1, 1)),
  EntityType.OBSTACLE: EntityProperties(30,  True, (0.3,   1, 0.3, 1)),
  EntityType.WALL:     EntityProperties(200, True, (1,     1,   1, 1)),
}