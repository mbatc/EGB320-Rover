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
  def __init__(self, size, collidable):
    self.__size       = size
    self.__collidable = collidable

  def size(self):
    return self.__size

  def collidable(self):
    return self.__collidable

entity_info = {
  EntityType.LANDER:   EntityProperties(30,  True),
  EntityType.SAMPLE:   EntityProperties(5,   True),
  EntityType.ROCK:     EntityProperties(10,  True),
  EntityType.ROVER:    EntityProperties(15,  True),
  EntityType.OBSTACLE: EntityProperties(20,  True),
  EntityType.WALL:     EntityProperties(200, True),
}