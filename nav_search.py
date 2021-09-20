from env_params import EntityType

from graph_search import *
from geometry     import *
from nav_routine  import *

import math
import random

step_directions = [
  Vector(0, 1),
  Vector(1, 1),
  Vector(1, 0),
  Vector(1, -1),
  Vector(0, -1),
  Vector(-1, -1),
  Vector(-1, 0),
  Vector(-1, 1),
]

step_size = 5 # cm

class NavNode:
  def __init__(self, prev_node, position, environment, target_pos):
    self.__env        = environment
    self.__position   = position
    self.__prev_node  = prev_node
    self.__target_pos = target_pos
    if prev_node is not None:
      self.__path_len = prev_node.__path_len + 1
    else:
      self.__path_len = 1

  def cost(self):
    '''
    Calculate the cost of traversing to this Node given the path taken.
    '''
    return self.__path_len + vec2_mag_sqr(self.__target_pos - self.get_position())

  def is_goal(self):
    '''
    Test if this PathNode is the goal.
    '''
    return vec2_mag_sqr(self.__position - self.__target_pos) < (4 * step_size * step_size)

  def prev_node(self):
    return self.__prev_node

  def path_length(self):
    return self.__path_len

  def path(self):
    '''
    Get the path to this node.
    '''
    path = [ None ] * self.__path_len
    next_node = self
    for i in range(self.__path_len):
      path[self.__path_len - i - 1] = next_node
      next_node = next_node.__prev_node
    return path

  def get_neighbours(self):
    '''
    Neighbours are all 
    '''
    return [ NavNode(self, self.__position + direction * step_size, self.__env, self.__target_pos) for direction in step_directions ]
    
  def get_position(self):
    return self.__position

  def __hash__(self):
    '''
    The derived class is required to implement a Hash function.
    '''
    return hash((self.__position, self.__path_len))

  def __eq__(self, other):
    '''
    The derived class is required to implement a Comparison function.
    '''
    if other is None:
      return False

    return self.__position == other.__position and self.__path_len == other.__path_len

class SearchRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.target = None
    self.env    = navigator.environment()
    self.failed = False
    self.path   = []

  def on_update(self, dt):
    rover       = self.navigator().get_rover_entity()
    start_node  = NavNode(None, rover.position(), self.env, self.target)
    path_finder = GraphSearch()
    path_finder.expand_frontier(start_node)
    goal = None
    # try:
    while goal is None:
      goal = path_finder.search()
    # except Exception as e:
      # Path find failed
      # print('Navigation Failed: '+ str(e))
      # self.failed = True

    if goal is not None:
      self.path = [ node.get_position() for node in goal.path() ]
    else:
      self.path = []

  def get_control_parameters(self):
    path_len = len(self.path)
    if path_len == 0:
      return 0, 0

    target_dir = None
    if path_len == 1:
      target_dir = self.target - self.path[0]
    else:
      target_dir = self.path[1] - self.path[0]

    return direction_to_control_param(target_dir, self.navigator().get_rover_entity())

  def is_done(self):
    '''
    This function should return True when the navigation
    routine has been completed.
    '''
    rover = self.navigator().get_rover_entity()
    return abs(self.target - rover.position()) < 5

class ExploreRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)

  def on_start(self):
    rover_pos   = self.navigator().get_rover_entity().position()
    angle       = random.random() * 2 - 1
    distance    = random.random() * 20
    target_pos  = rover_pos + VectorPolar(distance, angle * math.pi).to_cartesian()
    self.target = target_pos

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.EXPLORE

class LanderSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)

  def on_start(self):
    rover_pos   = self.navigator().get_rover_entity().position()
    lander      = self.env.find_closest(EntityType.LANDER, rover_pos)
    self.target = lander.position()

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_LANDER

class SampleSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.sample = None

  def on_start(self):
    rover_pos         = self.navigator().get_rover_entity().position()
    self.sample, dist = self.env.find_closest(EntityType.SAMPLE, rover_pos)
    self.target       = self.sample.position()

  def on_update(self, dt):
    if self.sample not in self.env:
      self.sample = None

    if self.sample is not None:
      self.target  = self.sample.position()
      super().on_update(dt)

  def is_done(self):
    return self.sample is None or super().is_done()

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_SAMPLE


class RockSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.rock = None

  def on_start(self):
    rover_pos       = self.navigator().get_rover_entity().position()
    self.rock, dist = self.env.find_closest(EntityType.ROCK, rover_pos)
    self.target     = self.rock.position()

  def on_update(self, dt):
    if self.rock not in self.env:
      self.rock = None

    if self.rock is not None:
      self.rock  = self.rock.position()
      super().on_update(dt)

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_ROCK
