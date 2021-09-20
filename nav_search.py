from environment import Entity
from env_params import EntityType
from env_params import entity_info
from copy         import *
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
  def __init__(self, routine, prev_node, position):
    self.__position   = position
    self.__prev_node  = prev_node
    self.__routine    = routine
    if prev_node is not None:
      self.__path_len = prev_node.__path_len + 1
    else:
      self.__path_len = 1

  def cost(self):
    '''
    Calculate the cost of traversing to this Node given the path taken.
    '''
    return self.__routine.calculate_cost(self)

  def is_goal(self):
    '''
    Test if this PathNode is the goal.
    '''
    return self.__routine.is_goal(self)

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
    neighbours = []
    for dir in step_directions:
      new_node = NavNode(self.__routine, self, self.__position + dir * step_size)
      if self.__routine.can_enter(new_node):
        neighbours.append(new_node)
    return neighbours

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
    self.goal_dist = 0

  def calculate_cost(self, node:NavNode):
    return node.path_length() + vec2_mag_sqr(self.target - node.get_position())

  def can_enter(self, node:NavNode):
    rover = deepcopy(self.env.get_rover())
    rover.set_position(node.get_position())
    return self.env.find_first_colliding(rover, [EntityType.OBSTACLE, EntityType.ROCK]) is None

  def is_goal(self, node):
    return vec2_mag_sqr(node.get_position() - self.target) <= (self.goal_dist * self.goal_dist)

  def on_update(self, dt):
    rover       = self.navigator().get_rover_entity()
    start_node  = NavNode(self, None, rover.position())
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

  def get_path(self):
    return self.path

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

  def calculate_cost(self, node):
    return super().calculate_cost(node)

  def is_goal(self, node):
    return super().is_goal(node)

  def on_start(self):
    rover_pos   = self.navigator().get_rover_entity().position()
    self.target = None
    while self.target is None:
      angle       = random.random() * 2 - 1
      distance    = random.random() * 50
      target_pos  = rover_pos + VectorPolar(distance, angle * math.pi).to_cartesian()

      if abs(target_pos.x) > 80 or abs(target_pos.y) > 80:
        continue

      rover = deepcopy(self.env.get_rover())
      rover.set_position(target_pos)
      if self.env.find_first_colliding(rover) is None:
        self.target = target_pos

    self.goal_dist = (entity_info[EntityType.ROVER].size() + entity_info[EntityType.ROCK].size()) / 2

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.EXPLORE

  def is_done(self):
    rover = deepcopy(self.env.get_rover())
    rover.set_position(self.target)

    if self.env.has_entities(EntityType.SAMPLE):
      return True

    return self.env.find_first_colliding(rover) is not None
    
class LanderSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)

  def calculate_cost(self, node):
    return super().calculate_cost(node)

  def on_start(self):
    rover_pos   = self.navigator().get_rover_entity().position()
    # lander      = self.env.find_closest(EntityType.LANDER, rover_pos)
    self.target = Vector(0,0) # lander.position()
    self.goal_dist = entity_info[EntityType.ROVER].size()

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_LANDER

class SampleSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.sample = None

  def calculate_cost(self, node):
    cost = 0
    lander_intesect = self.env.find_first_colliding(self.env.get_rover(), EntityType.LANDER)
    if lander_intesect is not None:
      cost = vec2_mag_sqr(lander_intesect.position() - self.env.get_rover().position())
    return cost + super().calculate_cost(node)

  def on_start(self):
    rover_pos         = self.navigator().get_rover_entity().position()
    self.sample, dist = self.env.find_closest(EntityType.SAMPLE, rover_pos)
    self.target       = self.sample.position()
    self.goal_dist    = (entity_info[EntityType.ROVER].size() + entity_info[EntityType.SAMPLE].size()) / 2

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

  def calculate_cost(self, node):
    cost = 0
    rover = copy(self.env.get_rover())
    rover.set_position(self.get_position())
    lander_intesect = self.env.find_first_colliding(rover, EntityType.LANDER)
    if lander_intesect is not None:
      cost = vec2_mag_sqr(lander_intesect.position() - self.env.get_rover().position())
    return cost + super().calculate_cost(node)

  def can_enter(self, node: NavNode):
    rover = deepcopy(self.env.get_rover())
    rover.set_position(node.get_position())
    return self.env.find_first_colliding(rover, [EntityType.OBSTACLE, EntityType.SAMPLE]) is None

  def on_start(self):
    rover_pos       = self.navigator().get_rover_entity().position()
    self.rock, dist = self.env.find_closest(EntityType.ROCK, rover_pos)
    self.target     = self.rock.position()
    self.goal_dist = (entity_info[EntityType.ROVER].size() + entity_info[EntityType.ROCK].size()) / 2

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
