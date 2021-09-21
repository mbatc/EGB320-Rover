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
  (0, 1),
  (1, 1),
  (1, 0),
  (1, -1),
  (0, -1),
  (-1, -1),
  (-1, 0),
  (-1, 1),
]

step_size = 5 # cm

class NavNode:
  def __init__(self, routine, prev_node, position, tile_pos):
    # The position of the node within the environment
    self.__position   = position
    # The node before 'self' in the path
    self.__prev_node  = prev_node
    # The navigation routine that created this node.
    # The routine also implements parts of the search (cost calculation, neigbour exclusion)
    self.__routine    = routine
    # The tile position of the node in the discretized navigation space
    self.__tile_pos   = tile_pos

    # A set of all positions in the path.
    # We use this to make sure we don't cross the same position twice in the same path
    self.__path_pos   = set() if prev_node is None else prev_node.__path_pos
    self.__path_pos.add(tile_pos)

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
      # Calculate the tile position in the discrete space
      new_tile_pos = (int(self.__tile_pos[0] + dir[0]), int(self.__tile_pos[1] + dir[1]))
      # Check if we have entered this node before
      if new_tile_pos in self.__path_pos:
        continue

      # Calculate the new position in the environment
      new_pos      = self.__position + Vector(dir[0], dir[1]) * step_size

      # Outside the map (TODO: Update this so it is based on walls detected)
      if abs(new_pos.x) > 100 or abs(new_pos.y) > 100:
        continue

      new_node = NavNode(self.__routine, self, new_pos, new_tile_pos)
      if self.__routine.can_enter(new_node): # Check if we can enter, based on the current nav routine
        neighbours.append(new_node)
    return neighbours

  def get_position(self):
    return self.__position

  def __hash__(self):
    '''
    The derived class is required to implement a Hash function.
    '''
    return hash((self.__tile_pos, self.__path_len))

  def __eq__(self, other):
    '''
    The derived class is required to implement a Comparison function.
    '''
    if other is None:
      return False

    return self.__tile_pos == other.__tile_pos and self.__path_len == other.__path_len

class SearchRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.target = None
    self.env    = navigator.environment()
    self.failed = False
    self.path   = []
    self.goal_dist   = 5
    self.finish_dist = 5

  def calculate_cost(self, node:NavNode):
    return node.path_length() + vec2_mag_sqr(self.target - node.get_position())

  def can_enter(self, node:NavNode):
    return True

  def is_goal(self, node):
    return vec2_mag_sqr(node.get_position() - self.target) <= (self.goal_dist * self.goal_dist)

  def on_update(self, dt):
    rover       = self.navigator().get_rover_entity()
    start_node  = NavNode(self, None, rover.position(), (0, 0))
    path_finder = GraphSearch()
    path_finder.expand_frontier(start_node)
    goal = None

    try:
      while goal is None:
        goal = path_finder.search()
    except Exception as e:
      print('Path find Failed: ' + str(e))

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
    return abs(self.target - rover.position()) < self.finish_dist

class ExploreRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)

  def calculate_cost(self, node):
    return super().calculate_cost(node)

  def is_goal(self, node):
    return super().is_goal(node)

  def on_start(self):
    self.target = None
    rover_copy = deepcopy(self.navigator().get_rover_entity())
    while self.target is None:    
      angle       = random.random() * 2 - 1
      distance    = random.random() * 100
      target_pos  = self.navigator().rover_start_position() + VectorPolar(distance, angle * math.pi).to_cartesian()
  
      if abs(target_pos.x) > 80 or abs(target_pos.y) > 80:
        continue
      rover_copy.set_position(target_pos)
      if self.env.find_first_colliding(rover_copy) is None:
        self.target = target_pos

    self.goal_dist = entity_info[EntityType.ROVER].size() / 2
    self.finish_dist = self.goal_dist

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

  def calculate_cost(self, node: NavNode):
    cost = 0
    rover_copy = deepcopy(self.env.get_rover())
    rover_copy.set_position(node.get_position())
    for colliding in self.env.find_colliding(rover_copy, [ EntityType.SAMPLE, EntityType.OBSTACLE, EntityType.ROCK ]):
      cost = cost + vec2_mag_sqr(colliding.position() - rover_copy.position())
    return cost + super().calculate_cost(node)

  def can_enter(self, node: NavNode):
    return True

  def on_start(self):
    rover_pos      = self.navigator().get_rover_entity().position()
    lander, dist   = self.env.find_closest(EntityType.LANDER, rover_pos)
    self.target    = lander.position() if lander is not None else self.navigator().rover_start_position()
    self.goal_dist = entity_info[EntityType.ROVER].size()
    self.finish_dist = self.goal_dist

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_LANDER

class SampleSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.sample = None

  def calculate_cost(self, node: NavNode):
    cost = 0
    rover_copy = deepcopy(self.env.get_rover())
    rover_copy.set_position(node.get_position())
    for colliding in self.env.find_colliding(rover_copy, [ EntityType.LANDER, EntityType.OBSTACLE, EntityType.ROCK ]):
      cost = cost + vec2_mag_sqr(colliding.position() - rover_copy.position())
    return cost + super().calculate_cost(node)

  def can_enter(self, node: NavNode):
    return True

  def on_start(self):
    rover_pos         = self.navigator().get_rover_entity().position()
    self.sample, dist = self.env.find_closest(EntityType.SAMPLE, rover_pos)
    self.target       = self.sample.position()
    self.goal_dist    = (entity_info[EntityType.ROVER].size() + entity_info[EntityType.SAMPLE].size()) / 2
    self.finish_dist  = self.goal_dist

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

  def calculate_cost(self, node: NavNode):
    cost = 0
    rover_copy = deepcopy(self.env.get_rover())
    rover_copy.set_position(node.get_position())
    for colliding in self.env.find_colliding(rover_copy, [ EntityType.LANDER, EntityType.OBSTACLE, EntityType.SAMPLE ]):
      cost = cost + vec2_mag_sqr(colliding.position() - rover_copy.position())
    return cost + super().calculate_cost(node)

  def can_enter(self, node: NavNode):
    return True

  def on_start(self):
    rover_pos       = self.navigator().get_rover_entity().position()
    self.rock, dist = self.env.find_closest(EntityType.ROCK, rover_pos)
    self.target     = self.rock.position()
    self.goal_dist = (entity_info[EntityType.ROVER].size() + entity_info[EntityType.ROCK].size()) / 2

  def on_update(self, dt):
    # if self.rock not in self.env:
    #   self.rock = None

    if self.rock is not None:
      self.rock  = self.rock.position()
      super().on_update(dt)

  def is_done(self):
    return self.rock is None or super().is_done()

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_ROCK
