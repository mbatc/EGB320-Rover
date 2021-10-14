from .environment import Entity
from .env_params   import ObjectType
from .env_params   import entity_info
from .graph_search import *
from .geometry     import *
from .nav_routine  import *
from copy          import *

import math
import time
import random

rock_goal_dist   = 3
sample_goal_dist = 3
lander_goal_dist = 5

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

step_size = 10 # cm

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
    self.__path_pos   = set() if prev_node is None else copy(prev_node.__path_pos)
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
    self.target_pos    = None
    self.target_entity = None
    self.env    = navigator.environment()
    self.failed = False
    self.path   = []
    self.goal_dist   = 5
    self.finish_dist = 5
    self.prev_length = 100000
    self.last_update = 0
    self.refresh_period = 1 # seconds

  def get_obstacle_types(self):
    return []

  def calculate_cost(self, node:NavNode):
    cost = node.path_length() + vec2_mag_sqr(self.target_pos - node.get_position())
    rover_copy = deepcopy(self.env.get_rover())
    rover_copy.set_position(node.get_position())
    for colliding in self.env.find_colliding(rover_copy, self.get_obstacle_types()):
      cost = cost * (2 + 100 / max(0.01, vec2_mag_sqr(colliding.position() - rover_copy.position())))
    return cost

  def can_enter(self, node:NavNode):
    return True

  def is_goal(self, node):
    return vec2_mag_sqr(node.get_position() - self.target_pos) <= (self.goal_dist * self.goal_dist)

  def on_update(self, dt):
    if self.target_entity is not None:
      self.target_pos = self.target_entity.position()

    recalulate_path = False
    # delta_pos = self.navigator().get_rover_delta_position()
    # self.path = [ pos - delta_pos for pos in self.path ]

    stale_time = time.time() - self.last_update
    if stale_time > self.refresh_period:
      recalulate_path = True

    path_node_index = 0
    for pos in self.path:
      cell = Rect(pos, Vector(step_size, step_size))
      if not cell.contains_point(self.navigator().get_rover_entity().position()):
        path_node_index = path_node_index + 1
      else:
        break
    self.path = self.path[path_node_index:]

    rover_copy = deepcopy(self.env.get_rover())
    for pos in self.path:
      rover_copy.set_position(pos)
      if self.env.find_first_colliding(rover_copy, self.get_obstacle_types()) is not None:
        recalulate_path = True
        break

    if recalulate_path or len(self.path) == 0:
      self.recalulate_path()

  def recalulate_path(self):
    rover       = self.navigator().get_rover_entity()
    start_node  = NavNode(self, None, rover.position(), (0, 0))
    path_finder = GraphSearch()
    goal        = None
    path_finder.expand_frontier(start_node)

    try:
      max_itr = 1000
      while goal is None and max_itr > 0:
        goal = path_finder.search()
        max_itr = max_itr - 1
    except Exception as e:
      print('Path find Failed: ' + str(e))

    if goal is None:
      goal = path_finder.get_best()

    if goal is not None:
      self.path = [ node.get_position() for node in goal.path() ]
    else:
      self.path = []

    self.last_update = time.time()

  def get_path(self):
    return self.path

  def get_control_parameters(self):
    path_len = len(self.path)
    if path_len == 0:
      return 0, 0

    target_dir = None
    if path_len == 1:
      target_dir = self.target_pos - self.path[0]
    else:
      target_dir = self.path[1] - self.path[0]

    return direction_to_control_param(target_dir, self.navigator().get_rover_entity())

  def is_done(self):
    if self.target_pos is None:
      return False
    rover = self.navigator().get_rover_entity()
    return abs(self.target_pos - rover.position()) < self.finish_dist

class ExploreRoutine(SearchRoutine):
  def __init__(self, navigator):
    self.target_angle = 0
    super().__init__(navigator)

  def get_obstacle_types(self):
    return [ ObjectType.LANDER, ObjectType.OBSTACLE, ObjectType.ROCK, ObjectType.SAMPLE ]

  def is_goal(self, node):
    return super().is_goal(node)

  def on_start(self):
    rover_copy = deepcopy(self.navigator().get_rover_entity())
    env_extents = self.navigator().environment().extents([ObjectType.ROCK, ObjectType.OBSTACLE, ObjectType.SAMPLE, ObjectType.LANDER])
    env_size    = abs(env_extents.size()) * 0.75
    self.target_angle = math.pi * (random.random() * 2 - 1)
    if env_extents.is_smallest():
      target_pos = rover_copy.position()
    else:
      while self.target_entity is None:    
        angle       = random.random() * 2 - 1
        distance    = random.random() * env_size
        print('angle: {}'.format(angle))
        target_pos  = env_extents.center() + VectorPolar(distance, angle * math.pi).to_cartesian()

        rover_copy.set_position(target_pos)
        intersect = self.env.find_first_colliding(rover_copy)
        if intersect is None or intersect is self.navigator().get_rover_entity():
          break

    self.target_entity = self.env.add_entity(ObjectType.EXPLORE, target_pos, 0, 1)
    self.goal_dist   = entity_info[ObjectType.ROVER].size() / 2
    self.finish_dist = self.goal_dist

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.EXPLORE

  def is_done(self):
    rover = deepcopy(self.env.get_rover())
    rover.set_position(self.target_pos)
    if self.env.has_entities(ObjectType.SAMPLE):
      return True

    if self.env.find_first_colliding(rover) is not None:
      return True
      
    if super().is_done():
      return abs(self.navigator().get_rover_entity().get_angle() - self.target_angle) < math.radians(5)

    return False

  def get_control_parameters(self):
    if super().is_done():
      return 0, 0.5 if self.navigator().get_rover_entity().get_angle() < self.target_angle else -0.5
    else:
      return super().get_control_parameters()

  def on_complete(self):
    self.env.remove(self.target_entity)

class LanderSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)

  def get_obstacle_types(self):
    return [ ObjectType.OBSTACLE, ObjectType.ROCK, ObjectType.SAMPLE ]

  def can_enter(self, node:NavNode):
    return True

  def on_start(self):
    rover_pos             = self.navigator().get_rover_entity().position()
    self.target_entity, _ = self.env.find_closest(ObjectType.LANDER, rover_pos)
    self.goal_dist        = entity_info[ObjectType.ROVER].size() / 2 + lander_goal_dist
    self.finish_dist      = self.goal_dist

  def on_update(self, dt):
    if self.target_entity is None:
      self.target_pos = self.navigator().rover_start_position()
    # Try find the lander
    rover_pos = self.navigator().get_rover_entity().position()
    self.target_entity, _ = self.env.find_closest(ObjectType.LANDER, rover_pos)
    return super().on_update(dt)

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_LANDER

class SampleSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)

  def get_obstacle_types(self):
    return [ ObjectType.LANDER, ObjectType.OBSTACLE, ObjectType.ROCK ]

  def can_enter(self, node: NavNode):
    return True

  def on_start(self):
    self.goal_dist        = (entity_info[ObjectType.ROVER].size() + entity_info[ObjectType.SAMPLE].size()) / 2 + sample_goal_dist
    self.finish_dist      = self.goal_dist

  def on_update(self, dt):
    rover_pos             = self.navigator().get_rover_entity().position()
    self.target_entity, _ = self.env.find_closest(ObjectType.SAMPLE, rover_pos)

    if self.target_entity is not None:
      super().on_update(dt)

  def is_done(self):
    return self.target_entity is None or super().is_done()

  def on_complete(self):
    self.navigator().set_target_sample(self.target_entity)

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_SAMPLE


class RockSearchRoutine(SearchRoutine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.rock = None

  def get_obstacle_types(self):
    return [ ObjectType.LANDER, ObjectType.OBSTACLE, ObjectType.SAMPLE ]

  def can_enter(self, node: NavNode):
    return True

  def on_start(self):
    rover_pos             = self.navigator().get_rover_entity().position()
    self.goal_dist        = (entity_info[ObjectType.ROVER].size() + entity_info[ObjectType.ROCK].size()) / 2 + rock_goal_dist
    self.finish_dist      = self.goal_dist

  def on_update(self, dt):
    rover_pos             = self.navigator().get_rover_entity().position()
    self.target_entity, _ = self.env.find_closest(ObjectType.ROCK, rover_pos)

    if self.target_entity is not None:
      super().on_update(dt)

  def is_done(self):
    return self.target_entity is None or super().is_done()

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_ROCK
