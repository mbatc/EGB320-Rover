from env_params import EntityType
from navigation import *
from graph_search import *

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
    if prev_node != None:
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
    return [ NavNode(self, self.__position + direction * step_size, self.__env) for direction in step_directions ]
    
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
    return self.__position == other.__position and self.__path_len == other.__path_len

class SearchRoutine(Routine):
  def __init__(self, navigator:Navigator):
    super().__init__(navigator)
    self.target = None
    self.env    = navigator.environment()
    self.failed = False
    self.path   = []

  def on_update(self):
    rover       = self.navigator().get_rover_entity()
    start_node  = NavNode(None, rover.position(), self.env, self.target)
    path_finder = GraphSearch()
    path_finder.expand_frontier(start_node)
    goal = None
    try:
      while goal == None:
        goal = path_finder.search()
    except:
      # Path find failed
      self.failed = True

    self.path = [ node.get_position() for node in goal.path() ]

  def get_control_parameters(self):
    path_len = len(self.path)
    if path_len == 0:
      return 0, 0

    target_dir = None
    if path_len == 1:
      target_dir = self.target - self.path[0]
    else:
      target_dir = self.path[1] - self.path[0]

    rover          = self.navigator().get_rover_entity()
    cur_dir        = VectorPolar(1, rover.angle()).to_cartesian().unit()
    target_dir     = target_dir.unit()
    ori_correction = vec2_cross(cur_dir, target_dir)

    return 1, ori_correction

  def is_done(self):
    '''
    This function should return True when the navigation
    routine has been completed.
    '''
    rover = self.navigator().get_rover_entity()
    return abs(self.target - rover.position()) < 3

class LanderSearchRoutine:
  def __init__(self, navigator:Navigator):
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

class SampleSearchRoutine:
  def __init__(self, navigator:Navigator):
    super().__init__(navigator)

  def on_start(self):
    rover_pos   = self.navigator().get_rover_entity().position()
    sample      = self.env.find_closest(EntityType.SAMPLE, rover_pos)
    self.target = sample.position()

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_SAMPLE


class RockSearchRoutine:
  def __init__(self, navigator:Navigator):
    super().__init__(navigator)

  def on_start(self):
    rover_pos   = self.navigator().get_rover_entity().position()
    rock        = self.env.find_closest(EntityType.ROCK, rover_pos)
    self.target = rock.position()

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.SEARCH_ROCK
