import traceback
from environment import *

from queue import PriorityQueue
from dataclasses import dataclass, field
from typing import Any

# Navigation implementation

@dataclass(order=True)
class PrioritizedItem:
    priority: int
    item: Any=field(compare=False)

class PathNode:
  def __init__(self, content, previous = None):
    self.previous = previous
    self.content  = content
    self.path     = None

  def set_previous(self, previous):
    prev_path = previous.path if previous != None else None
    self.path = prev_path.copy() if prev_path != None else []
    self.path.append(self)

  def path_len(self):
    return len(self.path) if self.path != None else 0

  def path(self):
    return self.path

  def prev(self):
    return self.path[-2] if len(self.path) > 1 else None

class PathFinder:
  '''
  This class can be used to generate a path from one entity to another within an Environment.
  '''
  def __init__(self, environment, nav_body, start_entity, target_entity, precision=1):
    self.environment   = environment
    self.nav_body      = nav_body
    self.precision     = precision
    self.start_entity  = start_entity
    self.target_entity = target_entity
    self.found_node    = None
    self.best_node     = None
    self.best_cost     = 10000000
    self.current_node  = None
    self.explored      = set()
    self.frontier      = PriorityQueue()
    self.extents       = self.environment.get_extents()
    self.ignored       = self.environment.get_colliding(nav_body)
    
    # Limit to 8 potential directions for path search
    self.steps = [
      Vector(x=0, y=1),
      Vector(x=1, y=1),
      Vector(x=1, y=0),
      Vector(x=1, y=-1),
      Vector(x=0, y=-1),
      Vector(x=-1, y=1),
      Vector(x=-1, y=0),
      Vector(x=-1, y=1),
    ]

    new_node = PathNode(vec2_round(start_entity.position() / self.precision))
    self.frontier.put(PrioritizedItem(sys.float_info.max, new_node))

  def search(self):
    '''
    Perform a step in the path-finding algorithm.

    Returns false if a path has not yet been found
    Returns true if the path has been found
    Throws an exception if no path could be found
    '''
    if self.frontier.empty():
      if self.found_node != None:
        return True
      else:
        raise Exception('No valid path could be found')

    queued = self.frontier.get()
    self.current_node = queued.item
    if queued.priority < self.best_cost:
      self.best_node = self.current_node
      self.best_cost = queued.priority

    # Mark the current nodes position as explored
    self.explored.add(self.current_node.content)

    # Check if the current node satisfies the goal condition
    if self.is_goal(self.current_node):
      self.found_node = self.current_node
      return True

    # Expand the frontier using the next potential steps from this node
    for direction in self.steps:
      next_node = PathNode(self.current_node.content + direction)
      self.expand_frontier(next_node, self.current_node)

  def calc_cost(self, position, prev_node = None):
    '''
    The heuristic to use when searching for a path.
    '''
    return self.target_entity.distance(self.get_environment_position(position))

  def get_environment_position(self, path_position):
    return path_position * self.precision

  def is_goal(self, test_node):
    # Move our navigation collider body to the test nodes position
    self.nav_body.position = self.get_environment_position(test_node.content)
    return intersect(self.target_entity.body, self.nav_body)

  def can_enter(self, test_node):
    # Move our navigation collider body to the test nodes position
    self.nav_body.position = self.get_environment_position(test_node.content)
    if not intersect(self.extents, self.nav_body):
      return False

    colliding_body = self.environment.get_first_colliding(self.nav_body)
    return colliding_body == None or colliding_body in self.ignored or self.is_goal(test_node)

  def expand_frontier(self, new_node, prev_node):
    if new_node.content in self.explored or not self.can_enter(new_node):
      return False
    new_node.set_previous(prev_node)
    cost = self.calc_cost(new_node.content, prev_node)
    self.frontier.put(PrioritizedItem(cost, new_node))
    return True
