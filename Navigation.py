from sys import path
from vector_2d import Vector
from vector_2d import VectorPolar
from queue import PriorityQueue
from math import * 
from multipledispatch import dispatch
from dataclasses import dataclass, field
from typing import Any

import sys
import turtle
import random

def vec2_min(a:Vector, b:Vector): return Vector(x=min(a.x, b.x), y=min(a.y, b.y))
def vec2_max(a:Vector, b:Vector): return Vector(x=max(a.x, b.x), y=max(a.y, b.y))
def vec2_clamp(a:Vector, min_vec:Vector, max_vec:Vector): return vec2_max(vec2_min(a, max_vec), min_vec)
def vec2_dot(a:Vector, b:Vector): return Vector(x=a.x*b.x, y=a.y*b.y)
def vec2_mag_sqr(a:Vector):       return a.x * a.x + a.y * a.y
def vec2_round(a:Vector):         return Vector(round(a.x), round(a.y))

# Some helpers for representing 2D geometry
class Rect:
  def __init__(self, position: Vector, size: Vector):
    '''
    Construct a 2D Rectangle from position and size vectors.
    '''
    self.min  = position - size * 0.5
    self.max  = position + size * 0.5

  @staticmethod
  def smallest():
    rect = Rect(Vector(), Vector())
    min_val = -sys.float_info.max
    max_val = sys.float_info.max
    rect.max = Vector(min_val, min_val)
    rect.min = Vector(max_val, max_val)
    return rect

  def scale(self, factor):
    self.min = self.min * factor
    self.max = self.max * factor

  def size(self):
    return self.max - self.min

  def half_size(self):
    return self.size() * 0.5

  def closest_point(self, point:Vector):
    return vec2_clamp(point, self.min, self.max)

  def contains_point(self, point:Vector):
    return self.closest_point(point) == point

  def grow_to_contain(self, extents):
    self.min = vec2_min(extents.min, self.min)
    self.max = vec2_max(extents.max, self.max)

  def extents(self):
    return self

class Circle:
  def __init__(self, position:Vector, radius):
    self.position = position
    self.radius   = radius

  def rad_sqr(self):
    return self.radius * self.radius

  def closest_point(self, point:Vector):
    to_point = point - self.position
    len_sqr = vec2_mag_sqr(to_point)
    return self.position + sqrt(min(len_sqr, self.radius)) * to_point.unit()

  def contains_point(self, point:Vector):
    return vec2_mag_sqr(point - self.position) <= self.rad_sqr()

  def distance(self, point:Vector):
    return sqrt(self.distance_sqr(point))

  def distance_sqr(self, point:Vector):
    return vec2_mag_sqr(point - self.position)

  def extents(self):
    return Rect(self.position, Vector(self.radius * 2, self.radius * 2))

@dispatch(Rect, Rect)
def intersect(a, b):
  a_min = a.min()
  a_max = a.max()
  b_min = b.min()
  b_max = b.max()
  return a_max.x > b_min.x and a_max.y > b_min.y and b_max.y > a_min.y and b_max.x > a_min.x

@dispatch(Circle, Circle)
def intersect(a, b):
  return abs(a.position - b.position) <= a.radius + b.radius

@dispatch(Rect, Circle)
def intersect(a, b):
  return b.contains(a.closest_point(b.position))

@dispatch(Circle, Rect)
def intersect(a, b):
  return intersect(b, a)

@dispatch(turtle.Turtle, Circle)
def debug_draw(tt, body):
  tt.penup()
  tt.setpos(body.position.x, body.position.y - body.radius)
  tt.pendown()
  tt.circle(body.radius)

# Constructs for representing the environment

class Entity:
  def __init__(self, entity_type, body):
    self.type = entity_type
    self.body = body

  def position(self):
    return self.body.position

  def distance(self, point:Vector):
    return sqrt(self.distance_sqr(point))

  def distance_sqr(self, point:Vector):
    return vec2_mag_sqr(point - self.body.closest_point(point))

class Environment:
  def __init__(self, precision = 1):
    self.entities    = []
    self.type_lookup = {}
    self.precision   = 1

  def get_extents(self):
    '''
    Get the bounds of the rovers environment
    '''
    extents = Rect.smallest()
    for entity in self.entities:
      extents.grow_to_contain(entity.body.extents())
    return extents

  def add(self, entity_type, body):
    # Remove existing entities of the same type that overlap with the new body
    for existing in self.get_colliding(body, entity_type):
      self.remove(existing)

    new_entity = Entity(entity_type, body)
    # Add to the list of all entities
    self.entities.append(new_entity)
    # Add to an entity group by type
    if entity_type in self.type_lookup:
      self.type_lookup[entity_type].append(new_entity)
    else:
      self.type_lookup[entity_type] = [ new_entity ]

  def remove(self, entity):
    try:
      # Remove from the full list, and type specific list
      self.entities.remove(entity)
      self.type_lookup[entity.type].remove(entity)
      return True
    except ValueError as e:
      return False # Handle not in list exception, just return false

  def get_entities(self, entity_type=None):
    if entity_type == None:
      return self.entities
    return self.type_lookup[entity_type] if entity_type in self.type_lookup else []

  def get_colliding(self, body, entity_type=None):
    collisions = []
    for entity in self.get_entities(entity_type):
      if body is not entity.body and intersect(entity.body, body):
        collisions.append(entity)
    return collisions

  def get_first_colliding(self, body, entity_type=None):
    for entity in self.get_entities(entity_type):
      if body is not entity.body and intersect(entity.body, body):
        return entity
    return None


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

    # Limit to 8 potential directions for broad path search
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

    self.expand_frontier(PathNode(vec2_round(start_entity.position() / self.precision)), None)

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
    return colliding_body == None or self.is_goal(test_node)

  def expand_frontier(self, new_node, prev_node):
    if new_node.content in self.explored or not self.can_enter(new_node):
      return False
    new_node.set_previous(prev_node)
    cost = self.calc_cost(new_node.content, prev_node)
    self.frontier.put(PrioritizedItem(cost, new_node))
    return True

import time
import math

class Navigator:
  def __init__(self):
    self.environment  = Environment()
    self.current_path = [] # An array of vectors

  def record_object(self, type, position):
    '''
    Record an object in the navigation software
    '''
    pass

  def move_rover(self, amount:Vector):
    '''
    Move the rover by the specified amount.
    '''
    pass

  def rotate_rover(self, amount):
    '''
    Rotate the rover by the specified number of radians
    '''
    pass

  def pick_target(self, type):
    '''
    Select a new target to navigate to
    '''
    pass

  def update_path(self):
    '''
    Recalculate the path to the target object
    '''
    pass

  def get_direction(self):
    '''
    Get the target direction for the rover
    '''
    pass

  def get_speed(self):
    '''
    Get the target speed for the rover
    '''
    pass

if __name__ == "__main__":
  random.seed(time.time())

  wn = turtle.Screen()
  tt = turtle.Turtle()

  def DoPathFind():
    env = Environment()
    start = VectorPolar(400, random.random() * math.pi * 2)
    env.add('rover',  Circle(start.to_cartesian(), 10))

    min_size = 5
    max_size = 10

    sample_count = 5
    rock_count = 10
    scrn_min = -200
    scrn_max = 200
    for i in range(sample_count):
      env.add('sample', Circle(Vector(random.randrange(scrn_min, scrn_max), random.randrange(scrn_min, scrn_max)), random.randrange(min_size, max_size)))

    for i in range(rock_count):
      env.add('rock', Circle(Vector(random.randrange(scrn_min, scrn_max), random.randrange(scrn_min, scrn_max)), random.randrange(min_size, max_size)))

    rover  = env.get_entities('rover')[0]
    all_samples = env.get_entities('sample')
    sample = env.get_entities('sample')[random.randrange(0, len(all_samples))]

    for entity in env.get_entities():
      print('[ ' + entity.type + ' ]: ' + str(entity.body.position))

    pathFinder = PathFinder(env, rover.body, rover, sample, 15)

    try:
      while not pathFinder.search():
        pass
    except:
      pass
    
    tt.speed(0)
    wn.clearscreen()
    tt.pendown()
    for entity in env.get_entities():
      if entity.type == 'rock':   tt.color('blue')
      if entity.type == 'sample': tt.color('orange')
      if entity.type == 'rover':  tt.color('red')
      debug_draw(tt, entity.body)

    tt.penup()
    tt.color('green')

    node = pathFinder.found_node
    if node == None:
      node = pathFinder.best_node

    if node != None:
      for p in node.path:
        pos = pathFinder.get_environment_position(p.content)
        tt.setpos(pos.x, pos.y)
        tt.pendown()

    wn.listen()
    wn.onkeypress(DoPathFind, 'r')

  wn.listen()
  wn.onkeypress(DoPathFind, 'r')
  wn.mainloop()
