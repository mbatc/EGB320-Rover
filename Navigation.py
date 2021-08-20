import traceback
from environment import *
from path_finder import *

import random

entity_size = {
  'rover':    0.1,
  'rock':     0.1,
  'sample':   0.1,
  'obstacle': 0.1,
  'lander':   0.5
}

class Navigator:
  def __init__(self):
    self.environment  = Environment()
    self.environment.add('rover', Circle(Vector(0, 0), entity_size['rover']))
    self.environment.add('explore', Circle(Vector(0, 0), 0.05))
    self.rover         = self.environment.get_entities('rover')[0]
    self.explore_point = self.environment.get_entities('explore')[0]
    self.current_path  = [] # An array of vectors
    self.target        = None
    self.on_reached    = {}
    self.__is_exploring = False

  def on_target_reached(self, type, callback):
    self.on_reached[type] = callback

  def record_object_list(self, object_type, objects):
    '''
    Record an object in the navigation software
    '''
    for o in objects:
      self.record_object(object_type, o[0], o[1])

  def record_object(self, object_type, range, bearing):
    '''
    Record an object in the navigation software
    '''
    position = self.rover.body.position
    offset   = VectorPolar(range, bearing + self.rover.body.rotation).to_cartesian()
    position += offset
    self.environment.add(object_type, Circle(position, entity_size[object_type]))

  def set_rover(self, position, rotation):
    '''
    Explicitly set the rovers position
    '''
    self.rover.body.position = position
    self.rover.body.rotation = rotation

    if self.target != None:
      # We say we reached a target be the rovers body
      # is overlapping with the targets body
      if intersect(self.rover.body, self.target.body):
        # Call the 'on-reached' callback
        if self.target.type in self.on_reached:
          self.on_reached[self.target.type]()
        # Clear the target
        self.target = None

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

  def is_exploring(self):
    return self.__is_exploring

  def explore(self):
    extents = self.environment.get_extents()

    self.explore_point.body.position = self.rover.body.position + Vector(0, 0.01)
    self.explore_point.body.rotation = random.random() * pi * 2

    while True:
      pos = Vector(random.random(), random.random())
      pos = Vector(extents.min.x + extents.size().x * pos.x, extents.min.y + extents.size().y * pos.y)

      if self.environment.get_first_colliding(self.explore_point.body) == None:
        break

      self.explore_point.body.position = self.explore_point.body.position + Vector(0, 0.1)
      self.explore_point.body.rotation = random.random() * pi * 2

    self.target = self.explore_point
    self.__is_exploring = True

  def pick_target(self, type):
    '''
    Select a new target to navigate to
    '''
    if type == 'rover':
      return False

    entity_list = self.environment.get_entities(type)
    if len(entity_list) == 0:
      return False
    if len(entity_list) == 1 and entity_list[0] != self.rover:
      self.target = entity_list[0]
      return True

    while self.target == None or self.target == self.rover:
      self.target = entity_list[random.randrange(0, len(entity_list) - 1)]
    self.__is_exploring = False
    return True

  def clear_path(self):
    '''
    Clear the current path and reset the target object
    '''
    self.target       = None
    self.current_path = []

  def update_path(self, max_steps = 1000):
    '''
    Recalculate the path to the target object.

    Specify a maximum number of steps to take before terminating.
    '''
    nav_body    = Circle(self.rover.body.position, self.rover.body.radius)
    path_finder = PathFinder(self.environment, nav_body, self.rover, self.target, 0.1)

    success = False
    try:
      for i in range(max_steps):
        if path_finder.search():
          success = True
          break
      node = path_finder.found_node if path_finder.found_node != None else path_finder.best_node
      if node != None:
        self.current_path = [ path_finder.get_environment_position(point.content) for point in node.path ]
      
      if len(self.current_path) < 2:
        self.clear_path()
    except:
      traceback.print_exc()

    return success

  def print_env(self):
    print('-- Navigation Environment -- ')
    for entity in self.environment.get_entities():
      print('  [' + entity.type + ']\t' + str(entity.body.position))
    print()

  def get_direction(self):
    '''
    Get the target direction for the rover
    '''
    if len(self.current_path) < 2:
      if self.is_exploring():
        return self.explore_point.body.rotation
      else:
        return self.rover.body.rotation
    return (self.current_path[1] - self.current_path[0]).to_polar().angle

  def get_dir_correction(self):
    current = VectorPolar(1, self.rover.body.rotation).to_cartesian().unit()
    target  = VectorPolar(1, self.get_direction()).to_cartesian().unit()

    return (1 - (1 + vec2_dot(current, target)) / 2) * sign(vec2_cross(current, target))

  def get_speed(self):
    '''
    Get the target speed for the rover
    '''
    if len(self.current_path) < 2:
      return 0
    else:
      return 1
