from .env_params  import EntityType
from .env_params  import entity_info
from .nav_routine import *
from vector_2d   import Vector

class CollectRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.__target    = self.navigator().get_target_sample()
    self.__rover     = self.navigator().get_rover_entity()
    self.__to_target = Vector()
    self.__sample_collected = False
    self.__direction_valid  = False

  def on_update(self, dt):
    self.__to_target = self.__target.position() - self.__rover.position()
    self.__direction_valid = True
    if abs(self.__to_target) < 5:
      self.__sample_collected = self.navigator().try_collect_sample(self.__target)

  def is_done(self):
    return self.__target is None or self.__sample_collected

  def get_control_parameters(self):
    if not self.__direction_valid or self.__sample_collected:
      return 0, 0
    else:
      speed, ori = direction_to_control_param(self.__to_target, self.navigator().get_rover_entity())
      return speed * 0.2, ori

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.COLLECT_SAMPLE

class DropRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.__target    = None
    self.__rover     = self.navigator().get_rover_entity()
    self.__to_target = Vector()
    self.__sample_dropped = False
    self.__direction_valid = False

  def on_start(self):
    # Get the closest sample to the rover
    self.__target, _ = self.navigator().environment().find_closest(EntityType.LANDER, self.__rover.position())

  def on_update(self, dt):
    self.__to_target = self.__target.position() - self.__rover.position()
    self.__direction_valid = True
    cur_dir = VectorPolar(1, self.__rover.angle()).to_cartesian().unit()

    if vec2_dot(cur_dir.unit(), self.__to_target.unit()) > 0.9995:
      self.__sample_dropped = True
      self.navigator().set_drop_sample(True)

  def is_done(self):
    return self.__sample_dropped

  def get_control_parameters(self):
    if not self.__direction_valid or self.__sample_dropped:
      return 0, 0
    else:
      speed, ori = direction_to_control_param(self.__to_target, self.navigator().get_rover_entity())
      return speed * 0.1, ori

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.DROP_SAMPLE


class FlipRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.__target    = self.navigator().get_target_rock()
    self.__rover     = self.navigator().get_rover_entity()
    self.__to_target = Vector()
    self.__sample_collected = False

  def on_start(self):
    # Get the closest sample to the rover
    self.__target, _ = self.navigator().environment().find_closest(EntityType.ROCK, self.__rover.position())

  def on_update(self, dt):
    self.__to_target = self.__target.position() - self.__rover.position()

    dist = entity_info[EntityType.ROVER].size() + entity_info[EntityType.ROCK].size()
    dist = dist / 2

    if abs(self.__to_target) < dist - 3:
      self.__sample_collected = True
      self.navigator().set_flip_rock(True)

  def is_done(self):
    return self.__sample_collected

  def get_control_parameters(self):
    if self.__sample_collected:
      return 0, 0
    else:
      speed, ori = direction_to_control_param(self.__to_target, self.navigator().get_rover_entity())
      return 0.2 * speed, ori

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.FLIP_ROCK