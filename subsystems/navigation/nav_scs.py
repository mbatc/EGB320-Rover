from subsystems.interop import SCS_ACTION
from .env_params  import ObjectType
from .env_params  import entity_info
from .nav_routine import *
from vector_2d   import Vector

class CollectRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.__target    = self.navigator().get_target_sample()
    self.__rover     = self.navigator().get_rover_entity()
    self.__to_target = Vector()
    self.__started_scs = False
    self.__direction_valid  = False

  def on_update(self, dt):
    self.__to_target = self.__target.position() - self.__rover.position()
    self.__direction_valid = True
    if abs(self.__to_target) < 2:
      self.__started_scs = True
      self.navigator().collect_sample()

  def is_done(self):
    return self.__target is None or self.navigator.get_scs_action() == SCS_ACTION.NONE

  def get_control_parameters(self):
    if not self.__direction_valid or self.__started_scs:
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
    self.__started_scs = False
    self.__direction_valid = False

  def on_start(self):
    # Get the closest sample to the rover
    self.__target, _ = self.navigator().environment().find_closest(ObjectType.LANDER, self.__rover.position())

  def on_update(self, dt):
    self.__to_target = self.__target.position() - self.__rover.position()
    self.__direction_valid = True
    cur_dir = VectorPolar(1, self.__rover.angle()).to_cartesian().unit()

    if vec2_dot(cur_dir.unit(), self.__to_target.unit()) > 0.9995:
      self.__started_scs = True
      self.navigator().drop_sample()

  def is_done(self):
    return self.__target is None or (self.__started_scs and self.navigator.get_scs_action() == SCS_ACTION.NONE)

  def get_control_parameters(self):
    if not self.__direction_valid or self.__started_scs:
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
    self.__started_scs = False

  def on_start(self):
    # Get the closest sample to the rover
    self.__target, _ = self.navigator().environment().find_closest(ObjectType.ROCK, self.__rover.position())

  def on_update(self, dt):
    self.__to_target = self.__target.position() - self.__rover.position()

    dist = entity_info[ObjectType.ROVER].size() + entity_info[ObjectType.ROCK].size()
    dist = dist / 2

    if abs(self.__to_target) < dist - 3:
      self.navigator().flip_rock(True)
      self.__started_scs = True

  def is_done(self):
    return self.__target is None or (self.__started_scs and self.navigator.get_scs_action() == SCS_ACTION.NONE)

  def get_control_parameters(self):
    if self.__started_scs:
      return 0, 0
    else:
      speed, ori = direction_to_control_param(self.__to_target, self.navigator().get_rover_entity())
      return 0.2 * speed, ori

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.FLIP_ROCK