from enum import *

from .geometry import *

class RoutineType(Enum):
  '''
  This enum describes the navigation routines that
  have been implemented.
  '''
  NONE           = -1,
  EXPLORE        = 0,
  SEARCH_LANDER  = 1,
  SEARCH_ROCK    = 2,
  SEARCH_SAMPLE  = 3,
  COLLECT_SAMPLE = 4,
  DROP_SAMPLE    = 5,
  FLIP_ROCK      = 6,
  COUNT          = 7

def direction_to_control_param(target_dir, rover):
  cur_dir        = VectorPolar(1, rover.angle()).to_cartesian().unit()
  target_dir     = target_dir.unit()
  ori_correction = pow((1 - max(0, vec2_dot(cur_dir, target_dir))), 0.5)
  ori_correction = ori_correction * sign(vec2_cross(cur_dir, target_dir))
  return 1 - abs(ori_correction), ori_correction

class Routine:
  '''
  Base class for a navigation routine.
  '''
  def __init__(self, navigator):
    self.__navigator = navigator
    self.__first_update = True
    pass

  def update(self, dt):
    '''
    This function should take a step in updating the navigation routine.
    '''
    if self.__first_update:
      self.on_start()
      self.__first_update = False

    self.on_update(dt)

  def navigator(self):
    '''
    Get the navigation context that is executing this routine.
    '''
    return self.__navigator

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.NONE

  def get_control_parameters(self):
    '''
    Get the current contol parameters to apply for this navigation routine.
    '''
    raise Exception("Not Implemented")

  def is_done(self):
    '''
    This function should return True when the navigation
    routine has been completed.
    '''
    raise Exception("Not Implemented")

  def get_path(self):
    return []

  def on_start(self):
    pass

  def on_update(self, dt):
    pass

  def on_complete(self):
    pass