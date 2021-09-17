from enum import *

import queue

from nav_collect   import *
from nav_drop      import *
from nav_explore   import *
from nav_search    import *
from nav_flip_rock import *

class RoutineType(Enum):
  '''
  This enum describes the navigation routines that
  have been implemented.
  '''
  NONE           = -1,
  EXPLORE        = 0,
  SEARCH         = 1,
  COLLECT_SAMPLE = 2,
  DROP_SAMPLE    = 3,
  FLIP_ROCK      = 4,
  COUNT          = 5

class Navigator:
  def __init__(self):
    self.__routine_stack   = queue.LifoQueue()
    self.__current_routine = None
    self.__environment     = None
    self.__routines = {
      RoutineType.EXPLORE:        ExploreRoutine,
      RoutineType.SEARCH:         SearchRoutine,
      RoutineType.COLLECT_SAMPLE: CollectRoutine,
      RoutineType.DROP_SAMPLE:    DropRoutine,
      RoutineType.FLIP_ROCK:      FlipRoutine,
    }

  def update(self):
    if self.routine() == None:
      next_action = self.decide_action()
      self.set_routine(next_action)
    else:
      self.routine().update()

  def environment(self):
    return self.__environment

  def routine(self):
    return self.__current_routine

  def push_routine(self, routine_type):
    # Check if the routine type has been registered
    if routine_type not in self.__routines:
      raise Exception("Routine type is not registered")

    # Create a new routine of the requested type
    new_routine = self.__routines[routine_type](self)

    # Push the current routine to the stack so we can resume
    # it after completing the new routine.
    if self.__current_routine != None:
      self.__routine_stack.put(self.__current_routine)
    # Set the current routine
    self.__current_routine = new_routine

  def get_control_parameters(self):
    if self.__current_routine == None:
      return 0, 0
    else:
      return self.__current_routine.get_control_parameters()

class Routine:
  '''
  Base class for a navigation routine.
  '''
  def __init__(self, navigator:Navigator):
    self.__navigator = navigator
    self.__first_update = True
    pass

  def update(self, dt):
    '''
    This function should take a step in updating the navigation routine.
    '''
    if self.__first_update:
      self.on_start(dt)
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

  def on_start(self):
    pass

  def on_update(self, dt):
    pass
