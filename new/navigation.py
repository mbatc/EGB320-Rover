from enum import *
from environment import Environment
from geometry import *

import queue

from env_params    import *
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
  SEARCH_LANDER  = 1,
  SEARCH_ROCK    = 2,
  SEARCH_SAMPLE  = 3,
  COLLECT_SAMPLE = 4,
  DROP_SAMPLE    = 5,
  FLIP_ROCK      = 6,
  COUNT          = 7

class RoverPose:
  def __init__(self, pos, ori):
    self.__position = pos
    self.__ori      = ori

  def set_position(self, position):
    self.__position = position

  def set_ori(self, ori):
    self.__ori = ori

  def apply_velocity(self, velocity, dt):
    self.__position = self.__position + velocity * dt

  def apply_angular_velocity(self, velocity, dt):
    self.__ori = self.__ori + velocity * dt

class DetectedObject:
  def __init__(self, type, heading, distance, angle):
    self.__type     = type
    self.__heading  = heading
    self.__distance = distance
    self.__best_distance = 10
    self.__best_angle    = 5
    self.__angle = angle

  def type(self):
    return self.__type

  def heading(self):
    return self.__heading

  def distance(self):
    return self.__distance

  def angle(self):
    return self.__angle

  def calculate_position(self):
    return VectorPolar(self.__distance, self.__heading).to_cartesian()

  def get_confidence(self):
    dist_confidence = max(0, min(1, self.__best_distance / self.__distance))
    head_confidence = max(0, min(1, abs(self.__best_angle) / abs(self.__heading)))
    return dist_confidence * head_confidence

class Navigator:
  def __init__(self):
    self.__current_routine = None
    self.__environment     = Environment()
    self.__routines = {
      RoutineType.EXPLORE:        ExploreRoutine,
      RoutineType.SEARCH_LANDER:  LanderSearchRoutine,
      RoutineType.SEARCH_ROCK:    RockSearchRoutine,
      RoutineType.SEARCH_SAMPLE:  SampleSearchRoutine,
      RoutineType.COLLECT_SAMPLE: CollectRoutine,
      RoutineType.DROP_SAMPLE:    DropRoutine,
      RoutineType.FLIP_ROCK:      FlipRoutine,
    }

    self.__has_sample        = False
    self.__last_routine_type = RoutineType.NONE
    self.__rover             = self.__environment.add_entity(EntityType.ROVER, Vector(0, 0), 0, 1)

  def has_sample(self):
    return self.__has_sample

  def update(self, rover_pose, visible_objects):
    # Update the map of the environment
    visible_entities = []
    for object in visible_objects:
      visible_entities.append(self.environment().add_or_update_entity(object.type(), object.calculate_position(), object.angle(), object.get_confidence()))

    # Remove 'ghost' entities
    rover_pos = Vector(0, 0)
    rover_dir = 0
    rover_fov = 90
    self.environment().prune_visible(
      visible_entities,
      rover_pos,
      rover_dir,
      rover_fov
    )

    # Update navigation routine
    if self.routine() == None:
      next_action = self.decide_action()
      self.set_routine(next_action)
    else:
      self.routine().update()

      if self.routine().is_done():
        self.finish_routine()

  def decide_action(self):
    last_routine = self.__last_routine_type

    if self.has_sample():
      if last_routine == RoutineType.SEARCH_LANDER:
        return RoutineType.DROP_SAMPLE
      else:
        return RoutineType.SEARCH_LANDER
    else:
      if last_routine == RoutineType.SEARCH_SAMPLE:
        return RoutineType.COLLECT_SAMPLE
      elif last_routine == RoutineType.SEARCH_SAMPLE:
        return RoutineType.FLIP_ROCK

      if self.environment().has_entities(EntityType.SAMPLE):
        return RoutineType.SEARCH_SAMPLE
      if self.environment().has_entities(EntityType.ROCK):
        return RoutineType.SEARCH_ROCK

    return RoutineType.EXPLORE

  def finish_routine(self):
    if self.__current_routine == None:
      self.__last_routine_type = RoutineType.NONE
    else:
      self.__last_routine_type = self.__current_routine.get_type()

  def set_routine(self, routine_type):
    # Check if the routine type has been registered
    if routine_type not in self.__routines:
      raise Exception("Routine type is not registered")
    # Create a new routine of the requested type
    self.__current_routine = self.__routines[routine_type](self)

  def environment(self):
    return self.__environment

  def routine(self):
    return self.__current_routine

  def get_rover_entity(self):
    return self.__rover

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
