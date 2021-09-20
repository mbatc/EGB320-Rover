from environment import Environment
from geometry import *

import time

from env_params    import *
from nav_collect   import *
from nav_drop      import *
from nav_explore   import *
from nav_search    import *
from nav_flip_rock import *

class RoverPose:
  def __init__(self, pos, angle):
    self.__position = pos
    self.__angle    = angle

  def set_position(self, position):
    self.__position = position

  def set_angle(self, angle):
    self.__angle = angle

  def apply_velocity(self, velocity, dt):
    self.__position = self.__position + velocity * dt

  def apply_angular_velocity(self, velocity, dt):
    self.__angle = self.__angle + velocity * dt

  def get_position(self):
    return self.__position

  def get_angle(self):
    return self.__angle

class DetectedObject:
  def __init__(self, type, distance, heading, angle):
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
  def __init__(self, sim):
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

    self.__has_sample         = False
    self.__try_collect_sample = False
    self.__try_flip_rock      = False
    self.__try_drop_sample    = False
    self.__scs_delay          = 2
    self.__scs_start_time     = 0
    self.__is_scs_active      = False
    self.__routine_delay      = 2
    self.__routine_end_time   = 0
    self.__last_routine_type  = RoutineType.NONE
    self.__rover              = self.__environment.add_entity(EntityType.ROVER, Vector(0, 0), 0, 1)
    self.__last_update        = time.time()
    self.__dt                 = 0

    self.__sim = sim

  def has_sample(self):
    return self.__has_sample

  def update(self, rover_pose, visible_objects):
    update_time = time.time()
    self.__dt   = update_time - self.__last_update
    self.__rover.set_position(rover_pose.get_position())
    self.__rover.set_angle(rover_pose.get_angle())

    self.update_environment(visible_objects)

    if not self.__is_scs_active:
      self.update_navigation_routine(self.__dt)

      if self.try_perform_sample_collection_action():
        self.__is_scs_active  = True
        self.__scs_start_time = time.time()
    else:
      self.__is_scs_active = self.__is_scs_active and time.time() - self.__scs_start_time < self.__scs_delay

  def update_environment(self, visible_objects):
    # Update the map of the environment
    visible_entities = []
    for object in visible_objects:
      visible_entities.append(self.environment().add_or_update_entity(
        object.type(),
        object.calculate_position(),
        object.angle(),
        object.get_confidence()))

    # Combine overlapping entities
    self.environment().combine_overlapping()

    # Remove 'ghost' entities
    rover_pos = self.__rover.position()
    rover_dir = self.__rover.direction()
    rover_fov = 60
    # self.environment().prune_visible(
    #   visible_entities,
    #   rover_pos,
    #   rover_dir,
    #   rover_fov
    # )

  def update_navigation_routine(self, dt):
    '''
    Update the active navigation routine. This should only be called if the
    sample collection system is not active.
    '''
    # Update navigation routine
    if self.routine() == None:
      if time.time() - self.__routine_end_time > self.__routine_delay:
        next_action = self.decide_action()
        self.set_routine(next_action)
    else:
      self.routine().update(dt)

      if self.routine().is_done():
        self.finish_routine()

  def try_perform_sample_collection_action(self):
    '''
    Try perform an action using the sample collection system.
    '''
    if self.__try_collect_sample:
      self.__sim.CollectSample()
    elif self.__try_drop_sample:
      self.__sim.DropSample()
    elif self.__try_flip_rock:
      pass

    return False

  def decide_action(self):
    '''
    Return the next action to take based on the Navigation systems current state.
    '''
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
    '''
    Finish the current routine.
    '''
    if self.__current_routine == None:
      self.__last_routine_type = RoutineType.NONE
    else:
      self.__last_routine_type = self.__current_routine.get_type()
    self.__current_routine  = None
    self.__routine_end_time = time.time()
    print('Finished ' + str(self.__last_routine_type))

  def set_routine(self, routine_type):
    '''
    Set the current routine being performed
    '''
    # Check if the routine type has been registered
    if routine_type not in self.__routines:
      raise Exception("Routine type is not registered")
    # Create a new routine of the requested type
    self.__current_routine = self.__routines[routine_type](self)
    print('Started ' + str(routine_type))

  def environment(self):
    '''
    Get the Navigation environment.
    '''
    return self.__environment

  def routine(self):
    '''
    Get the current Navigation Routine environment.
    '''
    return self.__current_routine

  def get_rover_entity(self):
    '''
    Get the entity that represents the Rover.
    '''
    return self.__rover

  def get_control_parameters(self):
    '''
    Get the current control parameters for the navigation system.
    '''
    if self.__is_scs_active or self.__current_routine == None:
      return 0, 0
    else:
      return self.__current_routine.get_control_parameters()

  def set_collect_sample(self, try_collect_sample):
    '''
    Set a flag indicating that we should attempt to collect a sample.
    '''
    self.__try_collect_sample = try_collect_sample

  def set_flip_rock(self, try_collect_sample):
    '''
    Set a flag indicating that we should attempt to flip a rock.
    '''
    self.__try_flip_rock = try_collect_sample
    
  def set_drop_sample(self, try_collect_sample):
    '''
    Set a flag indicating that we should attempt to drop a sample.
    '''
    self.__try_drop_sample = try_collect_sample
