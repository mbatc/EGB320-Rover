from environment import Environment
from geometry import *

import time

from env_params    import *
from nav_scs       import *
from nav_search    import *

class DetectedObject:
  def __init__(self, type, distance, heading, angle):
    self.__type          = type
    self.__heading       = heading
    self.__distance      = distance
    self.__best_distance = 10
    self.__best_heading  = 5
    self.__angle         = angle
    self.missing_time  = 0

  def type(self):
    return self.__type

  def distance(self):
    return self.__distance

  def calculate_heading(self, rover):
    return rover.angle() + self.__heading

  def calculate_angle(self, rover):
    return rover.angle() + self.__angle

  def calculate_position(self, rover):
    return rover.position() + VectorPolar(self.distance(), self.calculate_heading(rover)).to_cartesian()

  def get_confidence(self):
    dist_confidence = max(0, min(1, self.__best_distance / self.__distance))
    head_confidence = max(0, min(1, abs(self.__best_heading) / abs(self.__heading)))
    return dist_confidence * head_confidence

  def __str__(self):
    return '(type:{}, angle: {}, dist: {})'.format(self.__type, self.__heading, self.__distance)

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
    self.__sample_to_collect = False
    self.__try_flip_rock      = False
    self.__try_drop_sample    = False
    self.__scs_delay          = 2
    self.__scs_start_time     = 0
    self.__is_scs_active      = False
    self.__routine_delay      = 2
    self.__routine_end_time   = 0
    self.__last_routine_type  = RoutineType.NONE
    self.__rover              = self.__environment.add_entity(EntityType.ROVER, Vector(0, 0), 0, 1)
    self.__lander             = self.__environment.add_entity(EntityType.LANDER, Vector(0, 0), 0, 1)
    self.__last_update        = time.time()
    self.__target_sample      = None
    self.__target_rock      = None
    self.__dt                 = 0
    self.__rover_start_pos    = Vector(0, 0)
    self.__rover_delta_pos    = Vector(0, 0)
    self.__attached = []
    self.__sim = sim

  def has_sample(self):
    return self.__has_sample

  def get_target_sample(self):
    return self.__target_sample

  def set_target_sample(self, sample):
    self.__target_sample = sample

  def get_target_rock(self):
    return self.__target_rock

  def set_target_rock(self, sample):
    self.__target_rock = sample

  def attach_sample(self, entity):
    self.__attached.append(entity)

  def rover_start_position(self):
    return self.__rover_start_pos

  def get_rover_delta_position(self):
    return self.__rover_delta_pos

  def update(self, rover_delta_pos, rover_angle, visible_objects):
    update_time = time.time()
    self.__dt   = update_time - self.__last_update
    self.__rover_delta_pos = rover_delta_pos
    self.__rover.set_position(self.__rover_delta_pos)
    self.__rover.set_angle   (rover_angle)
    self.__rover_start_pos -= self.__rover_delta_pos
    self.environment().bring_to_center(self.__rover)

    for entity in self.__attached:
      entity.set_position(self.__rover.position())

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
      if object.distance() > 5:
        visible_entities.append(self.environment().add_or_update_entity(
          object.type(),
          object.calculate_position(self.__rover),
          object.calculate_angle(self.__rover),
          object.get_confidence()))

    sample_to_remove = []
    # If any samples are intersecting with the lander, remove them
    # We probably already collected it
    for sample in self.environment().get_group(EntityType.SAMPLE):
      if self.environment().find_first_colliding(sample, EntityType.LANDER) is not None:
        sample_to_remove.append(sample)
    for sample in sample_to_remove:
      self.environment().remove(sample)

    # Combine overlapping entities
    self.environment().combine_overlapping()

    # Remove 'ghost' entities
    rover_pos = self.__rover.position()
    rover_dir = self.__rover.direction()
    rover_fov = 60
    for entity_type in [EntityType.SAMPLE, EntityType.ROCK, EntityType.OBSTACLE]:
      self.environment().prune_visible(
        self.__dt,
        visible_entities,
        rover_pos,
        rover_dir,
        rover_fov,
        15,
        40,
        entity_type
      )

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
    if self.__sample_to_collect is not None:
      if self.__has_sample:
        self.__environment.remove(self.__sample_to_collect)
      self.__sample_to_collect = None
      return True
    elif self.__try_drop_sample:
      self.__sim.DropSample()
      self.__has_sample = False
      self.__try_drop_sample = False
      self.__attached = []
      return True
    elif self.__try_flip_rock:
      self.__try_flip_rock = False
      return True

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
      if last_routine == RoutineType.SEARCH_SAMPLE and self.get_target_sample() != None:
        return RoutineType.COLLECT_SAMPLE
      elif last_routine == RoutineType.SEARCH_ROCK and self.get_target_rock() != None:
        return RoutineType.FLIP_ROCK

      if last_routine == RoutineType.FLIP_ROCK:
        return RoutineType.SEARCH_LANDER

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
      self.__current_routine.on_complete()
      self.__last_routine_type = self.__current_routine.get_type()
      print('Finished ' + str(self.__last_routine_type))
    self.__current_routine  = None
    self.__routine_end_time = time.time()

  def current_path(self):
    routine = self.routine()
    return routine.get_path() if routine is not None else []

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

  def get_routine_type(self):
    return RoutineType.NONE if self.__current_routine is None else self.__current_routine.get_type() 

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

  def try_collect_sample(self, try_collect_sample):
    '''
    Set a flag indicating that we should attempt to collect a sample.
    '''
    self.__has_sample        = self.__sim.CollectSample()
    if self.__has_sample:
      self.__sample_to_collect = try_collect_sample
    return self.__has_sample

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
