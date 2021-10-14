from subsystems.navigation.pose_estimation import PoseEstimator
from .environment import Environment
from .geometry import *
from .env_params import *
from .nav_scs    import *
from .nav_search import *
from ..interop  import *
from .pose_estimation import PoseEstimator

import time

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

    self.__has_sample         = False
    self.__routine_delay      = 2
    self.__routine_end_time   = 0
    self.__last_routine_type  = RoutineType.NONE
    self.__rover              = self.__environment.add_entity(ObjectType.ROVER, Vector(0, 0), 0, 1)
    self.__lander             = self.__environment.add_entity(ObjectType.LANDER, Vector(0, 0), 0, 1)
    self.__last_update        = time.time()
    self.__target_sample      = None
    self.__target_rock        = None
    self.__dt                 = 0
    self.__rover_start_pos    = Vector(0, 0)
    self.__attached           = []
    self.__scs_action         = SCS_ACTION.NONE
    self.__pose_estimator     = PoseEstimator()

    self.__lander.invalidate_position()

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

  # def get_rover_delta_position(self):
  #   return self.__rover_delta_pos

  def update(self, visible_objects):
    update_time = time.time()
    self.__dt   = update_time - self.__last_update
    self.__last_update = update_time
    for entity in self.__attached:
      entity.set_position(self.__rover.position())

    self.update_environment(visible_objects)

    if self.__scs_action == SCS_ACTION.NONE:
      self.update_navigation_routine(self.__dt)

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

    self.estimate_rover_pose(visible_entities)

    for object in visible_objects:
      if object.distance() > 5:
        self.environment().add_or_update_entity(
          object.type(),
          object.calculate_position(self.__rover),
          object.calculate_angle(self.__rover),
          object.get_confidence())

    sample_to_remove = []
    # If any samples are intersecting with the lander, remove them
    # We probably already collected it
    for sample in self.environment().get_group(ObjectType.SAMPLE):
      if self.environment().find_first_colliding(sample, ObjectType.LANDER) is not None:
        sample_to_remove.append(sample)
    for sample in sample_to_remove:
      self.environment().remove(sample)

    # Combine overlapping entities
    # self.environment().combine_overlapping()

    # Remove 'ghost' entities
    rover_pos = self.__rover.position()
    rover_dir = self.__rover.direction()
    rover_fov = 60
    for entity_type in [ObjectType.SAMPLE, ObjectType.ROCK, ObjectType.OBSTACLE]:
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

  def estimate_rover_pose(self, visible_entities):
    rover_dir = self.__rover.direction()

    rover_delta_pos   = Vector(0,0)
    rover_delta_theta = 0

    # Estimate rover position/orientation based on movement of visible entities
    sample_count = 0
    for entity in visible_entities:
      last_position = entity.last_position() - self.__rover.position()
      new_position  = entity.position() - self.__rover.position()
      outer_arc     = Circle(Vector(0, 0), abs(last_position))
      if outer_arc.radius() < 0.0001:
        continue # It was previously on top of the rover, ignore this object
      pos_trace     = Line(new_position - rover_dir * outer_arc.radius(), new_position + rover_dir * outer_arc.radius())
      a, b = calculate_intersections(pos_trace, outer_arc)
      if a is None:
        arc_pos = b
      elif b is None:
        arc_pos = a
      else:
        arc_pos = a if abs(a - new_position) < abs(b - new_position) else b
      if arc_pos is None:
        continue

      arc = vec2_angle(last_position, new_position) * sign(vec2_cross(last_position, new_position))
      # print('arc: ' + str(degrees(arc)))
      rover_delta_pos   = rover_delta_pos   + (arc_pos - new_position)
      rover_delta_theta = rover_delta_theta - arc
      sample_count = sample_count + 1

    vel, angular_vel = self.get_control_parameters()
    inputs = [vel, angular_vel]

    if sample_count > 0:
      # Apply rover pose delta estimate
      rover_delta_theta = rover_delta_theta / sample_count
      rover_delta_pos   = rover_delta_pos   / sample_count
      # print(degrees(rover_delta_theta))
      # print(rover_delta_pos)
      pose = [ abs(rover_delta_pos) / self.__dt, rover_delta_theta / self.__dt]
      self.__rover.set_position(self.__rover.position() + rover_delta_pos)
      self.__rover.set_angle(self.__rover.angle()       + rover_delta_theta)
      print('vel: {}\t {}', pose[0], pose[1])
      self.__pose_estimator.add_sample(inputs, pose)
    else:
      pose = self.__pose_estimator.get_delta(inputs)
      if len(pose) > 0:
        print('model vel: {}\t {}', pose[0], pose[1])
        self.__rover.set_position(self.__rover.position() + self.__rover.direction() * pose[0] * self.__dt)
        self.__rover.set_angle(self.__rover.angle()       + pose[1] * self.__dt)

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

      if self.environment().has_entities(ObjectType.SAMPLE):
        return RoutineType.SEARCH_SAMPLE
      if self.environment().has_entities(ObjectType.ROCK):
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
    params = 0, 0
    if self.__current_routine is not None:
      params = self.__current_routine.get_control_parameters()

    # Scale control parameters based on navigation configuration options
    return params[0] * config.MOVE_SPEED_FAST, params[1] * config.ROTATE_SPEED

  def collect_sample(self):
    '''
    Set a flag indicating that we should attempt to collect a sample.
    '''
    self.__scs_action = SCS_ACTION.COLLECT_SAMPLE

  def flip_rock(self):
    '''
    Set a flag indicating that we should attempt to flip a rock.
    '''
    self.__scs_action = SCS_ACTION.FLIP_ROCK
    
  def drop_sample(self):
    '''
    Set a flag indicating that we should attempt to drop a sample.
    '''
    self.__scs_action = SCS_ACTION.DROP_SAMPLE

  def complete_scs_action(self, success):
    if self.__scs_action == SCS_ACTION.COLLECT_SAMPLE:
      self.__has_sample = True
    if self.__scs_action == SCS_ACTION.DROP_SAMPLE:
      self.__has_sample = False
    self.__scs_action = SCS_ACTION.NONE

  def get_scs_action(self):
    return self.__scs_action
