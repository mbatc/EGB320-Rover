from subsystems.interop import SCS_ACTION
from .env_params  import ObjectType
from .nav_routine import *
from vector_2d    import Vector
from .            import config

class CollectRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.__target    = None
    self.__rover     = self.navigator().get_rover_entity()
    self.__to_target = Vector()
    
    self.find_target()
    self.update_target_dist()
    
    # Prepare the SCS system for sample collection
    self.navigator().set_scs_action(SCS_ACTION.COLLECT_SAMPLE_PREP)

  def find_target(self):
    self.__target, _ = self.navigator().environment().find_closest(ObjectType.SAMPLE, self.__rover.position())
  
  def update_target_dist(self):
    self.__to_target = self.__target.position() - self.__rover.position()

  def on_update(self, dt):
    self.update_target_dist()

  def is_done(self):
    # Navigate until we are close enough to the target
    return abs(self.__to_target) < config.COLLECT_DIST

  def on_complete(self):
    # Try collect the sample
    self.navigator().set_scs_action(SCS_ACTION.COLLECT_SAMPLE)

  def get_control_parameters(self):
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
    self.__rover     = self.navigator().get_rover_entity()
    self.__target    = None
    self.__to_target = Vector()
    self.__started_scs = False
    
    self.find_target()
    self.update_target_dist()
    
  def find_target(self):
    self.__target, _ = self.navigator().environment().find_closest(ObjectType.LANDER, self.__rover.position())
  
  def update_target_dist(self):
    self.__to_target = self.__target.position() - self.__rover.position()
    
  def on_update(self, dt):
    self.__to_target = self.__target.position() - self.__rover.position()
    self.__direction_valid = True
    cur_dir = VectorPolar(1, self.__rover.angle()).to_cartesian().unit()

    if vec2_dot(cur_dir.unit(), self.__to_target.unit()) > config.DROP_ORI_THRESHOLD:
      if not self.__started_scs:
        self.navigator().drop_sample()
      self.__started_scs = True

  def is_done(self):
    return abs(self.__to_target) < config.DROP_DIST

  def get_control_parameters(self):
    speed, ori = direction_to_control_param(self.__to_target, self.navigator().get_rover_entity())
    return 0.2 * speed, ori

  def on_complete(self):
    self.navigator().set_scs_action(SCS_ACTION.DROP_SAMPLE)

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.DROP_SAMPLE


class FlipRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.__rover     = self.navigator().get_rover_entity()
    self.__to_target = Vector()

    self.find_target()
    self.update_target_dist()
    
    self.navigator().set_scs_action(SCS_ACTION.FLIP_ROCK_PREP)

  def find_target(self):
    self.__target, _ = self.navigator().environment().find_closest(ObjectType.ROCK, self.__rover.position())

  def update_target_dist(self):
    self.__to_target = self.__target.position() - self.__rover.position()

  def on_update(self, dt):
    self.update_target_dist()

  def is_done(self):
    return abs(self.__to_target) < config.FLIP_DIST

  def get_control_parameters(self):
    speed, ori = direction_to_control_param(self.__to_target, self.navigator().get_rover_entity())
    return 0.2 * speed, ori

  def get_type(self):
    '''
    Get the navigation routine type.
    '''
    return RoutineType.FLIP_ROCK
