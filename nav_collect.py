from env_params  import EntityType
from nav_routine import *
from vector_2d   import Vector

class CollectRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.__target    = None
    self.__rover     = self.navigator().get_rover_entity()
    self.__to_target = Vector()
    self.__sample_collected = False

  def on_start(self):
    # Get the closest sample to the rover
    self.__target, _ = self.navigator().environment().find_closest(EntityType.SAMPLE, self.__rover.position())

  def on_update(self, dt):
    self.__to_target = self.__target.position() - self.__rover.position()

    if abs(self.__to_target) < 3:
      self.__sample_collected = True
      self.navigator().set_collect_sample(True)

  def is_done(self):
    return self.__sample_collected

  def get_control_parameters(self):
    return direction_to_control_param(self.__to_target, self.navigator().get_rover_entity())
