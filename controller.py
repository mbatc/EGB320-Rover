
from subsystems import interop

has_mobility = True
has_scs      = True
has_vision   = True

# Try import subsystems
try:
  from subsystems.mobility import mobility
except (RuntimeError, ModuleNotFoundError) as ex:
  has_mobility = False
  print(ex)

try:
  from subsystems.scs import scs
except (RuntimeError, ModuleNotFoundError) as ex:
  has_scs = False
  print(ex)

try:
  from subsystems.vision import vision
except (RuntimeError, ModuleNotFoundError) as ex:
  has_vision = False
  print(ex)

class Controller:
  def __init__(self):
    self.detector = None
    if has_mobility:
      mobility.initialze()
      print('Initializing mobility system')
    else:
      print('Failed to initializing mobility system. Perhaps a module is missing')

    if has_scs:
      scs.initialize()
      print('Initializing collection system')
    else:
      print('Failed to initializing collection system. Perhaps a module is missing')

    if has_vision:
      self.detector = vision.Initialize()
      print('Initializing vision system')
    else:
      print('Failed to vision mobility system. Perhaps a module is missing')

  def __del__(self):
    if has_scs:
      scs.shutdown()

    if has_mobility:
      mobility.shutdown()

  def update(self, nav):
    '''
    Nothing to update for this controller
    '''
    pass

  def get_detected_objects(self):
    if has_vision and self.detector is not None:
      return vision.ObjectDetection(self.detector)
    else:
      print('Cannot Set Motors. Mobility system not available.')

  def set_motors(self, vel, ang):
    if has_mobility:
      mobility.update(vel * 100, -ang * 100)
    else:
      print('Cannot Set Motors. Mobility system not available.')

  def perform_action(self, action):
    if (action == interop.SCS_ACTION.FLIP_ROCK):
      return self.flip_rock()
    if (action == interop.SCS_ACTION.DROP_SAMPLE):
      return self.drop_sample()
    if (action == interop.SCS_ACTION.COLLECT_SAMPLE):
      return self.collect_sample()
    return False

  def flip_rock(self):
    if not has_scs:
      print('Cannot Flip Rock. Collection system not available')
    else:
      scs.FlipRock()
    return True

  def drop_sample(self):
    if not has_scs:
      print('Cannot Drop Sample. Collection system not available')
    else:
      scs.DropSample()
    return True

  def collect_sample(self):
    if not has_scs:
      print('Cannot Collect Sample. Collection system not available')
    else:
      scs.CollectSample()
    return True

  def calibrate(self):
    print('Nothing to do')
