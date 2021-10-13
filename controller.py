
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
      print('Initializing collection system')
    else:
      print('Failed to initializing collection system. Perhaps a module is missing')

    if has_vision:
      self.detector = vision.Initialize()
      print('Initializing vision system')
    else:
      print('Failed to vision mobility system. Perhaps a module is missing')

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
      mobility.update(vel, ang)
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
      print('flip_rock() is not implemented. Assumed successful')
    return True

  def drop_sample(self):
    if not has_scs:
      print('Cannot Drop Sample. Collection system not available')
    else:
      print('drop_sample() is not implemented. Assumed successful')
    return True

  def collect_sample(self):
    if not has_scs:
      print('Cannot Collect Sample. Collection system not available')
    else:
      print('collect_sample() is not implemented. Assumed successful')
    return True

  def calibrate(self):
    print('Nothing to do')
