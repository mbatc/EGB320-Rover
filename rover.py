import sys
from subsystems.interop import SCS_ACTION

# Add paths to the python modules
sys.path.insert(0, './fusion')
sys.path.insert(0, './gyro')
sys.path.insert(0, './navigation')
sys.path.insert(0, './scs')
sys.path.insert(0, './vision')
sys.path.insert(0, './mobility')

from subsystems.navigation import navigation as nav
from subsystems.vision     import vision
from subsystems.mobility   import mobility
from subsystems.scs        import scs

if __name__ == "__main__":
  navigator = nav.Navigator()
  detector  = vision.Initialize()

  while True:
    objects = vision.ObjectDetection(detector)

    if objects is not None:
      for obj in objects:
        print(obj)

      navigator.update(nav.Vector(0, 0), 0, objects)
      print('-- Environment --')
      print(navigator.environment())

      scs_action = navigator.get_scs_action()
      if scs_action == SCS_ACTION.NONE:
        control_params = navigator.get_control_parameters()
        mobility.update(control_params[0], control_params[1])
      else:
        mobility.update(0, 0)
        if scs.Perform(scs_action):
          navigator.complete_scs_action()
