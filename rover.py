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

import cmdline

manual_control = False

if __name__ == "__main__":
  cmdLine   = cmdline.RoverCommandLine()
  detector  = None
  navigator = nav.Navigator()

  # Initialize subsystems
  if mobility.imported:
    print('Initializing mobility system')
    mobility.initialze()
  else:
    print('Failed to import mobility system. Perhaps a module is missing')

  if vision.imported:
    print('Initializing vision system')
    detector = vision.Initialize()
  else:
    print('Failed to vision mobility system. Perhaps a module is missing')


  print('Rover Command Line Interface.\nType \'help\' for a list of commands.')
  while True:
    cmd_id, args = cmdLine.get_command()
    if cmd_id == cmdline.Command.COLLECT_SAMPLE:
      print('Collecting sample')
    elif cmd_id == cmdline.Command.DROP_SAMPLE:
      print('Dropping sample')
    elif cmd_id == cmdline.Command.FLIP_ROCK:
      print('Flip rock')
    elif cmd_id == cmdline.Command.SET_MOTORS:
      if mobility.imported:
        mobility.update(args[0], args[1])
    elif cmd_id == cmdline.Command.CALIBRATE:
      print('Calibration: Nothing to do...')
    elif cmd_id == cmdline.Command.BEGIN_NAV:
      try:
        print('Beginning navigation. Press CTRL+C to stop')
        while True:
          if detector is None:
            objects = []
          else:
            objects = vision.ObjectDetection(detector)

          if objects is not None:
            for obj in objects:
              print(obj)

            navigator.update(objects)
            # print('-- Environment --')
            # print(navigator.environment())

            scs_action = navigator.get_scs_action()
            if scs_action == SCS_ACTION.NONE:
              control_params = navigator.get_control_parameters()
              if mobility.imported:
                mobility.update(control_params[0], control_params[1])
            else:
              if mobility.imported:
                mobility.update(0, 0)
              if scs.Perform(scs_action):
                navigator.complete_scs_action()

      except KeyboardInterrupt as e:
        print('Ending auto navigation')
        if mobility.imported:
          mobility.update(0, 0)
    elif cmd_id == cmdline.Command.HELP:
      cmdLine.print_help()
    elif cmd_id == cmdline.Command.EXIT:
      break

  print('Shutting down...')
  if mobility.imported:
    # stop the motors
    mobility.update(0, 0)