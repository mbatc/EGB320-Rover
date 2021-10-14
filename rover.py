from subsystems.navigation import navigation as nav
from subsystems.interop    import SCS_ACTION

import time
import cmdline
import argparse

# Default parameters
simulated      = True
visualize_nav  = True # Only valid for simulation
print_timing   = False
print_detected = False
print_env      = False

def navigate(navigator, controller):
  while True:
    update_start = time.time()
    controller.update(navigator)

    objects = controller.get_detected_objects()

    if print_detected:
      if objects is not None:
        for obj in objects:
          print(obj)

    nav_start_time = time.time()
    navigator.update(objects)
    nav_update_time = time.time() - nav_start_time

    action = navigator.get_scs_action()

    control_params = navigator.get_control_parameters()
    controller.set_motors(control_params[0], control_params[1])

    if action != SCS_ACTION.NONE:
      if controller.perform_action(action):
        navigator.complete_scs_action(True)

    if print_env:
      print('-- Environment --')
      print(navigator.environment())

    if print_timing:
      print('Nav Update Time: {}'.format(nav_update_time))
      print('Full Update time: {}'.format(time.time() - update_start - nav_update_time))

def show_detected(navigator, controller):
  while True:
    controller.update(navigator)
    objects = controller.get_detected_objects()
    if len(objects) == 0:
      continue

    print('[')
    if objects is not None:
      for obj in objects:
        print('  {}'.format(obj))
    print(']')


def parse_args():
  global simulated
  global print_env
  global print_detected
  global print_timing
  global visualize_nav

  # Instantiate the parser
  parser = argparse.ArgumentParser(description='EGB320-Rover command line interface.')
  parser.add_argument('--sim',type=int, default=simulated, help='Specified whether to use the simulator.')
  parser.add_argument('--nav_vis', type=int, default=visualize_nav, help='Specified whether to visualise the navigation system state (simulation only).')
  parser.add_argument('--verbose_env', type=int, default=print_env, help='Print the environment state.')
  parser.add_argument('--verbose_detect', type=int, default=print_detected, help='Print the detected objects.')
  parser.add_argument('--verbose_timing', type=int, default=print_timing, help='Print the navigation update timing.')
  args = parser.parse_args()

  simulated      = args.sim >= 1
  visualize_nav  = args.nav_vis >= 1
  print_env      = args.verbose_env >= 1
  print_detected = args.verbose_detect >= 1
  print_timing   = args.verbose_timing >= 1

def run(controller):
  print('Rover Command Line Interface.\nType \'help\' for a list of commands.')
  
  cmdLine   = cmdline.RoverCommandLine()
  navigator = nav.Navigator()

  while True:
    cmd_id, args = cmdLine.get_command()
    if cmd_id == cmdline.Command.COLLECT_SAMPLE:
      print('Collecting sample')
      controller.collect_sample()
    elif cmd_id == cmdline.Command.DROP_SAMPLE:
      print('Dropping sample')
      controller.drop_sample()
    elif cmd_id == cmdline.Command.FLIP_ROCK:
      print('Flip rock')
      controller.flip_rock()
    elif cmd_id == cmdline.Command.SET_MOTORS:
      print('Setting motor speeds')
      controller.set_motors(args[0], args[1])
    elif cmd_id == cmdline.Command.CALIBRATE:
      print('Calibrating...')
      controller.calibrate()
    elif cmd_id == cmdline.Command.BEGIN_NAV:
      try:
        print('Beginning navigation. Press CTRL+C to stop')
        navigate(navigator, controller)
      except KeyboardInterrupt as e:
        print('Ending navigation')
        controller.set_motors(0, 0)
    elif cmd_id == cmdline.Command.HELP:
      cmdLine.print_help()
    elif cmd_id == cmdline.Command.SHOW_DETECTED:
      try:
        print('showing cv2 detected objects. Press CTRL+C to stop')
        show_detected(navigator, controller)
      except KeyboardInterrupt as e:
        print('Ending display detected...')
    elif cmd_id == cmdline.Command.EXIT:
      break
  pass

if __name__ == "__main__":
  parse_args()

  # Import either physical or simulated controller
  if simulated:
    print('Importing Controller (Simulated)')
    from controller_sim import Controller
    controller = Controller('127.0.0.1', visualize_nav)
  else:
    print('Importing Controller')
    from controller import Controller
    controller = Controller()

  try:
    run(controller)
  except Exception as e:
    print('An unhandled exception occured: {}'.format(e))
    raise # re-throw the exception
  finally:
    # Always stop the motors
    print('Shutting down...')
    controller.set_motors(0, 0)