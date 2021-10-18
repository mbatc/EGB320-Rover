from subsystems.navigation import navigation as nav
from subsystems.interop    import SCS_ACTION

import time
import cmdline
import argparse

# Default parameters
simulated      = True

def navigate(navigator, initial_state=None, expected_state=None):
  if expected_state is None:
    expected_state = initial_state

  navigator.set_state(initial_state, nav.StateTransition.CLEAR_STACK)

  while navigator.update() != expected_state or expected_state is None:
    pass

def show_detected(controller):
  while True:
    controller.update()
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
  args = parser.parse_args()

  simulated = args.sim >= 1

def run(controller):
  print('Rover Command Line Interface.\nType \'help\' for a list of commands.')
  
  cmdLine   = cmdline.RoverCommandLine()
  navigator = nav.Navigator(controller)

  while True:
    cmd_id, args = cmdLine.get_command()
    if cmd_id == cmdline.Command.COLLECT_SAMPLE_PREP:
      print('Preparing to Collecting sample')
      controller.perform_action(SCS_ACTION.COLLECT_SAMPLE_PREP)
    elif cmd_id == cmdline.Command.COLLECT_SAMPLE:
      print('Collecting sample')
      controller.perform_action(SCS_ACTION.COLLECT_SAMPLE)
    elif cmd_id == cmdline.Command.DROP_SAMPLE:
      print('Dropping sample')
      controller.perform_action(SCS_ACTION.DROP_SAMPLE)
    elif cmd_id == cmdline.Command.FLIP_ROCK_PREP:
      print('Preparing to Flip rock')
      controller.perform_action(SCS_ACTION.FLIP_ROCK_PREP)
    elif cmd_id == cmdline.Command.FLIP_ROCK:
      print('Flipping rock')
      controller.perform_action(SCS_ACTION.FLIP_ROCK)
    elif cmd_id == cmdline.Command.SET_MOTORS:
      print('Setting motor speeds')
      controller.set_motors(args[0], args[1])
    elif cmd_id == cmdline.Command.SET_SERVO_1:
      controller.set_servo(0, args[0])
    elif cmd_id == cmdline.Command.SET_SERVO_2:
      controller.set_servo(1, args[0])
    elif cmd_id == cmdline.Command.CALIBRATE:
      print('Calibrating...')
      controller.calibrate()
    elif cmd_id == cmdline.Command.BEGIN_NAV:
      try:
        print('Beginning navigation. Press CTRL+C to stop')
        navigate(navigator)
      except KeyboardInterrupt as e:
        print('Ending navigation')
        controller.set_motors(0, 0)
    elif cmd_id == cmdline.Command.HELP:
      cmdLine.print_help()
    elif cmd_id == cmdline.Command.SHOW_DETECTED:
      try:
        print('showing cv2 detected objects. Press CTRL+C to stop')
        show_detected(controller)
      except KeyboardInterrupt as e:
        print('Ending display detected...')
    elif cmd_id == cmdline.Command.NAV_SEARCH_SAMPLE:
      navigate(navigator, nav.State.DISCOVER_SAMPLE)
    elif cmd_id == cmdline.Command.NAV_SEARCH_ROCK:
      navigate(navigator, nav.State.DISCOVER_SAMPLE_OR_ROCK)
    elif cmd_id == cmdline.Command.NAV_SEARCH_LANDER:
      navigate(navigator, nav.State.DISCOVER_LANDER)
    elif cmd_id == cmdline.Command.NAV_GOTO_SAMPLE:
      navigate(navigator, nav.State.NAV_SAMPLE)
    elif cmd_id == cmdline.Command.NAV_GOTO_ROCK:
      navigate(navigator, nav.State.NAV_ROCK)
    elif cmd_id == cmdline.Command.NAV_GOTO_LANDER:
      navigate(navigator, nav.State.NAV_LANDER)
    elif cmd_id == cmdline.Command.NAV_COLLECT_SAMPLE:
      navigate(navigator, nav.State.NAV_SAMPLE, nav.State.DISCOVER_LANDER)
    elif cmd_id == cmdline.Command.NAV_FLIP_ROCK:
      navigate(navigator, nav.State.NAV_ROCK, nav.State.DISCOVER_SAMPLE)
    elif cmd_id == cmdline.Command.NAV_DROP_SAMPLE:
      navigate(navigator, nav.State.NAV_LANDER, nav.State.DISCOVER_SAMPLE_OR_ROCK)
    elif cmd_id == cmdline.Command.EXIT:
      break
  pass

if __name__ == "__main__":
  parse_args()

  # Import either physical or simulated controller
  if simulated:
    print('Importing Controller (Simulated)')
    from controller_sim import Controller
    controller = Controller('127.0.0.1')
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
