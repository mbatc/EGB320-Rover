from enum import Enum

class Command(Enum):
  NONE           = 0,
  HELP           = 1,
  COLLECT_SAMPLE_PREP = 2,
  COLLECT_SAMPLE = 3,
  DROP_SAMPLE    = 4,
  FLIP_ROCK      = 5,
  FLIP_ROCK_PREP = 6,
  SET_MOTORS     = 7,
  BEGIN_NAV      = 8,
  CALIBRATE      = 9,
  SHOW_DETECTED  = 10,
  SET_SERVO_1    = 11,
  SET_SERVO_2    = 12,
  EXIT           = 13

class Input:
  def __init__(self, name, type):
    self.name = name
    self.type = type

  def input(self):
    while True:
      value = input('{}: '.format(self.name))
      try:
        return self.type(value)
      except Exception as e:
        print(e)

  def __str__(self):
    return '{} [{}]'.format(self.name, str(self.type))


class RoverCommandLine:
  def __init__(self):
    # Define commands for the console interface
    self.commands = {
      'help':    [ Command.HELP ],
      'collect-prep': [Command.COLLECT_SAMPLE_PREP ],
      'collect': [ Command.COLLECT_SAMPLE ],
      'drop':    [ Command.DROP_SAMPLE ],
      'flip':    [ Command.FLIP_ROCK ],
      'flip-prep': [Command.FLIP_ROCK_PREP ],
      'set-servo-1': [
        Command.SET_SERVO_1,
        Input('angle', int)
      ],
      'set-servo-2': [
        Command.SET_SERVO_2,
        Input('angle', int)
      ],
      'set-velocity': [
        Command.SET_MOTORS,
        Input('linear',  float),
        Input('angular', float)
      ],
      'begin-nav': [ Command.BEGIN_NAV ],
      'calibrate': [ Command.CALIBRATE ],
      'show-detected': [Command.SHOW_DETECTED ],
      'exit':      [ Command.EXIT ]
    }

  def print_help(self):
    print('-- COMMAND LIST -- \n\n' + '\n\n'.join([ '\n'.join(['> {}'.format(cmd)] + [ '  -' + str(a) for a in args ]) for cmd, args in self.commands.items()]))

  def get_command(self):
    cmd = input('> ')
    if cmd not in self.commands:
      print("Unknown command '{}'. Use 'help' for command info.".format(cmd))
      return Command.NONE, []

    params = self.commands[cmd]
    cmd_id = params[0]
    args   = []

    if len(params) > 0:
      for arg in params[1:]:
        args.append(arg.input())
    
    return cmd_id, args
