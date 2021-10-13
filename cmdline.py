from enum import Enum

class Command(Enum):
  NONE           = 0,
  HELP           = 1,
  COLLECT_SAMPLE = 2,
  DROP_SAMPLE    = 3,
  FLIP_ROCK      = 4,
  SET_MOTORS     = 5,
  BEGIN_NAV      = 6,
  CALIBRATE      = 7,
  EXIT           = 8

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
      'collect': [ Command.COLLECT_SAMPLE ],
      'drop':    [ Command.DROP_SAMPLE ],
      'flip':    [ Command.FLIP_ROCK ],
      'set-velocity': [
        Command.SET_MOTORS,
        Input('linear',  float),
        Input('angular', float)
      ],
      'begin-nav': [ Command.BEGIN_NAV ],
      'calibrate': [ Command.CALIBRATE ],
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
