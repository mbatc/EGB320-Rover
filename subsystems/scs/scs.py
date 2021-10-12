
from ..interop import SCS_ACTION

def CollectSample():
  pass

def DropSample():
  pass

def FlipRock():
  pass

def Perform(action):
  if action == SCS_ACTION.COLLECT_SAMPLE:
    CollectSample()
  elif action == SCS_ACTION.DROP_SAMPLE:
    DropSample()
  elif action == SCS_ACTION.FLIP_ROCK:
    FlipRock()

