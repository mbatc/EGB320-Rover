from navigation import *
from graph_search import *

class FlipRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)

  def on_start(self):
    self.navigator

  def on_update(self):
    pass
