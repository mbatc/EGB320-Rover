from graph_search import *
from nav_routine  import *

class FlipRoutine(Routine):
  def __init__(self, navigator):
    super().__init__(navigator)

  def on_start(self):
    self.navigator

  def on_update(self, dt):
    pass
