
class PoseEstimator:
  def __init__(self):
    self.bias = 0
    self.weights = []
    self.num_samples = 20
    self.last_outputs = []

  def add_sample(self, inputs, outputs, dt):
    try:
      self.last_outputs = [ o / (i * dt) for o, i in zip(outputs, inputs) ]
    except Exception as e:
      if len(self.last_outputs) == 0:
        self.last_outputs = [ o / dt for o in outputs ]

  def get_delta(self, inputs, dt):
    return [i * o * dt for o, i in zip(self.last_outputs, inputs)]
