
class Pose:
  def __init__(self):
    pass

class PoseEstimator:
  def __init__(self):
    self.bias = 0
    self.weights = []
    self.num_samples = 20
    self.last_outputs = []

  def add_sample(self, inputs, outputs):
    try:
      self.last_outputs = [ o / i for o, i in zip(outputs, inputs) ]
    except Exception as e:
      if len(self.last_outputs) == 0:
        self.last_outputs = outputs

  def get_delta(self, inputs):
    return [o * i for o, i in zip(self.last_outputs, inputs)]
