
from .config import DEFAULT_WEIGHT_VEL, DEFAULT_WEIGHT_ANG

class PoseEstimator:
  def __init__(self):
    self.bias = 0
    self.weights = []
    self.num_samples = 20
    try:
      self.weights = self.read_weights('weights.txt')
    except:
      self.weights = [ DEFAULT_WEIGHT_VEL, DEFAULT_WEIGHT_ANG ]
  def __del__(self):
    try:
      self.write_weights('weights.txt')
    except:
      pass

  def read_weights(self, path):
    weights = []
    with open(path, 'r') as file:
      for l in file:
        weights.append(float(l))
    return weights

  def write_weights(self, path, weights):
    with open(path, 'w') as file:
      for w in weights:
        file.write('{}\n'.format(w))

  def __apply_w(self, w, w2):
    return (w * (self.num_samples - 1) + w2) / self.num_samples

  def add_sample(self, inputs, outputs):
    try:
      new_weights  = [ o / i for i, o in zip(inputs, outputs) ]
      prev_weights = self.weights
      if len(self.weights) > 0:
        self.weights = [ self.__apply_w(w, w2) for w, w2 in zip(self.weights, new_weights) ]
      else:
        self.weights = new_weights
      # print('weights: {} -> {}'.format(prev_weights, self.weights))
    except ZeroDivisionError as e:
      if len(self.weights) == 0:
        self.weights = [ o for o in outputs ]

  def get_pose(self, inputs):
    pose = [ i * o for i, o in zip(inputs, self.weights) ]
    # print('pose: {} -> {}'.format(inputs, pose))
    return pose
