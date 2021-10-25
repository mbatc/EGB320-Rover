
class pid_controller:
  def __init__(self, kp, ki, kd):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.accum = 0
    self.last_err = 0

  def reset(self):
    self.last_err = 0
    self.accum = 0

  def update(self, dt, current, fixed_point):
    err = fixed_point - current
    self.accum = current * dt
    vel = (err - self.last_err) / dt
    self.last_err = err

    return err * self.kp + self.accum * self.ki + vel * self.kd