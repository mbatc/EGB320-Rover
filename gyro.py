

from math import *
import time
import copy
import mpu6050
from fusion.fusion import *

class Gyro:
  def __init__(self):
    self.mpu    = mpu6050.mpu6050(0x68)
    self.fusion = Fusion()

  def update(self):
    accel = self.mpu.get_accel_data()
    gryo  = self.mpu.get_gyro_data()
    self.fusion.update_nomag((accel['x'], accel['y'], accel['z']), (gryo['x'], gryo['y'], gryo['z']))
    self.ypr = [self.fusion.heading, self.fusion.pitch, self.fusion.roll]

if __name__ == "__main__":
    last_print = time.time()
    while (1):
      gyro = Gyro()
      gyro.update()
      print('ypr: {}'.format(gyro))
