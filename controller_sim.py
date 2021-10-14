
from subsystems.interop import SCS_ACTION, DetectedObject
from subsystems.navigation import env_params
from subsystems.navigation.env_params import ObjectType
from subsystems.navigation.geometry import *
from subsystems.navigation.navigation import Navigator
from subsystems.vrep.roverbot_lib import *
from subsystems.navigation.nav_viz import NavViz

import time

sceneParameters = SceneParameters()
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'

def to_detected_objects(object_type, object_list):
  if object_list == None:
    return []

  detected_objects = []
  if len(object_list) > 0:
    if isinstance(object_list[0], list):
      for o in object_list:
        detected_objects.append(DetectedObject(object_type, o[1], o[0] * env_params.meter_scale, 0))
    else:
      detected_objects.append(DetectedObject(object_type, object_list[1], object_list[0] * env_params.meter_scale, 1))

  return detected_objects

class Controller:
  def __init__(self, ip, visualize):
    self.nav_viz = NavViz() if visualize else None
    self.sim = VREP_RoverRobot(ip, robotParameters, sceneParameters)
    self.sim.StartSimulator()
    self.sim.UpdateObjectPositions()
    self.sim.SetTargetVelocities(0, 0)
    self.sim.UpdateObjectPositions()
    self.vel = 0
    self.ang = 0
    # self.last_rover_pose = None

  def update(self, nav):
    self.rover_pose, _, _, _ = self.sim.UpdateObjectPositions()
    # if self.last_rover_pose is not None:
    #   print('Actual Rover delta: ' + ', '.join(str(b - a) for a, b in zip(self.last_rover_pose, self.rover_pose)))
    # self.last_rover_pose = self.rover_pose

    if self.nav_viz:
      self.nav_viz.update()
      self.nav_viz.draw(nav.environment(), nav.current_path(), self.vel, self.ang, nav.get_routine_type())

  def get_detected_objects(self):
    sample, lander, obstacle, rock = self.sim.GetDetectedObjects()
    visible_objects = []
    visible_objects = visible_objects + to_detected_objects(ObjectType.ROCK,     rock)
    visible_objects = visible_objects + to_detected_objects(ObjectType.SAMPLE,   sample)
    visible_objects = visible_objects + to_detected_objects(ObjectType.OBSTACLE, obstacle)
    visible_objects = visible_objects + to_detected_objects(ObjectType.LANDER,   lander)
    return visible_objects

  def set_motors(self, vel, ang):
    self.sim.SetTargetVelocities(vel, ang)
    self.vel = vel
    self.ang = ang

  def perform_action(self, action):
    if (action == SCS_ACTION.FLIP_ROCK):
      return self.flip_rock()
    if (action == SCS_ACTION.DROP_SAMPLE):
      return self.drop_sample()
    if (action == SCS_ACTION.COLLECT_SAMPLE):
      return self.collect_sample()
    time.sleep(1)
    return True

  def flip_rock(self):
    print('flip_rock() not supported by simulation.')
    return True

  def drop_sample(self):
    self.sim.DropSample()
    return not self.sim.SampleCollected()

  def collect_sample(self):
    self.sim.CollectSample()
    return self.sim.SampleCollected()

  def calibrate(self):
    print('Nothing to do')
