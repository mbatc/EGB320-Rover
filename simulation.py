# Import navigation code
from env_params import EntityType
# from nav_viz    import NavViz
from navigation import Navigator
from navigation import DetectedObject
from geometry   import *

import env_params

from roverbot_lib import *

class RoverPose:
  def __init__(self, pos, angle):
    self.__position = pos
    self.__angle    = angle
    self.__last_position = pos

  def set_position(self, position):
    self.__last_position = self.__position
    self.__position      = position

  def set_angle(self, angle):
    self.__angle = angle

  def apply_velocity(self, velocity, dt):
    self.__position = self.__position + velocity * dt

  def apply_angular_velocity(self, velocity, dt):
    self.__angle = self.__angle + velocity * dt

  def get_position(self):
    return self.__position

  def get_angle(self):
    return self.__angle

  def delta_position(self):
    return self.__position - self.__last_position

sceneParameters = SceneParameters()

robotParameters = RobotParameters()
robotParameters.driveType = 'differential'
robotParameters.collectorQuality

def to_detected_objects(object_type, object_list):
  if object_list == None:
    return []

  detected_objects = []
  if len(object_list) > 0:
    if isinstance(object_list[0], list):
      for o in object_list:
        detected_objects.append(DetectedObject(object_type, o[0] * env_params.meter_scale, o[1], 0))
    else:
      detected_objects.append(DetectedObject(object_type, object_list[0] * env_params.meter_scale, object_list[1], 1))

  return detected_objects

if __name__ == '__main__':
    roverBotSim = VREP_RoverRobot('192.168.43.168', robotParameters, sceneParameters)
    roverBotSim.StartSimulator()

    # nav_viz    = NavViz()
    nav        = Navigator(roverBotSim)
    rover_pose = RoverPose(Vector(0, 0), 0)

    roverBotSim.UpdateObjectPositions()
    roverBotSim.SetTargetVelocities(0, 0)
    roverBotSim.UpdateObjectPositions()

    while True:
      # nav_viz.update()
      sim_update_start = time.time()
      sim_rover_pos, _, _, _ = roverBotSim.UpdateObjectPositions()

      if sim_rover_pos == None:
        continue

      rover_pose.set_position(Vector(sim_rover_pos[0] * env_params.meter_scale, sim_rover_pos[1] * env_params.meter_scale))
      rover_pose.set_angle(sim_rover_pos[5])

      sample, lander, obstacle, rock = roverBotSim.GetDetectedObjects()
      visible_objects = []
      visible_objects = visible_objects + to_detected_objects(EntityType.ROCK,     rock)
      visible_objects = visible_objects + to_detected_objects(EntityType.SAMPLE,   sample)
      visible_objects = visible_objects + to_detected_objects(EntityType.OBSTACLE, obstacle)
      visible_objects = visible_objects + to_detected_objects(EntityType.LANDER,   lander)

      nav_start_time = time.time()
      nav.update(rover_pose, visible_objects)
      nav_update_time = time.time() - nav_start_time
      print('Nav Update Time: {}'.format(nav_update_time))

      # nav_viz.draw(nav.environment(), nav.current_path())

      speed, ori_cor = nav.get_control_parameters()
      # print('Speed: {}, Ori: {}'.format(speed, ori_cor))
      roverBotSim.SetTargetVelocities(speed * 0.05, ori_cor)

      print('Sim update time: {}'.format(time.time() - sim_update_start - nav_update_time))
