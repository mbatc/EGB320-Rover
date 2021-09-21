# Import navigation code
from env_params import EntityType
from nav_viz    import NavViz
from navigation import Navigator
from navigation import DetectedObject
from navigation import RoverPose
from geometry   import *

import timeit
import env_params

from roverbot_lib import *

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
        detected_objects.append(DetectedObject(object_type, o[0] * env_params.meter_scale, o[1], 0))
    else:
      detected_objects.append(DetectedObject(object_type, object_list[0] * env_params.meter_scale, object_list[1], 1))

  return detected_objects

if __name__ == '__main__':
    roverBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
    roverBotSim.StartSimulator()

    nav_viz    = NavViz()
    nav        = Navigator(roverBotSim)
    rover_pose = RoverPose(Vector(0, 0), 0)

    roverBotSim.UpdateObjectPositions()
    roverBotSim.SetTargetVelocities(0, 0)
    roverBotSim.UpdateObjectPositions()

    while True:
      nav_viz.update()
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

      # start = time.time()
      nav.update(rover_pose, visible_objects)
      # print('Update Time: {}'.format(time.time() - start))

      nav_viz.draw(nav.environment(), nav.current_path())

      speed, ori_cor = nav.get_control_parameters()

      roverBotSim.SetTargetVelocities(speed * 0.05, ori_cor)
