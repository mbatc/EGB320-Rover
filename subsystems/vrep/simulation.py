# Import navigation code
from ..navigation.env_params import ObjectType
from ..navigation.navigation import Navigator
from ..navigation.geometry   import *
from ..interop               import DetectedObject
from ..navigation            import env_params

from roverbot_lib import *

# Some options for debugging
print_timing  = False
visualize_nav = True
move_speed    = 0.03
rotate_speed  = 1

first_update = True

if visualize_nav:
  from nav_viz import NavViz

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
    roverBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
    roverBotSim.StartSimulator()

    if visualize_nav:
      nav_viz    = NavViz()

    nav        = Navigator(roverBotSim)
    rover_pose = RoverPose(Vector(0, 0), 0)

    roverBotSim.UpdateObjectPositions()
    roverBotSim.SetTargetVelocities(0, 0)
    roverBotSim.UpdateObjectPositions()

    while True:
      sim_update_start = time.time()
      sim_rover_pos, _, _, _ = roverBotSim.UpdateObjectPositions()

      if sim_rover_pos == None:
        continue

      # if first_update:
      #   input() # Wait for some input
      #   first_update = False

      rover_pose.set_position(Vector(sim_rover_pos[0] * env_params.meter_scale, sim_rover_pos[1] * env_params.meter_scale))
      rover_pose.set_angle(sim_rover_pos[5])

      sample, lander, obstacle, rock = roverBotSim.GetDetectedObjects()
      visible_objects = []
      visible_objects = visible_objects + to_detected_objects(ObjectType.ROCK,     rock)
      visible_objects = visible_objects + to_detected_objects(ObjectType.SAMPLE,   sample)
      visible_objects = visible_objects + to_detected_objects(ObjectType.OBSTACLE, obstacle)
      visible_objects = visible_objects + to_detected_objects(ObjectType.LANDER,   lander)

      nav_start_time = time.time()
      nav.update(rover_pose.delta_position(), rover_pose.get_angle(), visible_objects)
      nav_update_time = time.time() - nav_start_time

      speed, ori_cor = nav.get_control_parameters()
      # print('Speed: {}, Ori: {}'.format(speed, ori_cor))
      roverBotSim.SetTargetVelocities(speed * move_speed, ori_cor * rotate_speed)

      if visualize_nav:
        nav_viz.update()
        nav_viz.draw(nav.environment(), nav.current_path(), speed, ori_cor, nav.get_routine_type())

      if print_timing:
        print('Nav Update Time: {}'.format(nav_update_time))
        print('Sim update time: {}'.format(time.time() - sim_update_start - nav_update_time))
