# Import navigation code
from subsystems.navigation.env_params import ObjectType
from subsystems.navigation.navigation import Navigator
from subsystems.navigation.geometry   import *
from subsystems.interop               import SCS_ACTION, DetectedObject
from subsystems.navigation            import env_params
from subsystems.vrep.roverbot_lib     import *

# Some options for debugging
print_timing  = False
print_env     = True
visualize_nav = True
move_speed    = 0.015
rotate_speed  = 0.2

first_update = True

if visualize_nav:
  from subsystems.navigation.nav_viz import NavViz

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
        detected_objects.append(DetectedObject(object_type, o[1], o[0] * env_params.meter_scale, 0))
    else:
      detected_objects.append(DetectedObject(object_type, object_list[1], object_list[0] * env_params.meter_scale, 1))

  return detected_objects

if __name__ == '__main__':
    roverBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
    roverBotSim.StartSimulator()

    if visualize_nav:
      nav_viz    = NavViz()

    nav        = Navigator()
    rover_pose = RoverPose(Vector(0, 0), 0)

    roverBotSim.UpdateObjectPositions()
    roverBotSim.SetTargetVelocities(0, 0)
    roverBotSim.UpdateObjectPositions()

    while True:
      sim_update_start = time.time()
      sim_rover_pos, _, _, _ = roverBotSim.UpdateObjectPositions()
      if sim_rover_pos == None:
        continue

      sample, lander, obstacle, rock = roverBotSim.GetDetectedObjects()
      print(sample, lander, obstacle, rock)
      visible_objects = []
      visible_objects = visible_objects + to_detected_objects(ObjectType.ROCK,     rock)
      visible_objects = visible_objects + to_detected_objects(ObjectType.SAMPLE,   sample)
      visible_objects = visible_objects + to_detected_objects(ObjectType.OBSTACLE, obstacle)
      visible_objects = visible_objects + to_detected_objects(ObjectType.LANDER,   lander)

      nav_start_time = time.time()
      nav.update(visible_objects)
      nav_update_time = time.time() - nav_start_time

      scs_action = nav.get_scs_action()

      if scs_action == SCS_ACTION.NONE:
        speed, ori_cor = nav.get_control_parameters()
        roverBotSim.SetTargetVelocities(speed * move_speed, ori_cor * rotate_speed)
      else:
        roverBotSim.SetTargetVelocities(0, 0)
        # if (scs_action == SCS_ACTION.FLIP_ROCK):
        #   roverBotSim.DropSample()
        if (scs_action == SCS_ACTION.DROP_SAMPLE):
          roverBotSim.DropSample()
          if not roverBotSim.SampleCollected():
            nav.complete_scs_action()
        if (scs_action == SCS_ACTION.COLLECT_SAMPLE):
          roverBotSim.CollectSample()
          if roverBotSim.SampleCollected():
            nav.complete_scs_action()

      if visualize_nav:
        nav_viz.update()
        nav_viz.draw(nav.environment(), nav.current_path(), speed, ori_cor, nav.get_routine_type())

      if print_timing:
        print('Nav Update Time: {}'.format(nav_update_time))
        print('Sim update time: {}'.format(time.time() - sim_update_start - nav_update_time))

      if print_env:
        print('-- Environment --')
        print(nav.environment())