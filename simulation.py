# Import navigation code
from nav_viz    import NavViz
from Navigation import Navigator
from geometry   import *
import traceback

from roverbot_lib import *

sceneParameters = SceneParameters()

robotParameters = RobotParameters()
robotParameters.driveType = 'differential'

if __name__ == '__main__':
  # try:
    roverBotSim = VREP_RoverRobot('127.0.0.1', robotParameters, sceneParameters)
    roverBotSim.StartSimulator()

    nav_viz = NavViz()
    nav = Navigator()

    nav.on_target_reached('lander', roverBotSim.DropSample)
    nav.on_target_reached('sample', roverBotSim.CollectSample)
    nav.on_target_reached('rock',   roverBotSim.CollectSample)
    nav.environment.add('lander', Circle(Vector(0,0), 0.4))

    while True:
      nav_viz.update()
      # roverBotSim.SetTargetVelocities(0.1, 10)
      robotPose, _, _, _ = roverBotSim.UpdateObjectPositions()

      if robotPose == None:
        continue

      # For now, explicitly set the robots location in the world
      nav.set_rover(Vector(robotPose[0], robotPose[1]), robotPose[5])

      sample, lander, obstacle, rock = roverBotSim.GetDetectedObjects()
      if (rock     != None): nav.record_object_list('rock',     rock)
      if (sample   != None): nav.record_object_list('sample',   sample)
      if (obstacle != None): nav.record_object_list('obstacle', obstacle)
      if (lander   != None): nav.record_object('lander', lander[0], lander[1])

      if nav.target == None:
        if roverBotSim.SampleCollected():
          if nav.pick_target('lander'):
            print('Nav To [lander] ' + str(nav.target.body.position))
        else:
          for target_type in [ 'sample', 'rock' ]:
            if nav.pick_target(target_type):
              print('Nav To [' + target_type + '] ' + str(nav.target.body.position))
              break

          if nav.target == None:
            nav.explore()
            print('Exploring To ' + str(nav.target.body.position))

      nav_viz.draw(nav.environment, nav.current_path)
      nav.update_path()

      print(robotPose[5], str(nav.get_direction()), str(nav.get_dir_correction()))

      roverBotSim.SetTargetVelocities(0.01 + nav.get_speed() * 0.03, nav.get_dir_correction())

  # except Exception as e:
  #   traceback.print_exc()
  #   roverBotSim.StopSimulator()
