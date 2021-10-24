from ..interop import SCS_ACTION, DetectedObject, ObjectType, Status

from vector_2d import *
from . import pid

from enum import Enum

import queue
import math
import time

cfg = None

def sign(x): return -1 if x < 0 else 1 

class ObjectMap:
  def __init__(self):
    self.__objects = []
    self.__groups  = {}

  @staticmethod
  def is_same(a, b):
    if abs(a.distance - b.distance) > cfg.DIST_THRESH:
      return False
    if abs(a.heading - b.heading) > cfg.HEAD_THRESH:
      return False
    return True

  @staticmethod
  def polar(a):
    return VectorPolar(a.distance, a.heading)

  @staticmethod
  def cartesian(a):
    return ObjectMap.polar(a).to_cartesian()

  @staticmethod
  def distance(a, b):
    return abs(ObjectMap.cartesian(a) - ObjectMap.cartesian(b))

  def objects(self, type=None):
    if type is None:
      return self.__objects
    elif type in self.__groups:
      return self.__groups[type]
    else:
      return []

  def find(self, query):
    for o in self.objects(query.type):
      if ObjectMap.is_same(o, query):
        return o
    return None

  def closest(self, query):
    min_dist = 1000000
    closest  = None
    for o in self.objects(query.type):
      dist = ObjectMap.distance(o, query)
      if dist < min_dist:
        closest = o
        min_dist = dist
    return closest

  def most_recent_of_type(self, type):
    max_time = 0
    recent   = None
    for o in self.objects(type):
      if o.last_detected > max_time:
        recent   = o
        max_time = o.last_detected
    return recent

  def closest_of_type(self, type):
    min_dist = 1000000
    closest  = None
    for o in self.objects(type):
      dist = o.distance
      if dist < min_dist:
        closest = o
        min_dist = dist
    return closest

  def add(self, o):
    if o.type not in self.__groups:
      self.__groups[o.type] = [ o ]
    else:
      self.__groups[o.type].append(o)
    self.__objects.append(o)

  def rem(self, o):
    self.__objects.remove(o)
    self.__groups[o.type].remove(o)

  def update(self, visible):
    # Add/update visible objects
    cur_time = time.time()
    for o in visible:
      o.last_detected = cur_time
      found = self.find(o)
      if found:
        found.type          = o.type
        found.heading       = o.heading
        found.distance      = o.distance
        found.angle         = o.angle
        found.last_detected = o.last_detected
      else:
        self.add(o)
    self.prune()

  def prune(self):
    # Remove old entries
    to_remove = []
    for o in self.objects():
      if time.time() - o.last_detected > cfg.PRUNE_TIME:
        to_remove.append(o)

    for o in to_remove:
      self.rem(o)

class StateTransition(Enum):
  RECORD_STATE = 0,
  CLEAR_STACK  = 1,
  PREVIOUS     = 2,

class State:
  def __init__(self, navigator):
    self.navigator          = navigator
    self.controller         = navigator.controller
    self.map                = navigator.map
    self.move_speed         = cfg.MOVE_SPEED_MED
    self.rotate_speed       = cfg.ROTATE_SPEED_MED
    self.target_dist        = 0
    self.target_head        = 0
    self.state_start_time   = 0
    self.state_first_update = True
    self.obstacle           = None
    self.obstacle_detected_time = 0

  def is_first_update(self):
    return self.state_first_update

  def state_duration(self):
    return time.time() - self.state_start_time

  def start(self):
    self.state_start_time = time.time()

  def avoid_obstacles(self, object_type):
    self.obstacle = self.map.closest_of_type(object_type)
    if self.obstacle is not None:
      self.obstacle_detected_time = time.time()
      print('Avoiding ' + str(self.obstacle))
      limit = math.asin(min(1, cfg.AVOID_RADIUS[object_type] / self.obstacle.distance))
      dist  = self.target_head - self.obstacle.heading
      dir   = sign(dist)

      if abs(dist) < limit:
        self.target_head = self.obstacle.heading + dir * limit
    elif time.time() - self.obstacle_detected_time < cfg.AVOID_MOVE_TIME:
      self.target_head = 0

# ///////////////////////////////////////////////////////////
# DISCOVER OBJECT STATES

class DiscoverBase(State):
  def __init__(self, navigator):
    super().__init__(navigator)

    self.travel = False
    self.rotate = True
    self.active_time = time.time()

  def has_discovered(self, object_type):
    if len(self.map.objects(object_type)) > 0:
      return True

  def discover(self):
    if self.is_first_update():
      self.active_time = time.time()

    if self.rotate:
      self.target_dist = 0
      self.target_head = 1

      if time.time() - self.active_time > cfg.MIN_ROTATE_TIME:
        closest = self.map.closest_of_type(None)
        if closest is None or closest.distance > 40:
          self.rotate = False
          self.active_time = time.time()
          print('Discover travelling')
    else:
      self.target_head = 0
      self.target_dist = 1

      switch_state = time.time() - self.active_time > cfg.MAX_TRAVEL_TIME
      closest = self.map.closest_of_type(None)
      switch_state = switch_state or (closest is None or closest.distance < 20)

      if switch_state:
        self.rotate = True
        self.active_time = time.time()
        print('Discover rotating')

class DiscoverSampleOrRock(DiscoverBase):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST

  def update(self):
    if self.is_first_update():
      self.controller.set_status(Status.SEARCH_SAMPLE)

    # Check finish conditions
    if self.has_discovered(ObjectType.SAMPLE):
      return NavSample(self.navigator), None
    if self.has_discovered(ObjectType.ROCK):
      return NavRock(self.navigator), None
    # Update discovery routine
    self.discover()
    return None, None

class DiscoverSample(DiscoverBase):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST

  def update(self):
    # Check finish conditions
    if self.has_discovered(ObjectType.SAMPLE):
      return NavSample(self.navigator), None
    if self.state_duration() > cfg.DISC_TIMEOUT_SAMPLE:
      return DiscoverSampleOrRock(self.navigator), None
    # Update discovery routine
    self.discover()
    return None, None

class DiscoverObstacle(DiscoverBase):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST

  def update(self):
    # Check finish conditions
    if self.has_discovered(ObjectType.OBSTACLE):
      return NavLander(self.navigator), None
    # Update discovery routine
    self.discover()
    return None, None
    
class DiscoverLander(DiscoverBase):
  def __init__(self, navigator):
    super().__init__(navigator)
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST

  def update(self):
    # Check finish conditions
    if self.has_discovered(ObjectType.LANDER):
      return NavLander(self.navigator), None
    # Update discovery routine
    self.discover()
    return None, None

# ///////////////////////////////////////////////////////////
# NAVIGATE STATES

class ObjectTargetState(State):
  def __init__(self, navigator, target_type):
    super().__init__(navigator)
    self.target_object = None
    self.target_type   = target_type
    self.target_last_detected_time = time.time()

  def update_target(self):
    # Get the most recently detected object of the target type
    most_recent = self.map.most_recent_of_type(self.target_type)
    if most_recent is not None:
      self.target_object = most_recent

    # If we are avoiding an obstacle delay pruning of the target object
    if most_recent is not None or self.obstacle is not None:
      self.target_last_detected_time = time.time()

    # Check if we haven't detected a target for an extended period of time
    if time.time() - self.target_last_detected_time > 10:
      self.target_object = None # We lost the target

class NavRock(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.ROCK)
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST

  def update(self):
    self.update_target()

    if self.target_object is None:
      return DiscoverSampleOrRock(self.navigator), None

    # If we detected a sample we should attempt to navigate to itt
    if len(self.map.objects(ObjectType.SAMPLE)) > 0:
      return DiscoverSample(self.navigator), None

    self.target_dist = self.target_object.distance
    self.target_head = self.target_object.heading

    if self.target_dist < cfg.NAV_DIST_ROCK:
      return FlipRockLookat(self.navigator), None

    self.avoid_obstacles(ObjectType.OBSTACLE)

    return None, None

class NavSample(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.SAMPLE)
    self.move_speed = cfg.MOVE_SPEED_FAST

  def update(self):
    self.update_target()

    if self.target_object is None:
      return DiscoverSample(self.navigator), None

    self.target_dist = self.target_object.distance
    self.target_head = self.target_object.heading

    if self.target_dist < cfg.NAV_DIST_SAMPLE:
      return CollectSampleLookat(self.navigator), None

    self.avoid_obstacles(ObjectType.OBSTACLE)

    return None, None

class NavLander(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.LANDER)
    self.move_speed = cfg.MOVE_SPEED_FAST

  def update(self):
    self.update_target()

    if self.target_object is None:
      return DiscoverLander(self.navigator), None

    self.target_dist = self.target_object.distance
    self.target_head = self.target_object.heading

    if self.target_dist < cfg.NAV_DIST_LANDER:
      return DropSampleLookat(self.navigator), None

    self.avoid_obstacles(ObjectType.OBSTACLE)

    return None, None

# ///////////////////////////////////////////////////////////
# DROP SAMPLE STATES

class DropSampleLookat(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.LANDER)
    self.rotate_speed = cfg.ROTATE_SPEED_MED
    self.move_speed   = 0

  def update(self):
    self.update_target()
    if self.target_object is None:
      return DiscoverSampleOrRock(self.navigator), None

    self.target_head = self.target_object.heading
    self.target_dist = 0

    if abs(self.target_head) < cfg.LOOKAT_THRESH_LANDER:
      return DropSampleApproach(self.navigator), None
      
    return None, None

class DropSampleApproach(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.LANDER)
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = 0

  def update(self):
    self.update_target()

    self.target_head  = 0
    self.target_dist  = 1

    if self.state_duration() > cfg.APPROACH_TIME_LANDER:
      return DropSample(self.navigator), None

    return None, None

class DropSample(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.LANDER)

  def update(self):
    self.controller.perform_action(SCS_ACTION.DROP_SAMPLE)
    return DiscoverSampleOrRock(self.navigator), None

# ///////////////////////////////////////////////////////////
# FLIP ROCK STATES

class FlipRockLookat(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.ROCK)
    self.move_speed   = 0
    self.rotate_speed = cfg.ROTATE_SPEED_MED

  def update(self):
    self.update_target()
    if self.target_object is None:
      return DiscoverSampleOrRock(self.navigator), None

    self.target_dist  = 0
    self.target_head = self.target_object.heading

    if abs(self.target_head) < cfg.LOOKAT_THRESH_SAMPLE:
      return FlipRockApproach(self.navigator), None

    return None, None

class FlipRockApproach(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.ROCK)
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = 0

  def update(self):
    if self.is_first_update():
      self.controller.perform_action(SCS_ACTION.FLIP_ROCK_PREP)
      self.state_start_time = time.time()

    self.target_head  = 0
    self.target_dist  = 1

    if self.state_duration() > cfg.APPROACH_TIME_ROCK:
      return FlipRock(self.navigator), None

    return None, None

class FlipRock(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.ROCK)

  def update(self):
    self.controller.perform_action(SCS_ACTION.FLIP_ROCK)
    return DiscoverSample(self.navigator), None

# ///////////////////////////////////////////////////////////
# COLLECT SAMPLE STATES

class CollectSampleLookat(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.SAMPLE)
    self.rotate_speed = cfg.ROTATE_SPEED_SLOW
    self.move_speed   = 0

  def update(self):
    self.update_target()

    if self.target_object is None:
      return DiscoverSampleOrRock(self.navigator), None

    self.target_head = self.target_object.heading
    self.target_dist = 0

    if abs(self.target_head) < cfg.LOOKAT_THRESH_SAMPLE:
      return CollectSampleApproach(self.navigator), None
      
    return None, None

class CollectSampleApproach(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.SAMPLE)
    self.rotate_speed = cfg.ROTATE_SPEED_SLOW
    self.move_speed   = cfg.MOVE_SPEED_SLOW

  def update(self):
    if self.is_first_update():
      self.controller.set_status(Status.COLLECT_SAMPLE)
      self.controller.perform_action(SCS_ACTION.COLLECT_SAMPLE_PREP)

    self.update_target()

    self.target_head  = 0
    self.target_dist  = 1

    if self.state_duration() > cfg.APPROACH_TIME_SAMPLE:
      return CollectSample(self.navigator), None

    return None, None

class CollectSample(ObjectTargetState):
  def __init__(self, navigator):
    super().__init__(navigator, ObjectType.SAMPLE)

  def update(self):
    self.update_target()
    self.controller.perform_action(SCS_ACTION.COLLECT_SAMPLE)
    self.controller.set_status(Status.SEARCH_LANDER)
    return DiscoverLander(self.navigator), None

# //////////////////////////////////////////////////////////////
# NAVIGATION STATE CONTAINER

class Navigator:
  def __init__(self, controller):
    global cfg

    cfg = controller.config()

    self.pid         = pid.pid_controller(cfg.CONTROL_KP, cfg.CONTROL_KI, cfg.CONTROL_KD)
    self.controller  = controller
    self.map         = ObjectMap()
    self.state       = None
    self.state_stack = queue.LifoQueue()
    self.last_update = time.time()

    controller.travel_position_open()

    self.set_state(DiscoverSampleOrRock(self), None)

  # ///////////////////////////////////////////////////////////
  # NAVIGATOR UPDATE

  def update(self):
    self.controller.update()

    self.map.update(self.controller.get_detected_objects())

    if self.state is not None:
      if self.state.is_first_update():
        self.state.start()
      next_state, transition_type = self.state.update()
      self.state.state_first_update = False
      self.set_state(next_state, transition_type)

    self.apply_motor_speed()

    if next_state is not None:
      time.sleep(cfg.STATE_CHANGE_DELAY) # Wait between state changes

    return next_state

  def apply_motor_speed(self):
    update_time = time.time()
    dt = update_time - self.last_update
    self.last_update = update_time

    if self.state is None:
      self.controller.set_motors(0, 0)
      return

    target_dist  = self.state.target_dist
    target_head  = max(-1, min(1, self.state.target_head / cfg.CONTROL_HEAD_MAX))
    move_speed   = self.state.move_speed
    rotate_speed = self.state.rotate_speed

    vel = 0
    if target_dist != 0:
      vel = move_speed * sign(target_dist)
    ang = self.pid.update(dt, -target_head, 0)
    ang = ang * rotate_speed
    self.controller.set_motors(vel, ang)

  # ///////////////////////////////////////////////////////////
  # STATE MANAGEMENT

  def set_state(self, new_state, transition_type):
    if transition_type == StateTransition.RECORD_STATE and self.state != None:
      self.state_stack.put(self.state)
    if transition_type == StateTransition.CLEAR_STACK:
      self.state_stack = queue.LifoQueue()
    if transition_type == StateTransition.PREVIOUS:
      if self.state_stack.empty():
        return False
      new_state = self.state_stack.get()

    if new_state == None:
      return False

    print('State Changed > {}'.format(new_state))
    self.state = new_state
    self.keep_target = False
    self.state_start_time   = time.time()
    self.state_first_update = True
    return True
