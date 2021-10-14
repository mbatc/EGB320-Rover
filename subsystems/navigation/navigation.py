from ..interop import SCS_ACTION, DetectedObject, ObjectType

from vector_2d import *

from enum import Enum

import math
import time

cfg = None

class State(Enum):
  DISCOVER_SAMPLE_OR_ROCK = 0,
  DISCOVER_SAMPLE         = 1,
  DISCOVER_OBSTACLE       = 2,
  DISCOVER_LANDER         = 3,
  NAV_ROCK                = 4,
  NAV_SAMPLE              = 5,
  NAV_LANDER              = 7,
  DROP_SAMPLE_LOOKAT      = 8,
  DROP_SAMPLE_APPROACH    = 9,
  DROP_SAMPLE             = 10,
  FLIP_ROCK_LOOKAT        = 11,
  FLIP_ROCK_APPROACH      = 12,
  FLIP_ROCK               = 13,
  COLLECT_SAMPLE_LOOKAT   = 14,
  COLLECT_SAMPLE_APPROACH = 15,
  COLLECT_SAMPLE          = 16,

def sign(x): return -1 if x < 0 else 1 

class Map:
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
    return Map.polar(a).to_cartesian()

  @staticmethod
  def distance(a, b):
    return abs(Map.cartesian(a) - Map.cartesian(b))

  def objects(self, type=None):
    if type is None:
      return self.__objects
    elif type in self.__groups:
      return self.__groups[type]
    else:
      return []

  def find(self, query):
    for o in self.objects(query.type):
      if Map.is_same(o, query):
        return o
    return None

  def closest(self, query):
    min_dist = 1000000
    closest  = None
    for o in self.objects(query.type):
      dist = Map.distance(o, query)
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

class Navigator:
  def __init__(self, controller):
    global cfg
    
    cfg = controller.config()

    self.map                = Map()
    self.target_dist        = 0
    self.target_head        = 0
    self.target_head        = 0
    self.target_object      = None
    self.target_object_type = None
    self.move_speed         = cfg.MOVE_SPEED_MED
    self.rotate_speed       = cfg.ROTATE_SPEED_FAST
    self.state              = None
    self.state_start_time   = 0
    self.state_first_update = False
    self.controller         = controller
    controller.travel_position_open()
    self.set_state(State.DISCOVER_SAMPLE_OR_ROCK)


  # ///////////////////////////////////////////////////////////
  # NAVIGATOR UPDATE

  def update(self):
    self.controller.update()

    self.map.update(self.controller.get_detected_objects())

    next_state = None
    if self.state == State.DISCOVER_SAMPLE_OR_ROCK:
      next_state = self.discover_rock_or_sample()
    if self.state == State.DISCOVER_SAMPLE:
      next_state = self.discover_sample()
    if self.state == State.DISCOVER_OBSTACLE:
      next_state = self.discover_obstacle()
    if self.state == State.DISCOVER_LANDER:
      next_state = self.discover_lander()
    if self.state == State.NAV_ROCK:
      next_state = self.nav_rock()
    if self.state == State.NAV_SAMPLE:
      next_state = self.nav_sample()
    if self.state == State.NAV_LANDER:
      next_state = self.nav_lander()
    if self.state == State.FLIP_ROCK_LOOKAT:
      next_state = self.flip_rock_lookat()
    if self.state == State.FLIP_ROCK_APPROACH:
      next_state = self.flip_rock_approach()
    if self.state == State.FLIP_ROCK:
      next_state = self.flip_rock()
    if self.state == State.COLLECT_SAMPLE_LOOKAT:
      next_state = self.collect_sample_lookat()
    if self.state == State.COLLECT_SAMPLE_APPROACH:
      next_state = self.collect_sample_approach()
    if self.state == State.COLLECT_SAMPLE:
      next_state = self.collect_sample()
    if self.state == State.DROP_SAMPLE_LOOKAT:
      next_state = self.drop_sample_lookat()
    if self.state == State.DROP_SAMPLE_APPROACH:
      next_state = self.drop_sample_approach()
    if self.state == State.DROP_SAMPLE:
      next_state = self.drop_sample()

    self.state_first_update = False
    self.set_state(next_state)
    self.apply_motor_speed()

    if next_state is not None:
      time.sleep(0.5)

    return next_state

  def apply_motor_speed(self):
    vel = 0
    if self.target_dist != 0:
      vel = self.move_speed * sign(self.target_dist)
    ang = self.rotate_speed
    ang = ang * min(max(self.target_head, -cfg.ROTATE_DEAD_ZONE), cfg.ROTATE_DEAD_ZONE) / cfg.ROTATE_DEAD_ZONE
    self.controller.set_motors(vel, ang)

  # ///////////////////////////////////////////////////////////
  # STATE MANAGEMENT

  def clear_target(self):
    self.target_dist   = 0
    self.target_head   = 0
    self.target_object = None

  def set_state(self, new_state):
    if new_state == None:
      return False

    print('State Changed > {}'.format(new_state))
    self.state = new_state
    self.clear_target()
    self.state_start_time   = time.time()
    self.state_first_update = True
    return True

  def this_state_duration(self):
    return time.time() - self.state_start_time


  # ///////////////////////////////////////////////////////////
  # DISCOVER OBJECT STATES

  def discover_rock_or_sample(self):
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST
    if self.navigate_discover_type(ObjectType.SAMPLE):
      return State.NAV_SAMPLE
    if self.navigate_discover_type(ObjectType.ROCK):
      return State.NAV_ROCK
    return None

  def discover_sample(self):
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST
    if self.navigate_discover_type(ObjectType.SAMPLE):
      return State.NAV_SAMPLE

    if self.this_state_duration() > cfg.DISC_TIMEOUT_SAMPLE:
      return State.DISCOVER_SAMPLE_OR_ROCK

    return None

  def discover_lander(self):
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST
    if self.navigate_discover_type(ObjectType.LANDER):
      return State.NAV_LANDER
    return None

  def discover_obstacle(self):
    self.move_speed   = cfg.MOVE_SPEED_FAST
    self.rotate_speed = cfg.ROTATE_SPEED_FAST
    if self.navigate_discover_type(ObjectType.OBSTACLE):
      return State.NAV_LANDER
    return None


  # ///////////////////////////////////////////////////////////
  # NAVIGATE STATES

  def nav_sample(self):
    self.move_speed = cfg.MOVE_SPEED_FAST
    self.update_target_object_by_type(ObjectType.SAMPLE)
    if self.target_object is None:
      return State.DISCOVER_SAMPLE

    self.target_dist = self.target_object.distance
    self.target_head = self.target_object.heading

    if self.target_dist < cfg.NAV_DIST_SAMPLE:
      return State.COLLECT_SAMPLE_LOOKAT

    return None

  def nav_rock(self):
    self.move_speed = cfg.MOVE_SPEED_FAST
    self.update_target_object_by_type(ObjectType.ROCK)
    if self.target_object is None:
      return State.DISCOVER_SAMPLE_OR_ROCK

    # If we detected a sample
    if len(self.map.objects(ObjectType.SAMPLE)) > 0:
      return State.DISCOVER_SAMPLE

    self.target_dist = self.target_object.distance
    self.target_head = self.target_object.heading

    if self.target_dist < cfg.NAV_DIST_ROCK:
      return State.FLIP_ROCK_LOOKAT

    return None

  def nav_lander(self):
    self.move_speed = cfg.MOVE_SPEED_FAST
    self.update_target_object_by_type(ObjectType.LANDER)
    if self.target_object is None:
      return State.DISCOVER_LANDER

    self.target_dist = self.target_object.distance
    self.target_head = self.target_object.heading

    if self.target_dist < cfg.NAV_DIST_LANDER:
      return State.DROP_SAMPLE_LOOKAT

    return None


  # ///////////////////////////////////////////////////////////
  # COLLECT SAMPLE STATES

  def collect_sample_lookat(self):
    self.rotate_speed = cfg.ROTATE_SPEED_SLOW
    self.rotate_speed = cfg.MOVE_SPEED_SLOW
    if self.state_first_update:
      self.controller.perform_action(SCS_ACTION.COLLECT_SAMPLE_PREP)

    self.update_target_object_by_type(ObjectType.SAMPLE)
    if self.target_object is None:
      return State.DISCOVER_SAMPLE_OR_ROCK

    self.target_head = self.target_object.heading
    self.target_dist = 0

    if abs(self.target_head) < cfg.LOOKAT_THRESH_SAMPLE:
      return State.COLLECT_SAMPLE_APPROACH
      
    return None

  def collect_sample_approach(self):
    self.rotate_speed = cfg.ROTATE_SPEED_SLOW
    self.rotate_speed = cfg.MOVE_SPEED_SLOW
    self.target_head = 0
    self.target_dist = 1

    if self.this_state_duration() > cfg.APPROACH_TIME_SAMPLE:
      return State.COLLECT_SAMPLE

    return None

  def collect_sample(self):
    self.controller.perform_action(SCS_ACTION.COLLECT_SAMPLE)
    return State.DISCOVER_LANDER


  # ///////////////////////////////////////////////////////////
  # DROP SAMPLE STATES

  def drop_sample_lookat(self):
    self.update_target_object_by_type(ObjectType.LANDER)
    if self.target_object is None:
      return State.DISCOVER_LANDER

    self.target_head = self.target_object.heading
    self.target_dist = 0

    if abs(self.target_head) < cfg.LOOKAT_THRESH_SAMPLE:
      return State.DROP_SAMPLE_APPROACH
      
    return None

  def drop_sample_approach(self):
    self.move_speed  = cfg.MOVE_SPEED_FAST
    self.target_head = 0
    self.target_dist = 1

    if self.this_state_duration() > cfg.APPROACH_TIME_LANDER:
      return State.DROP_SAMPLE

    return None

  def drop_sample(self):
    self.controller.perform_action(SCS_ACTION.DROP_SAMPLE)
    return State.DISCOVER_SAMPLE_OR_ROCK


  # ///////////////////////////////////////////////////////////
  # FLIP ROCK STATES

  def flip_rock_lookat(self):
    if self.state_first_update:
      self.controller.perform_action(SCS_ACTION.FLIP_ROCK_PREP)

    self.update_target_object_by_type(ObjectType.ROCK)
    if self.target_object is None:
      return State.DISCOVER_SAMPLE_OR_ROCK

    self.target_head = self.target_object.heading
    self.target_dist = 0

    if abs(self.target_head) < cfg.LOOKAT_THRESH_ROCK:
      return State.FLIP_ROCK_APPROACH
      
    return None

  def flip_rock_approach(self):
    self.move_speed  = cfg.MOVE_SPEED_MED
    self.target_head = 0
    self.target_dist = 1

    if self.this_state_duration() > cfg.APPROACH_TIME_ROCK:
      return State.FLIP_ROCK

    return None

  def flip_rock(self):
    self.controller.perform_action(SCS_ACTION.FLIP_ROCK)
    return State.DISCOVER_SAMPLE

  # ///////////////////////////////////////////////////////////
  # GENERIC HELPERS

  def navigate_discover_type(self, object_type=None):
    if len(self.map.objects(object_type)) > 0:
      return True
    self.target_dist = 0
    self.target_head = math.radians(1)
    return False

  def update_target_object_by_type(self, type=None):
    if type is not None and self.target_object is not None:
      if self.target_object.type != type:
        self.target_object.type = None

    most_recent = self.map.most_recent_of_type(type)
    if most_recent is not None:
      self.target_object = most_recent
