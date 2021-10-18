import math

from subsystems.interop import ObjectType

STATE_CHANGE_DELAY = 1.5 # Seconds

MIN_ROTATE_TIME  = 8
MAX_TRAVEL_TIME  = 5

DIST_THRESH      = 10
HEAD_THRESH      = math.radians(5)
PRUNE_TIME       = 0.75

ROTATE_DEAD_ZONE = math.radians(2)

ROTATE_SPEED_FAST = 0.5
ROTATE_SPEED_MED  = 0.25
ROTATE_SPEED_SLOW = 0.15

MOVE_SPEED_FAST  = 0.3
MOVE_SPEED_MED   = 0.2
MOVE_SPEED_SLOW  = 0.15

NAV_DIST_ROCK   = 25
NAV_DIST_SAMPLE = 17
NAV_DIST_LANDER = 30

LOOKAT_THRESH_SAMPLE = math.radians(1.5)
LOOKAT_THRESH_ROCK   = math.radians(1.5)
LOOKAT_THRESH_LANDER = math.radians(1.5)

APPROACH_TIME_ROCK   = 2.5
APPROACH_TIME_SAMPLE = 4
APPROACH_TIME_LANDER = 6

DISC_TIMEOUT_SAMPLE  = 10 

AVOID_MOVE_TIME = 3

AVOID_DISTANCE = {}
AVOID_HEADING  = {}

AVOID_DISTANCE[ObjectType.OBSTACLE] = 30
AVOID_HEADING [ObjectType.OBSTACLE] = math.radians(10)

AVOID_DISTANCE[ObjectType.ROCK]     = 25
AVOID_HEADING [ObjectType.ROCK]     = math.radians(10)

AVOID_DISTANCE[ObjectType.SAMPLE]   = 10
AVOID_HEADING [ObjectType.SAMPLE]   = math.radians(5)

AVOID_DISTANCE[ObjectType.LANDER]   = 30
AVOID_HEADING [ObjectType.LANDER]   = math.radians(15)
