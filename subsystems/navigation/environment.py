
from sys import float_info
# from typing import get_args
from enum     import *
from .geometry import *

from . import env_params
import math

def weighted_average(a, b, a_w, b_w):
  return (a * a_w + b * b_w) / (a_w + b_w)

class Entity:
  def __init__(self, entity_type, position, angle, confidence, body:Body):
    self.__entity_type = entity_type
    self.__position    = position
    self.__angle       = angle
    self.__confidence  = confidence
    self.__body        = body
    self.missing_time  = 0

  def update_position(self, new_pos, new_angle, new_confidence):
    '''
    Update the entities position and angle using a weighted average of the current confidence,
    and the new confidence.
    '''
    self.__position   = weighted_average(self.position(), new_pos, self.confidence(), new_confidence)
    self.__angle      = weighted_average(self.angle(), new_angle, self.confidence(), new_confidence)
    self.__confidence = (new_confidence + self.confidence()) * 0.5

  def set_position(self, new_pos):
    self.__position = new_pos

  def set_angle(self, new_angle):
    self.__angle = new_angle

  def body(self):
    return self.__body

  def type(self):
    return self.__entity_type

  def position(self):
    return self.__position

  def direction(self):
    return VectorPolar(1, self.__angle).to_cartesian()

  def angle(self):
    return self.__angle

  def confidence(self):
    '''
    Return a value indicating how accurate to consider the Entities
    position/angle.
    '''
    return self.__confidence

  def intersects(self, other):
    my_transformed_body    = self.transformed_body()
    other_transformed_body = other.transformed_body()
    return intersect(my_transformed_body, other_transformed_body)

  def intersects_inflated(self, other):
    my_transformed_body    = self.inflated_body()
    other_transformed_body = other.inflated_body()
    return intersect(my_transformed_body, other_transformed_body)

  def inflated_body(self):
    inflation = env_params.entity_info[self.type()].inflation()
    return self.__body.scale(1 + inflation * (1 - self.confidence())).transformed(self.__position, self.__angle)

  def transformed_body(self):
    return self.__body.transformed(self.__position, self.__angle)

  def size(self, inflated = False):
    if inflated:
      return self.inflated_body().extents().size()
    else:
      return self.__body.extents().size()

  def extents(self):
    '''
    Return the extents of the entity
    '''
    raise Exception("Not implemented")

class Environment:
  def __init__(self):
    self.entities = []
    self.groups   = {}
    self.__rover  = None

  def get_group(self, entity_type=None):
    '''
    Get a group of entities by type.

    If the group does not exist, a new empty group is added.
    If entity_type is None, all entities are returned.
    '''
    if entity_type == None:
      return self.entities
    if entity_type not in self.groups:
      self.groups[entity_type] = []
    return self.groups[entity_type]

  def has_entities(self, entity_type):
    return entity_type in self.groups and len(self.groups[entity_type]) > 0

  def update_entity(self, entity_type, position, angle, confidence):
    '''
    Attempt to update an existing entity in the environment map.
    Returns True if an entity was successfully updated. 
    Returns False if no entitiy was updated.
    '''
    new_entity = Entity(entity_type, position, angle, confidence, Circle(Vector(0, 0), env_params.entity_info[entity_type].size() / 2, 0))
    existing   = self.find_first_colliding(new_entity, entity_type, True)
    if existing == None:
      return None
    existing.update_position(position, angle, confidence)
    return existing

  def get_rover(self):
    return self.__rover

  def find_colliding(self, entity, entity_type = None, inflated = False):
    found = []
    if isinstance(entity_type, list):
      for t in entity_type:
        found = found + self.find_colliding(entity, t, inflated)
    else:
      for other in self.get_group(entity_type):
        if entity is not other:
          if inflated:
            if entity.intersects_inflated(other):
              found.append(other)
          else:
            if entity.intersects(other):
              found.append(other)
    return found


  def find_first_colliding(self, entity, entity_type = None, inflated = False):
    if isinstance(entity_type, list):
      for t in entity_type:
        other = self.find_first_colliding(entity, t, inflated)
        if other is not None:
          return other
    else:
      for other in self.get_group(entity_type):
        if not env_params.entity_info[other.type()].collidable():
          continue
        if entity is not other:
          if inflated:
            if entity.intersects_inflated(other):
              return other
          else:
            if entity.intersects(other):
              return other
    return None

  def add_entity(self, entity_type, position, angle, confidence):
    '''
    Add a new entity to the map.
    '''
    body       = Circle(Vector(0, 0), env_params.entity_info[entity_type].size() / 2)
    new_entity = Entity(entity_type, position, angle, confidence, body)

    return self.add_entity_instance(new_entity)

  def add_entity_instance(self, new_entity):
    group = self.get_group(new_entity.type())
    group.append(new_entity)
    self.entities.append(new_entity)

    if new_entity.type() == env_params.ObjectType.ROVER:
      self.__rover = new_entity

    return new_entity

  def add_or_update_entity(self, entity_type, position, angle, confidence):
    '''
    Attempt to update an existing entity. If updating fails, add a new entity to
    the environment.
    '''
    new_entity = self.update_entity(entity_type, position, angle, confidence)
    if new_entity == None:
      new_entity = self.add_entity(entity_type, position, angle, confidence)
    return new_entity

  def bring_to_center(self, entity:Entity):
    '''
    Make an entity the center of the map. This will position all other entities relative
    to this entity. This entities position/orientation
    '''
    offset = entity.position()
    for other in self.entities:
      # TODO: Perhaps adjust the confidence when we shift the entities.
      # TODO: Might be worthwhile also removing rotation.
      other.set_position(other.position() - offset)

  def find_closest(self, entity_type, position):
    '''
    Find the entity that is closest to the 'position' specified.
    '''
    closest_dist   = float_info.max
    closest_entity = None
    if entity_type is list:
      for type in entity_type:
        entity, dist = self.find_closest(type, position)
        if dist < closest_dist:
          closest_entity = entity
          closest_dist   = dist
    else:
      for entity in self.get_group(entity_type):
        dist = vec2_mag_sqr(entity.position() - position)
        if dist < closest_dist:
          closest_entity = entity
          closest_dist   = dist
      closest_dist = sqrt(closest_dist)

    return closest_entity, closest_dist

  def combine_overlapping(self):
    for group in self.groups.values():
      for entity in group.copy():
        self.remove(entity)
        updated = self.update_entity(entity.type(), entity.position(), entity.angle(), entity.confidence())
        if updated == None:
          self.add_entity_instance(entity)

  def prune_visible(self, dt, visible_entities, cam_pos, cam_dir, cam_fov, min_dist, max_dist, entity_type=None):
    '''
    This function will prune invalid entities from the map based on
    the visibility of other entities and the camera properties.

    If an entity is in-front of a visible entity (and should be visible),
    but is not in the visible_entities list, it will be removed.
    '''
    unit_cam_dir       = cam_dir.unit()
    visible_candidates = []
    direction_limit = math.cos(radians(cam_fov / 2))

    min_dist_sqr = min_dist * min_dist
    max_dist_sqr = max_dist * max_dist

    to_remove    = set()
    # Find potential visible entities
    for entity in self.get_group(entity_type):
      to_entity  = entity.position() - cam_pos
      entity_dir = to_entity.unit()

      if vec2_mag_sqr(to_entity) < min_dist_sqr:
        continue

      in_view   = vec2_dot(entity_dir, unit_cam_dir) > direction_limit
      if in_view:
        visible_candidates.append(entity)

    # Check for entities that should be occluding the 'visible' entities
    # They need to be removed
    for entity in visible_entities:
      entity.missing_time = 0
      for occluder in visible_candidates:
        if entity is occluder:
          continue
        to_occluder = Line(cam_pos, occluder.position())
        to_entity   = Line(cam_pos, entity.position())
        is_in_front = to_entity.length_sqr() > to_occluder.length_sqr()
        if is_in_front and intersect(to_entity, occluder.body()):
          # entity should be occluded by occluder, so occluder should be
          # visible. Remove occluder from the map.
          to_remove.add(occluder)

    # Check for potentially visible entities that should not be occluded the 'visible' entities
    # They need to be removed
    for entity in visible_candidates:
      is_occluded = False
      for occluder in visible_entities:
        if entity is occluder:
          is_occluded = True
          break

        to_entity   = Line(cam_pos, entity.position())
        to_occluder = Line(cam_pos, occluder.position())
        is_in_front = to_entity.length_sqr() > to_occluder.length_sqr()
        if is_in_front and intersect(to_entity, occluder.body()):
          is_occluded = True
          break

      if not is_occluded:
        to_remove.add(entity)

    for entity in to_remove:
      entity_dist_sqr  = vec2_mag_sqr(entity.position() - cam_pos)
      if entity_dist_sqr > max_dist_sqr or entity_dist_sqr < min_dist_sqr:
        continue

      entity.missing_time = entity.missing_time + dt
      if entity.missing_time > 2:
        self.remove(entity)
        print('Pruned: ' + str(entity))


  def remove(self, entity:Entity):
    '''
    Remove the specified entity from the environment.
    Returns True on success.
    Returns False if the entity is not in the map, or another error occurs.
    '''
    try:
      self.entities.remove(entity)
      group = self.get_group(entity.type())
      group.remove(entity)
      return True
    except:
      return False

  def __str__(self):
    return '\n'.join([str(entity) for entity in self.entities])

  def __contains__(self, entity):
    if not isinstance(entity, Entity):
      return False
    group = self.get_group(entity.type())
    return entity in group