
from typing import get_args
from enum import *

def weighted_average(a, b, a_w, b_w):
  return (a * a_w + b * b_w) / (a_w + b_w)

class Entity:
  def __init__(self, entity_type, position, angle, confidence):
    self.__entity_type = entity_type
    self.__position    = position
    self.__angle       = angle
    self.__confidence  = confidence

  def update_position(self, new_pos, new_angle, new_confidence):
    '''
    Update the entities position and angle using a weighted average of the current confidence,
    and the new confidence.
    '''
    self.__position   = weighted_average(self.position(), new_pos, self.confidence(), new_confidence)
    self.__position   = weighted_average(self.angle(), new_angle, self.confidence(), new_confidence)
    self.__confidence = (new_confidence + self.confidence()) * 0.5

  def type(self):
    return self.__entity_type

  def position(self):
    return self.__position

  def angle(self):
    return self.__angle

  def confidence(self):
    '''
    Return a value indicating how accurate to consider the Entities
    position/angle.
    '''
    return self.__confidence

  def extents(self):
    '''
    Return the extents of the entity
    '''
    raise Exception("Not implemented")

class Environment:
  def __init__(self):
    self.entities = []
    self.groups   = {}

  def get_group(self, entity_type):
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

  def update_entity(self, entity_type, position, angle, confidence):
    '''
    Attempt to update an existing entity in the environment map.
    Returns True if an entity was successfully updated. 
    Returns False if no entitiy was updated.
    '''
    pass

  def add_entity(self, entity_type, position, angle, confidence):
    '''
    Add a new entity to the map.
    '''
    new_entity = Entity(entity_type, position, angle, confidence)
    group = self.get_group(entity_type)

    group.append(new_entity)
    self.entities.append(new_entity)

  def add_or_update_entity(self, entity_type, position, angle, confidence):
    '''
    Attempt to update an existing entity. If updating fails, add a new entity to
    the environment.
    '''
    if not self.update_entity(entity_type, position, angle, confidence):
      self.add_entity(entity_type, position, angle, confidence)

  def select_entity(self, entity_type):
    '''
    Find an entity in the map of the selected type
    '''
    group = self.get_group(entity_type)
      


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