from geometry import *

# Constructs for representing the environment

class Entity:
  def __init__(self, entity_type, body):
    self.type = entity_type
    self.body = body

  def position(self):
    return self.body.position

  def distance(self, point:Vector):
    return sqrt(self.distance_sqr(point))

  def distance_sqr(self, point:Vector):
    return vec2_mag_sqr(point - self.body.closest_point(point))

class Environment:
  def __init__(self, precision = 1):
    self.entities    = []
    self.type_lookup = {}
    self.precision   = 1

  def get_extents(self):
    '''
    Get the bounds of the rovers environment
    '''
    extents = Rect.smallest()
    for entity in self.entities:
      extents.grow_to_contain(entity.body.extents())
    return extents

  def add(self, entity_type, body):
    # Remove existing entities of the same type that overlap with the new body
    for existing in self.get_colliding(body, entity_type):
      self.remove(existing)

    new_entity = Entity(entity_type, body)
    # Add to the list of all entities
    self.entities.append(new_entity)
    # Add to an entity group by type
    if entity_type in self.type_lookup:
      self.type_lookup[entity_type].append(new_entity)
    else:
      self.type_lookup[entity_type] = [ new_entity ]

  def remove(self, entity):
    try:
      # Remove from the full list, and type specific list
      self.entities.remove(entity)
      self.type_lookup[entity.type].remove(entity)
      return True
    except ValueError as e:
      return False # Handle not in list exception, just return false

  def get_entities(self, entity_type=None):
    if entity_type == None:
      return self.entities
    return self.type_lookup[entity_type] if entity_type in self.type_lookup else []

  def get_colliding(self, body, entity_type=None):
    collisions = []
    for entity in self.get_entities(entity_type):
      if body is not entity.body and intersect(entity.body, body):
        collisions.append(entity)
    return collisions

  def get_first_colliding(self, body, entity_type=None):
    for entity in self.get_entities(entity_type):
      if body is not entity.body and intersect(entity.body, body):
        return entity
    return None
