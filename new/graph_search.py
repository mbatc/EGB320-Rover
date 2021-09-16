import queue
import sys
from dataclasses import dataclass, field
from typing import Any

@dataclass(order=True)
class PrioritizedItem:
    priority: int
    item: Any=field(compare=False)

class Node:
  def __init__(self):
    pass

  def cost(self):
    '''
    Calculate the cost of traversing to this Node given the path taken.
    '''
    pass

  def is_goal(self):
    '''
    Test if this PathNode is the goal.
    '''
    return 

  def path(self):
    '''
    Get the path to this node.
    '''
    return [ self ]

  def get_neighbours(self):
    '''
    We don't know how to expand the frontier, just return an empty list
    '''
    return []

  def __hash__(self):
    '''
    The derived class is required to implement a Hash function.
    '''
    return NotImplemented

  def __eq__(self, other):
    '''
    The derived class is required to implement a Comparison function.
    '''
    return NotImplemented

class GraphSearch:
  '''
  Performs a simple graph search using on the root node specified.
  '''
  def __init__(self):
    self.frontier  = queue.PriorityQueue()
    self.explored  = set() # The set of explored nodes
    self.best      = None # The best node found so far
    self.best_cost = sys.float_info.max
    self.goal      = None # The goal that was found

  def search(self):
    '''
    Perform a step in the GraphSearch.
    Returns true if the goal was found.
    Returns false if the search has not completed. 
    '''

    if self.frontier.empty():
      raise Exception("The goal not could not be found")
    cost, node = self.frontier.get()

    # Check if this is the best node we have found
    if cost < self.best_cost:
      self.best      = node
      self.best_cost = cost

    # Check if we have found the the goal node
    if node.is_goal():
      self.goal = node
    else:
      # Add all neighbours of 'node' to the frontier
      for neighbour in node.get_neighbours():
        if neighbour not in self.explored:
          self.frontier.put(PrioritizedItem(neighbour.cost(), neighbour))

      # Add 'node' to the explored set
      self.explored.add(node)

    return self.goal != None
