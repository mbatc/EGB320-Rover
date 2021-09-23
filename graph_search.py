import queue
import sys


class PrioritizedItem:
  def __init__(self, priority, item):
    self.priority = priority
    self.item     = item

  
  def __gt__(self, other):
      return self.priority > other.priority

  def __lt__(self, other):
      return self.priority < other.priority

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
    return False

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
    self.__frontier  = queue.PriorityQueue()
    self.__explored  = set() # The set of explored nodes
    self.__best      = None # The best node found so far
    self.__best_cost = sys.float_info.max
    self.__goal      = None # The goal that was found

  def expand_frontier(self, node):
    if node in self.__explored:
      return False

    self.__frontier.put(PrioritizedItem(node.cost(), node))
    return True

  def search(self):
    '''
    Perform a step in the GraphSearch.
    Returns true if the goal was found.
    Returns false if the search has not completed. 
    '''

    if self.__goal != None:
      return self.__goal

    if self.__frontier.empty():
      raise Exception("The goal not could not be found")
    item = self.__frontier.get()
    cost = item.priority
    node = item.item

    # Check if this is the best node we have found
    if cost < self.__best_cost:
      self.__best      = node
      self.__best_cost = cost

    # Check if we have found the the goal node
    if node.is_goal():
      self.__goal = node
    else:
      # Add all neighbours of 'node' to the frontier
      for neighbour in node.get_neighbours():
        self.expand_frontier(neighbour)

      # Add 'node' to the explored set
      self.__explored.add(node)

    return self.__goal
