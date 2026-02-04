class GraphNode:
  def __init__(self, x, y, cost=0, prev=None):
    self.x = x
    self.y = y
    self.cost = cost
    self.prev = prev

  def __lt__(self, other):
    return self.cost < other.cost
  
  def __eq__(self, other):
    return self.x == other.x and self.y == other.y
  
  def __hash__(self):
    return hash((self.x, self.y))
  
  def __add__(self, other):
    return GraphNode(self.x + other[0], self.y + other[1])
  
