
import math
from typing import Union
# for grid map.
class Position2d:
    def __init__(self, x:int=0, y:int=0):
        # round
        self.x = round(x)
        self.y = round(y)

    def __eq__(self, other:'Position2d'):
        if isinstance(other, Position2d):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        return hash((self.x, self.y))
    
    def __add__(self, other:'Position2d'):
        x = self.x + other.x
        y = self.y + other.y
        return Position2d(x, y)
    
    def __sub__(self, other:'Position2d'):
        x = self.x - other.x
        y = self.y - other.y
        return Position2d(x, y)
    
    def __truediv__(self, other:float):
        if not isinstance(other, Union[float, int]):
            raise RuntimeError(f"unsupport divided type for Position2d:{type(other)}")
        if abs(other) < 1e-10:
            raise ZeroDivisionError()
        x = self.x / other
        y = self.y / other
        return Position2d(x, y)
    
    def __mul__(self, other:float):
        if not isinstance(other, Union[float, int]):
            raise RuntimeError(f"unsupport divided type for Position2d:{type(other)}")
        x = self.x * other
        y = self.y * other
        return Position2d(x, y)
    
    def __rmul__(self, other:float):
        return self * other
    
        
    
    def norm(self, base=2):
        ans = abs(self.x) ** base + abs(self.y) ** base
        return ans ** (1./base)
    
    def __str__(self):
        return f"({self.x}, {self.y})"
    
    def rotate(self, theta:float) -> 'Position2d':
        x_ = math.cos(theta) * self.x + math.sin(theta) * self.y
        y_ = -math.sin(theta) * self.x + math.cos(theta) * self.y
        return Position2d(x_, y_)
        
    
    def angle_x(self) -> float:
        """return the angle between vec:[x, y] and x-axis.

        Returns:
            float: angle, unit:rad
        """
        return math.atan2(self.y, self.x)
        

def euclidean_distance(p1: Position2d, p2: Position2d) -> float:
    """euclidean distance between two points.
    Returns:
        float: (x1 - x2)^2 + (y1 - y2)^2
    """
    return (p1 - p2).norm()
    
    
    