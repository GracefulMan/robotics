from abc import ABC
import enum
from threading import RLock
import numpy as np


class MapValue(enum.Enum):
    FREE = 0
    WARNING = 5
    SAFEITY = 10
    # if the value great then SAFEITY, then it's dangous.
    OBSTACLE = 20

class GridMap(ABC):
    map_lock = RLock()
    def __init__(self, length=200, width=200, resolution=0.05) -> None:
        """_summary_

        Args:
            length (int, optional): the length of map in grid map. Defaults to 200.
            width (int, optional): the width of map in grid map. Defaults to 200.
            resolution(double, optional): the resolution of map, for grid map, the real size in world represented by each cell's length is resolution. 
        """    
        super().__init__()
        self._resolution = resolution
        self._width = width
        self._length = length
        self._grid_map = np.zeros((length, width), dtype=np.uint8)
        
    def __new__(cls, *args, **kwargs):
        """singleton instance.
        Returns:
            self: 
        """
        with GridMap.map_lock:
            if not hasattr(GridMap,"_instance"):
                GridMap._instance = object.__new__(cls, *args, **kwargs)
        return GridMap._instance
    
    def map(self) -> np.ndarray:
        return self._grid_map
    
    
    
if __name__ == '__main__':
    map1 = GridMap()
    map2 = GridMap()
    print(map1 == map2)
    print(map1.map())
    
        
    