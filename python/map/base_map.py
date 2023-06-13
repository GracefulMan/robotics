from abc import ABC
from typing import List
import enum
from threading import RLock
import numpy as np
import logging
import sys

sys.path.append('/Users/bytedance/Desktop/robotics/python')
from common.type_define import *
from common.map_tools import load_image
from common.logger import logger
from common.pose2d import Position2d
class MapValue(enum.Enum):
    FREE = 0
    WARNING = 5
    SAFETY = 10
    OBSTACLE = 20
    

    def __int__(self):
        return self.value

    def __lt__(self, other):
        if isinstance(other, MapValue):
            return self.value < other.value
        elif isinstance(other, Union[int, float]):
            return self.value < other
        else:
            return NotImplemented

    def __le__(self, other):
        if isinstance(other, MapValue):
            return self.value <= other.value
        elif isinstance(other, Union[int, float]):
            return self.value <= other
        else:
            return NotImplemented

    def __eq__(self, other):
        if isinstance(other, MapValue):
            return self.value == other.value
        elif isinstance(other, Union[int, float]):
            return self.value == other
        else:
            return NotImplemented

    def __ne__(self, other):
        if isinstance(other, MapValue):
            return self.value != other.value
        elif isinstance(other, Union[int, float]):
            return self.value != other
        else:
            return NotImplemented

    def __gt__(self, other):
        if isinstance(other, MapValue):
            return self.value > other.value
        elif isinstance(other, Union[int, float]):
            return self.value > other
        else:
            return NotImplemented

    def __ge__(self, other):
        if isinstance(other, MapValue):
            return self.value >= other.value
        elif isinstance(other, Union[int, float]):
            return self.value >= other
        else:
            return NotImplemented
class GridMap(ABC):
    map_lock = RLock()
    def __init__(self, length:int=200, width:int=200, resolution:float=0.05) -> None:
        """_summary_
        ---------------y------------------------->    
        |    -----------------------------
        |    |                           |
        |    |                           |
        x    |          MAP              | width, 
        |    |                           |
        |    -----------------------------
        |           length
        |
        Args:
            length (int, optional): the length of map in grid map. Defaults to 200.
            width (int, optional): the width of map in grid map. Defaults to 200.
            resolution(double, optional): the resolution of map, for grid map, the real size in world represented by each cell's length is resolution. 
        """    
        super().__init__()
        self._resolution = resolution
        self._width = width
        self._length = length
        self._grid_map = np.zeros((width, length), dtype=np.uint8)
    
    def width(self): return self._width
    def length(self): return self._length   
    
    
    def map(self) -> np.ndarray:
        return self._grid_map
    
    def get_value(self, x:int, y:int) -> MapValue:
        if self._valid_idx(x, y):
            return self._grid_map[x, y]
        raise ValueError()
    
    def get_pos_value(self, p:Position2d) -> MapValue:
        return self.get_value(p.x, p.y)
    
    def set_value(self, x:int, y:int, val:MapValue):
        self._valid_idx(x, y)
        self._grid_map[x, y] = val
    
    def loadmap_from_mat(self, mat:np.ndarray, resolution:float)->bool:    
        if isinstance(mat.dtype, Value):
            logger.fatal("the dtype of map is not integer type")
            return False
        if len(mat.shape) != 2: return False
        self._width = mat.shape[0]
        self._length = mat.shape[1]
        self._resolution = resolution
        mat[mat == 0] = MapValue.OBSTACLE
        mat[mat == 1] = MapValue.FREE
        self._grid_map = mat
        print(self._grid_map.shape)
        return True
    
    def loadmap_from_image(self, filepath:str):
        mat, resolution = load_image(filepath=filepath)
        success = self.loadmap_from_mat(mat, resolution)
        logger.info(f'load map from image {"success" if success else "failed"}')
        if success:
            logger.info(f"map info: length:{self.length()}, width:{self.width()}")
       
    def _valid_idx(self, x:int, y:int) -> bool:
        valid = (isinstance(x, Value) and isinstance(y, Value) and x >= 0 and x < self.width() and y >= 0 and y < self.length())
        return valid   

    def can_reached(self, p:Position2d):
        if not self._valid_idx(p.x, p.y): return False
        val = self.get_pos_value(p)
        return val < MapValue.OBSTACLE
        
    def get_valid_neighbor(self, p:Position2d) -> List[Position2d]:
        ans = []
        left = Position2d(p.x - 1, p.y)
        right = Position2d(p.x + 1, p.y)
        up = Position2d(p.x, p.y - 1)
        down = Position2d(p.x, p.y + 1)
        if self.can_reached(left): ans.append(left)
        if self.can_reached(right): ans.append(right)
        if self.can_reached(up): ans.append(up)
        if self.can_reached(down): ans.append(down)
        return ans
    
        
        