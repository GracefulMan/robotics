from map.base_map import GridMap

class GlobalMap(GridMap):
    def __new__(cls, *args, **kwargs):
        with GlobalMap.map_lock:
            if not hasattr(GridMap,"_instance"):
                GlobalMap._instance = object.__new__(cls, *args, **kwargs)
        return GlobalMap._instance
    
