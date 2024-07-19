
from typing import List, Tuple, Dict, Union, Callable


from src                                import coord 

class Target:
    def __init__(self) -> None:
        self.clear()
        pass
    
    def initialize(self) -> None:
        self.before_scan = True        # 是否在扫描之前，如果是，则按照起飞。不是按照降落高度飞行。
        pass
    
    def start(self) -> None:
        pass
    
    def clear(self) -> None:
        #################### from gcs json #################### 
        self.id                 = None 
        self.expect_num         = None 
        self.lon                = None  # "经度（double）",
        self.lat                = None  # "纬度（double）",
        self.alt                = None  # "高度（double）"
        #################### from Task_Parser sub_parse_mission0 #################### 
        self.x                  = None  # "当前x（double）",
        self.y                  = None  # "当前y（double）",
        self.z                  = None  # "当前高度（double）"
        #################### from Split_In_And_Out sub_execution_target_consumption #################### 
        self.dist_cost          = None  
        pass

    def mount_parser_sub_parse_mission0(self, ) -> None:
        #################### from Task_Parser sub_parse_mission0 #################### 
        x, y = coord.ned.instance.gps_to_xy(lat=self.lat, lon=self.lon)
        self.x      = x
        self.y      = y
        self.z      = self.alt
        pass



target_id_2_target:Dict[int, Target] = {}

tasker_key_point_xylist_list_poollist = [] # key_point_list
