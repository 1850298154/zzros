
import shapely
from shapely.geometry import Point, Polygon

from typing import List, Tuple, Dict, Union, Callable

from src                            import coord 
from src                            import tracker

class Scout_Area:
    def __init__(self) -> None:
        self.clear()
        pass
    
    def initialize(self) -> None:
        pass
    
    def start(self) -> None:
        pass
    
    def clear(self) -> None:
        #################### from gcs json #################### 
        self.areas_id                       = None      #   "int",
        self.areas_type                     = None      #   " 0:城镇city  1:森林forest  2:平原plain  3:山地mountain",
        self.area_target_num                = None      #   "区域预计目标总数",
        self.area_fire_num                  = None      #   "区域预计火力点总数",
        self.area_point_num                 = None      #   "本区域点总数",
        self.area_point_content             = None      #   
        """
        "area_point_content" : [
                            {
                                "lon": "多边形点经度（double）",
                                "lat": "多边形点纬度（double）",
                                "alt": "多边形点高度（double）"
                            },
                            {
                                "lon": "多边形点经度（double）",
                                "lat": "多边形点纬度（double）",
                                "alt": "多边形点高度（double）"
                            },
                            {
                                "lon": "多边形点经度（double）",
                                "lat": "多边形点纬度（double）",
                                "alt": "多边形点高度（double）"
                            }
                        ]
        """
        #################### from gcs json #################### 
        self.area_point_content_xy_list     = None      # len==7  #  [[100, 100], [200, 100], [300, 150], [375, 225], [400, 300], [200, 250], [100, 150]]
        self.polygon:Polygon                = None
        self.centroid:Point                 = None
        self.center_x:float                 = None
        self.center_y:float                 = None
        self.center_y:float                 = None
        self.areasize:float                 = None
        #################### from run config configuration_process_run #################### 
        self.full_route_xyz                 = None
        self.full_route_gps                 = None
        #################### from run config sub_configuration_hierarchical_clustering #################### 
        self.tasker_drones_assigned_key_num         = None
        self.tasker_drones_assigned_key_list    = None
        self.surveil_drones_assigned_key_num        = None
        self.surveil_drones_assigned_key_list   = None
        self.relay_drones_assigned_key_num          = None
        self.relay_drones_assigned_key_list     = None
        pass

    def mount_run_config_sub_run_init_area(self, ) -> None:
        #################### from run config sub_run_init_scout_area #################### 
        self.area_point_content_xy_list     = []
        if len(self.area_point_content) < 3:
            tracker.debug_tracker.instance.current_position_print()
            raise ValueError("Invalid input: area_point_content must be greater than 3 points.")

        for gps in self.area_point_content:
            x, y = coord.ned.instance.gps_to_xy(lat=gps['lat'], lon=gps['lon'])
            self.area_point_content_xy_list.append(
                [x, y]
            )
        self.polygon:Polygon    = Polygon(self. area_point_content_xy_list)
        self.centroid:Point     = self.polygon.centroid
        self.center_x:float     = self.polygon.centroid.x
        self.center_y:float     = self.polygon.centroid.y
        self.center_y:float     = self.polygon.centroid.y
        self.areasize:float     = self.polygon.area
        if self.areasize <= 0:
            tracker.debug_tracker.instance.current_position_print()
            raise ValueError("Invalid input: self.areasize <= 0")
        #################### from run config configuration_process_run #################### 
        self.full_route_xyz     = [] # 一维列表存放字典
        self.full_route_gps     = [] # 一维列表存放字典
        pass




scan_width = None
areas_id_2_scout_area_dict: Dict[int, Scout_Area] = {}

