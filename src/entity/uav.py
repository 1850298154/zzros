

from typing import List, Tuple, Dict, Union, Callable


from src                                import coord 
from src                            import protocol

class UAV:
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
        self.order              = None  # "无人机编号（int）",
        self.areas_id           = None  # "int",
        self.role               = None  # "unassigned   repeater    wide_searcher    tasker     ",
        self.can_fly_status     = None  # "飞行状态（int）： 0已损坏|不可飞行|没电|打击报废     1飞行中                 12.29之前师兄使用",
        self.lon                = None  # "无人机经度（double）",
        self.lat                = None  # "无人机纬度（double）",
        self.alt                = None  # "无人机高度（double）"
        #################### from run config sub_run_init_uav_pos_phase #################### 
        self.home_lon           = None  # "地面站传入无人机期起点经度（double）",
        self.home_lat           = None  # "地面站传入无人机期起点纬度（double）"
        self.home_alt           = None  # "地面站传入无人机期起点纬度（double）"
        self.home_x             = None  # "地面站传入无人机期起点x（double）",
        self.home_y             = None  # "地面站传入无人机期起点y（double）",
        self.home_z             = None  # "地面站传入无人机期起点y（double）",
        self.cur_x              = None  # "无人机当前x（double）",
        self.cur_y              = None  # "无人机当前y（double）",
        self.cur_z              = None  # "无人机当前高度（double）"
        #################### from run config sub_run_init_uav_pos_phase #################### 
        self.plan_phase_list    = None      
        self.xyz_path_phase     = None
        self.plan_phase_index   = None
        #################### from run config result out #################### 
        self.way_point_xyz      = None
        self.way_point_gps      = None  # (40N, 116E)
        #################### from run sub_configuration_hierarchical_clustering #################### 
        self.assigned_areas_id  = None
        #################### from run config sub_configuration_cpp_keypoint #################### 
        self.relay_station_xylist        = None  # [x,y]
        self.surveil_station_xylist      = None  # [x,y]
        self.tasker_key_point_xylist_list= None  # [[x,y], [x,y]]
        self.tasker_key_point_finished_id= None 
        #################### from parse sub_parse_uav #################### 
        self.finished_point_id  = None  # 没有航点默认 -1  扫描平原|侦察任务 切换到 目标识别跟踪 或者 自杀式任务， 需要告诉集群任务规划已经完成了的平原扫描的航点id（而不是数量） 。
        self.can_fly_status     = protocol.uav.can_fly_status.good  # can_fly_status
        self.break_lon          = None  # "离开扫描侦查的经度（double）",
        self.break_lat          = None  # "离开扫描侦查的纬度（double）"
        self.break_alt          = None  # "离开扫描侦查的纬度（double）"
        self.break_x            = None  # "离开扫描侦查的经度（double）",
        self.break_y            = None  # "离开扫描侦查的纬度（double）"
        self.break_z            = None  # "离开扫描侦查的纬度（double）"
        self.changed_5msg       = None  # [x,y] tasker
        #################### from Split_In_And_Out sub_execution_target_consumption #################### 
        self.changed_5msg       = True  #  surveil
        self.target_id          = None  #        surveil
        self.scan_all_finished  = None  #  # （int）  0 没有完成，都不返航  1 完成，返航  
        #################### from Split_In_And_Out sub_execution_cppkey_supplement #################### 


        pass

    def mount_run_config_sub_run_init_uav_pos_phase(self, plan) -> None:
        #################### from run config sub_run_init_uav_pos_phase #################### 
        x, y = coord.ned.instance.gps_to_xy(lat=self.lat, lon=self.lon)
        self.home_lon   = self.lon
        self.home_lat   = self.lat
        self.home_alt   = self.alt
        self.home_x     = x
        self.home_y     = y
        self.home_z     = self.alt
        self.cur_x      = x
        self.cur_y      = y
        self.cur_z      = self.alt
        #################### from run config sub_run_init_uav_pos_phase #################### 
        self.plan_phase_index   = 0
        self.plan_phase_list    = [ one_plan[0]     for one_plan in plan]  # 一维列表    
        self.xyz_path_phase     = [ []              for one_plan in plan]  # 二维列表 # 起飞降落，中间阶段要多两个点
        """
        1  2.1  3.1          3.2
        |￣￣↓↑￣￣￣￣￣￣￣￣￣|              relay
        |    ↓↑                 |             
        |    2.2                |             
        |                       |             
        |1   2.1  3.1           |3.1 
        |￣￣￣￣↓↑￣￣￣￣￣￣￣ |             surveil
        |       ↓↑              |    
        |       2.2             |    
        |                       |             
        | 1  2.1                |        
        | |￣|  2.2  3.1 3.2    |     
        | |  |_______|￣￣|     |             tasker
        | |               |3.3  |3.2   
        ======================================================
        1.1  1.2  3.1          3.2
        |￣￣↓↑￣￣￣￣￣￣￣￣￣|              relay
        |    ↓↑                 |             
        |    2                  |             
        |                       |             
        |1.1  1.2  3.1          |3.1 
        |￣￣￣￣↓↑￣￣￣￣￣￣￣ |             surveil
        |       ↓↑              |    
        |       2               |    
        |                       |             
        | 1.1 1.2               |        
        | |￣|   2   3.1 3.2    |     
        | |  |_______|￣￣|     |             tasker
        | |               |3.3  |3.2   
        """
        #################### from aggregate_take_off_and_set_off #################### 
        self.way_point_xyz      = []
        self.way_point_gps      = []
        self.way_point__id      = []
        #################### from run config sub_run_init_uav_pos_phase #################### 
        self.excute_phase_index = 2
        pass




order2uav           : Dict[int, UAV] = {}
order2unassigned    : Dict[int, UAV] = {}
order2relay         : Dict[int, UAV] = {}
order2surveil       : Dict[int, UAV] = {}
order2tasker        : Dict[int, UAV] = {}
order2waiter        : Dict[int, UAV] = {}
order2bad           : Dict[int, UAV] = {}
