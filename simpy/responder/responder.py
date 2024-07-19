

from simpy                                import entity
from simpy                                import protocol 
from simpy                                import tracker 
from simpy                                import coord 


class Responder:
    def __init__(self) -> None:
        self.clear()
        pass
    
    def initialize(self) -> None:
        self.debug_flag = True
        pass
    
    def start(self) -> None:
        pass
    
    def clear(self) -> None:
        pass

    def mount(self, ) -> None:
        pass

    def respond_wait(self, uav:entity.uav.UAV) -> None:
        finished_point_id = (
            uav.wp_id_list[uav.xyzlist_index]
            if uav.xyzlist_index < len(uav.wp_id_list)
            else uav.wp_id_list[uav.xyzlist_index - 1] + 1
        )
        scan_wait_uav_json_dict = {
                    "order"             : uav.order,
                    "areas_id"          : -1,
                    "target_id"         : -1,
                    "role"              : uav.role,
                    # "finished_point_id" : uav.finished_point_id,
                    # "finished_point_id" : uav.wp_id_list[uav.xyzlist_index],
                    # "finished_point_id" : uav.wp_id_list[min(uav.xyzlist_index, len(uav.wp_id_list))], # IndexError: list index out of range
                    # "finished_point_id" : uav.wp_id_list[min(uav.xyzlist_index, -1+len(uav.wp_id_list))], # IndexError: list index out of range
                    # "finished_point_id" : uav.xyzlist_index, # split_in_and_out.py # unfinished_xylist_list      = key_point_xylist_list[v.finished_point_id:] # 扫描任务完成 回收任务机
                    "finished_point_id" : finished_point_id, # split_in_and_out.py # unfinished_xylist_list      = key_point_xylist_list[v.finished_point_id:] # 扫描任务完成 回收任务机
                    "can_fly_status"    : 1,
                    "lat"               : coord.gps.x2lat(uav.pos[0]),
                    "lon"               : coord.gps.y2lon(uav.pos[1]),
                    "alt"               : 0,
                    "break_lat"         : coord.gps.x2lat(uav.pos[0]),
                    "break_lon"         : coord.gps.y2lon(uav.pos[1]),
                    "break_alt"         : 0
                }
        ret = {
            "decision":"execute",
            "mission": [
                {
                    "static_target":{
                        "target_todo": [
                        ]
                    }
                }
            ],
            "UAVs": [
                scan_wait_uav_json_dict
            ]
        }
        return ret

    def respond_scan_target(self, uav:entity.uav.UAV, target:entity.uav.Target) -> None:
        target_json_dict = {
                        "id"        : target.id,
                        "expect_num": 1,
                        "lat"       : coord.gps.x2lat(target.pos[0]),
                        "lon"       : coord.gps.y2lon(target.pos[1]),
                        "alt"       : 0
                    }
        target_todo = [target_json_dict]
        finished_point_id = (
            uav.wp_id_list[uav.xyzlist_index]
            if uav.xyzlist_index < len(uav.wp_id_list)
            else uav.wp_id_list[uav.xyzlist_index - 1] + 1
        )
        scan_target_uav_json_dict = {
                    "order"             : uav.order,
                    "areas_id"          : -1,
                    "target_id"         : target.id,
                    "role"              : uav.role,
                    # "finished_point_id" : uav.finished_point_id,
                    # "finished_point_id" : uav.wp_id_list[uav.xyzlist_index],
                    "finished_point_id" : finished_point_id,
                    "can_fly_status"    : 0,
                    "lat"               : coord.gps.x2lat(uav.pos[0]),
                    "lon"               : coord.gps.y2lon(uav.pos[1]),
                    "alt"               : 0,
                    "break_lat"         : coord.gps.x2lat(uav.pos[0]),
                    "break_lon"         : coord.gps.y2lon(uav.pos[1]),
                    "break_alt"         : 0
                }
        uav:entity.uav.UAV = list(entity.uav.order2surveil.values())[0]
        finished_point_id = (
            uav.wp_id_list[uav.xyzlist_index]
            if uav.xyzlist_index < len(uav.wp_id_list)
            else uav.wp_id_list[uav.xyzlist_index - 1] + 1
        )
        surveil_uav_json_dict = {
                    "order"             : uav.order,
                    "areas_id"          : -1,
                    "target_id"         : target.id,
                    "role"              : uav.role,
                    # "finished_point_id" : uav.finished_point_id,
                    "finished_point_id" : finished_point_id,
                    "can_fly_status"    : 1,
                    "lat"               : coord.gps.x2lat(uav.pos[0]),
                    "lon"               : coord.gps.y2lon(uav.pos[1]),
                    "alt"               : 0,
                    "break_lat"         : coord.gps.x2lat(uav.pos[0]),
                    "break_lon"         : coord.gps.y2lon(uav.pos[1]),
                    "break_alt"         : 0
                }
        
        ret = {
            "decision":"execute",
            "mission": [
                {
                    "static_target":{
                        "target_todo": target_todo
                    }
                }
            ],
            "UAVs": [
                scan_target_uav_json_dict,
                surveil_uav_json_dict,
            ]
        }
        return ret


