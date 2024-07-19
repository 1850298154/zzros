import pprint


from src                        import entity
from src                        import coord
from src                        import tracker
from src                        import protocol
from .base_parser               import Base_Parser             


class Task_Parser(Base_Parser):
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        pass

    def start(self) -> None:
        pass

    def sub_parse_mission0(self, mission0_json_dict):
        static_target = mission0_json_dict["static_target"]
        target_todo_list = static_target["target_todo"]
        for target_todo in target_todo_list:
            new_target_obj: entity.task.Target = entity.task.Target()
            new_target_obj: entity.task.Target = self.mount_dict2obj(
                                                obj=new_target_obj, 
                                                kwargs=target_todo,
                                            )
            new_target_obj.mount_parser_sub_parse_mission0()
            if self.debug_flag == True:
                tracker.debug_tracker.instance.pf('new_target_obj.__dict__')
                tracker.debug_tracker.instance.pf(new_target_obj.__dict__)
                tracker.debug_tracker.instance.current_position_print()
            
            entity.task.target_id_2_target[new_target_obj.id] = new_target_obj
            pass
        if self.debug_flag == True:
            tracker.debug_tracker.instance.pf('entity.task.target_id_2_target')
            tracker.debug_tracker.instance.pf(entity.task.target_id_2_target)
            for k, v in entity.task.target_id_2_target.items():
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        pass 

    def sub_parse_uav(self, UAVs_json_dict):
        if self.debug_flag == True:
            for k, v in entity.uav.order2uav.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        for uav_json_dict in UAVs_json_dict:
            order               = uav_json_dict["order"]
            finished_point_id   = uav_json_dict["finished_point_id"]
            can_fly_status      = uav_json_dict["can_fly_status"]
            break_lat           = uav_json_dict["break_lat"]
            break_lon           = uav_json_dict["break_lon"]
            break_alt           = uav_json_dict["break_alt"]
            lat                 = uav_json_dict["lat"]
            lon                 = uav_json_dict["lon"]
            alt                 = uav_json_dict["alt"]
            break_x, break_y = coord.ned.instance.gps_to_xy(
                lat=break_lat, 
                lon=break_lon,
                )
            cur_x, cur_y = coord.ned.instance.gps_to_xy(
                lat=lat, 
                lon=lon,
                )
            # entity.uav.order2uav[order].order           = order
            entity.uav.order2uav[order].finished_point_id   = finished_point_id  
            entity.uav.order2uav[order].tasker_key_point_finished_id   = finished_point_id  # DONE:   TODO :: LOG :: There must error, but not found why:  tasker_key_point_xylist_list == None
            entity.uav.order2uav[order].can_fly_status      = can_fly_status  
            entity.uav.order2uav[order].break_lon           = break_lon
            entity.uav.order2uav[order].break_lat           = break_lat
            # entity.uav.order2uav[order].break_alt           = break_alt # 飞机传过来的是错误的 海拔高度 ，不能用
            entity.uav.order2uav[order].break_x             = break_x
            entity.uav.order2uav[order].break_y             = break_y
            # entity.uav.order2uav[order].break_z             = break_lat
            entity.uav.order2uav[order].lat                 = lat
            entity.uav.order2uav[order].lon                 = lon
            # entity.uav.order2uav[order].alt                 = alt
            entity.uav.order2uav[order].cur_x               = cur_x
            entity.uav.order2uav[order].cur_y               = cur_y
            
            
            role = entity.uav.order2uav[order].role
            if False:
                pass
            elif protocol.uav.role(role) == protocol.uav.role.unassigned:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: area_point_content must be greater than 3 points.")
                pass
            elif protocol.uav.role(role) == protocol.uav.role.relay:
                pass
            elif protocol.uav.role(role) == protocol.uav.role.surveil:
                pass
            elif protocol.uav.role(role) == protocol.uav.role.tasker:
                if entity.uav.order2uav[order].tasker_key_point_xylist_list == None:
                    tracker.debug_tracker.instance.pf('TODO :: LOG :: There must error, but not found why:  tasker_key_point_xylist_list == None')
                    tracker.debug_tracker.instance.pf('entity.uav.order2uav[order].__dict__')
                    tracker.debug_tracker.instance.pf(entity.uav.order2uav[order].__dict__)
                    tracker.debug_tracker.instance.current_position_print()
                else:
                    k_len = len(entity.uav.order2uav[order].tasker_key_point_xylist_list) 
                    # entity.uav.order2uav[order].cur_z               =  # 飞机传过来的是错误的 海拔高度 ，不能用
                    if k_len <= finished_point_id:
                        entity.uav.order2uav[order].scan_all_finished = 1
                        tracker.debug_tracker.instance.pf('scan_all_finished')
                        tracker.debug_tracker.instance.pf(1)
                        tracker.debug_tracker.instance.pf('finished_point_id')
                        tracker.debug_tracker.instance.pf(finished_point_id)
                        tracker.debug_tracker.instance.pf('k_len')
                        tracker.debug_tracker.instance.pf(k_len)
                        tracker.debug_tracker.instance.current_position_print()
                pass
            else:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: GCS JSON role.")
                pass
            
            # k_len = len(entity.uav.order2uav[order].tasker_key_point_xylist_list) 
            # # entity.uav.order2uav[order].cur_z               =  # 飞机传过来的是错误的 海拔高度 ，不能用
            # if k_len <= finished_point_id:
            #     entity.uav.order2uav[order].scan_all_finished = 1
            #     tracker.debug_tracker.instance.pf('scan_all_finished')
            #     tracker.debug_tracker.instance.pf(1)
            #     tracker.debug_tracker.instance.pf('finished_point_id')
            #     tracker.debug_tracker.instance.pf(finished_point_id)
            #     tracker.debug_tracker.instance.pf('k_len')
            #     tracker.debug_tracker.instance.pf(k_len)
            #     tracker.debug_tracker.instance.current_position_print()

            
            entity.uav.order2uav[order].changed_5msg             = True
            
            if self.debug_flag == True:
                tracker.debug_tracker.instance.pf('entity.uav.order2uav[order].__dict__')
                tracker.debug_tracker.instance.pf(entity.uav.order2uav[order].__dict__)
                tracker.debug_tracker.instance.current_position_print()
            
        pass 

