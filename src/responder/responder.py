

from src                                import entity
from src                                import coord 
from src                                import protocol 
from src                                import tracker 


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

    def respond(self, ) -> None:
        ret = self.respond_ret()
        for k, v in entity.uav.order2uav.items():
            v.changed_5msg = False
        return ret
        pass
    def respond_ret(self, ) -> None:
        decision_str = entity.decision.decision
        if False:
            pass
        elif protocol.decision.decision(decision_str) == protocol.decision.decision.configure:
            UAVs_json_dict = self.respond_sim_pid_ned()
            # UAVs_json_dict = self.respond_zz_gcs_gps()
            return UAVs_json_dict
            pass
        elif protocol.decision.decision(decision_str) == protocol.decision.decision.start:
            UAVs_json_dict = self.respond_sim_pid_ned()
            # UAVs_json_dict = self.respond_zz_gcs_gps()
            return UAVs_json_dict
            pass
        elif protocol.decision.decision(decision_str) == protocol.decision.decision.execute:
            if self.debug_flag == True:
                tracker.debug_tracker.instance.current_position_print()
                # tracker.debug_tracker.instance.enter_pause()
            UAVs_json_dict = self.respond_sim_pid_ned()
            # UAVs_json_dict = self.respond_zz_gcs_gps()
            return UAVs_json_dict
        elif protocol.decision.decision(decision_str) == protocol.decision.decision.back:
            self.aggregate_back_off()
            pass        
        else:
            tracker.debug_tracker.instance.current_position_print()
            raise ValueError("Invalid input: GCS JSON decision.")
            pass
        pass
        


    def respond_sim_pid_ned(self, ) -> None:
        UAVs_json_list = []
        for k, v in entity.uav.order2uav.items():
            if v.changed_5msg != True:
                continue
            role = v.role
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
                k_len = len(v.tasker_key_point_xylist_list) 
                if v.scan_all_finished == 1 and (k_len <= v.finished_point_id):
                    continue
                v.scan_all_finished = 1
                tracker.debug_tracker.instance.pf('k')
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(1)
                tracker.debug_tracker.instance.pf('finished_point_id')
                tracker.debug_tracker.instance.pf(v.finished_point_id)
                tracker.debug_tracker.instance.pf('k_len')
                tracker.debug_tracker.instance.pf(k_len)
                pass
            else:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: GCS JSON role.")
                pass

            id_list = v.way_point__id
            pt_list = v.way_point_xyz
            num1 = len(id_list)
            num2 = len(pt_list)
            if num1 != num2:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: len(id_list) != len(pt_list)")
            
            points = [
                {
                    "wp_id": id_list[i],
                    "x"  : pt_list[i][0],
                    "y"  : pt_list[i][1],
                    "z"  : pt_list[i][2],
                    "yaw"  : 0,
                }
                for i in range(num1)
            ]
            
            uav = {
                    "order"     : v.order,
                    "areas_id"  : v.areas_id,
                    "role"      : v.role,
                    "points"    : points
                }
            UAVs_json_list.append(uav)
        UAVs_json_dict = {'UAVs':UAVs_json_list}
        tracker.debug_tracker.instance.pf(UAVs_json_list)
        tracker.debug_tracker.instance.current_position_print()
        return UAVs_json_dict

    def respond_zz_gcs_gps(self, ) -> None:
        """
{
    "UAVs": [
        { 
            "order": "无人机编号（int）",
            "areas_id": "int",
            "role": "unassigned   repeater    wide_searcher    tasker     ",
            "points": [
                {
                    "wp_id": "航迹点 ID（int）",
                    "lon": "航迹点经度（double）",
                    "lat": "航迹点纬度（double）",
                    "alt": "无人机高度（double）",
                    "yaw": "航向（double）"
                }
            ]
        }
    ]
}
        """
        UAVs_json_list = []
        for k, v in entity.uav.order2uav.items():
            if v.changed_5msg != True:
                continue
            id_list = v.way_point__id
            pt_list = v.way_point_gps
            num1 = len(id_list)
            num2 = len(pt_list)
            if num1 != num2:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: len(id_list) != len(pt_list)")
            
            points = [
                {
                    "wp_id": id_list[i],
                    "lat"  : pt_list[i][0],
                    "lon"  : pt_list[i][1],
                    "alt"  : pt_list[i][2],
                    "yaw"  : 0,
                }
                for i in range(num1)
            ]
            
            uav = {
                    "order"     : v.order,
                    "areas_id"  : v.areas_id,
                    "role"      : v.role,
                    "points"    : points
                }
            UAVs_json_list.append(uav)
        UAVs_json_dict = {'UAVs':UAVs_json_list}
        tracker.debug_tracker.instance.pf(UAVs_json_list)
        tracker.debug_tracker.instance.current_position_print()
        return UAVs_json_dict
        pass


