



from src                import entity
from src                         import tracker
from src                import coord
from src.server_queue   import random_access_queue  


from .uav_parser        import UAV_Parser
from .phase_parser      import Phase_Parser     
from .task_parser       import Task_Parser


class Parser( ):
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag     = True
        self.uav_parser     = UAV_Parser()
        self.phase_parser   = Phase_Parser()
        self.task_parser    = Task_Parser()
        pass

    def start(self) -> None:
        pass

    def parse(self, json_dict):
        entity.decision.decision = json_dict["decision"]
        if entity.decision.decision == "configure":
            entity.task.        tasker_key_point_xylist_list_poollist.      clear()
            entity.task.        target_id_2_target.                         clear()
            entity.uav.         order2uav.                                  clear()
            entity.uav.         order2unassigned.                           clear()
            entity.uav.         order2relay.                                clear()
            entity.uav.         order2surveil.                              clear()
            entity.uav.         order2tasker.                               clear()
            entity.uav.         order2waiter.                               clear()
            entity.uav.         order2bad.                                  clear()
            entity.airspace.    mission_phase_type_2_airspace__pair_list.   clear()
            entity.scout_area.  areas_id_2_scout_area_dict.                 clear()
            coord.ned.          instance.                                   clear()
            random_access_queue.instance.                                   clear()
            self.   uav_parser.   sub_parse_uav                 (UAVs_json_dict=json_dict["UAVs"])
            self.   uav_parser.   sub_parse_coordinate_system   (UAVs_json_dict=json_dict["UAVs"])
            self.   phase_parser. sub_parse_mission0            (mission0_json_dict=json_dict["mission"][0])
            plan = entity.airspace.mission_phase_type_2_airspace__pair_list
            if self.debug_flag == True:
                tracker.debug_tracker.instance.pf('plan = ')
                tracker.debug_tracker.instance.pf(plan)
                tracker.debug_tracker.instance.current_position_print()
            return plan
            pass
        elif entity.decision.decision == "start":
            return None
            pass
        elif entity.decision.decision == "execute":
            self.   task_parser.  sub_parse_mission0            (mission0_json_dict=json_dict["mission"][0])
            self.   task_parser.  sub_parse_uav                 (UAVs_json_dict=json_dict["UAVs"])
            
            if self.debug_flag == True:
                tracker.debug_tracker.instance.current_position_print()
                # tracker.debug_tracker.instance.enter_pause()
            
            return None
        elif entity.decision.decision == "back":
            pass
        else:
            pass
        return 
