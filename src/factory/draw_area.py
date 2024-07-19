
from src                            import tracker
from src                            import entity
from src                            import protocol
from src.factory.server_abstract    import Server_Abstract

from .scout_area_planner            import load_balancer

class Draw_Area(Server_Abstract):
    def __init__(self) -> None:
        self.clear()
        self.initialize()
        pass

    def initialize(self) -> None:
        self.mount_debug_flag   = True
        self.config_debug_flag  = True
        self.phase_enum         = protocol.mission.phase.draw_area
        pass

    def start(self) -> None:
        pass

    def clear(self) -> None:
        pass

    def mount_input(self, input_list):
        if self.mount_debug_flag == True:
            tracker.debug_tracker.instance.pf('input_list')
            tracker.debug_tracker.instance.pf(input_list) 
            tracker.debug_tracker.instance.current_position_print()

        pass

    def configuration_process_run(self, config_data=None):
        if self.config_debug_flag == True:
            tracker.debug_tracker.instance.pf('config_data')
            tracker.debug_tracker.instance.pf(config_data) 
            tracker.debug_tracker.instance.current_position_print()
        
        self.sub_configuration_unassigned2tasker()
        self.sub_configuration_hierarchical_clustering()
        self.sub_configuration_cpp_keypoint()
        self.sub_configuration_surveil_relay_station()
        if self.config_debug_flag == True:
            for k, v  in entity.uav.order2uav.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
        pass

    def sub_configuration_surveil_relay_station(self, ):
        for k, v  in entity.uav.order2surveil.items():
            scout_area =  entity.scout_area.areas_id_2_scout_area_dict[v.assigned_areas_id]
            v.surveil_station_xylist = [ scout_area.center_x, scout_area.center_y, ]
        for k, v  in entity.uav.order2relay.items():
            scout_area =  entity.scout_area.areas_id_2_scout_area_dict[v.assigned_areas_id]
            x = (v.home_x + scout_area.center_x) * 4 / 5
            y = (v.home_y + scout_area.center_y) * 4 / 5
            v.relay_station_xylist = [ x, y, ]
        tracker.debug_tracker.instance.current_position_print()
        pass


    def sub_configuration_cpp_keypoint(self, ):
        from .scout_area_planner import cpp_solve 
        for areas_id, scout_area in entity.scout_area.areas_id_2_scout_area_dict.items():

            uav_id_2_xylist             = {}
            region_polygon_xylist_list  = []
            half_scout_range            = entity.scout_area.scan_width

            for uav_id in scout_area.tasker_drones_assigned_key_list:
                tasker                  = entity.uav.order2tasker[uav_id]
                uav_xylist              = [tasker.cur_x, tasker.cur_y]
                uav_id_2_xylist[uav_id] = uav_xylist
            
            region_polygon_xylist_list = scout_area.area_point_content_xy_list
            
            uav_id_2_xylist_list  = cpp_solve(
                                    uav_id_2_xylist=uav_id_2_xylist,
                                    region_polygon_xylist_list=region_polygon_xylist_list,
                                    half_scout_range=half_scout_range,
                                    )
            
            for uav_id, xylist_list in uav_id_2_xylist_list.items():
                tasker                      = entity.uav.order2tasker[uav_id]
                tasker.tasker_key_point_xylist_list= xylist_list
                tasker.tasker_key_point_finished_id= 0
            pass # scout_area
        tracker.debug_tracker.instance.current_position_print()
        return

    def sub_configuration_hierarchical_clustering(self, ):
        
        tasker_k        =   [
                            k
                            for k, v 
                            in entity.uav.order2tasker.items()
                            ]
        n_drone         = len(entity.uav.order2tasker)
        m_region        = len(entity.scout_area.areas_id_2_scout_area_dict)
        # region_sizes    =   [ 
        #                     v.areasize
        #                     for k, v 
        #                     in entity.scout_area.areas_id_2_scout_area_dict.items()
        #                     ]
        region_k_list   =   [ 
                            k
                            for k, v 
                            in entity.scout_area.areas_id_2_scout_area_dict.items()
                            ]
        region_k_list.sort()
        region_sizes    =   [ 
                            entity.scout_area.areas_id_2_scout_area_dict[k].areasize
                            for k 
                            in region_k_list
                            ]
        
        
        drones_assigned  = load_balancer.Load_Balancer().assign_drones(
            n_drone      = n_drone,
            m_region     = m_region,
            region_sizes = region_sizes,
        )
        
        
        start = 0
        for i in range(m_region):
            region_k = region_k_list[i]
            
            entity.scout_area.areas_id_2_scout_area_dict[region_k].tasker_drones_assigned_key_num  = drones_assigned[i]
            entity.scout_area.areas_id_2_scout_area_dict[region_k].tasker_drones_assigned_key_list = tasker_k[start : start+drones_assigned[i]]
            # for uav_id in drones_assigned[i]:
            for uav_id in tasker_k[start : start+drones_assigned[i]]:
                entity.uav.order2tasker[uav_id].assigned_areas_id = region_k
            start = drones_assigned[i]
            
            # TODO:
            if len(entity.uav.order2surveil) != 1:
                tracker.debug_tracker.instance.pf('len(entity.uav.order2surveil) != 1')
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: len(entity.uav.order2surveil).")
            if len(entity.uav.order2relay) != 1:
                tracker.debug_tracker.instance.pf('len(entity.uav.order2relay) != 1')
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: len(entity.uav.order2relay).")
            shared_surveil_id_list = list(entity.uav.order2surveil.keys())
            shared_surveil_id_num  = len(shared_surveil_id_list)
            shared_relay_id_list   = list(entity.uav.order2relay.keys())
            shared_relay_id_num    = len(shared_relay_id_list)
            entity.scout_area.areas_id_2_scout_area_dict[region_k].surveil_drones_assigned_key_list    = shared_surveil_id_list
            entity.scout_area.areas_id_2_scout_area_dict[region_k].surveil_drones_assigned_key_num     = shared_surveil_id_num
            entity.scout_area.areas_id_2_scout_area_dict[region_k].relay_drones_assigned_key_list      = shared_relay_id_list
            entity.scout_area.areas_id_2_scout_area_dict[region_k].relay_drones_assigned_key_num       = shared_relay_id_num
            for _, v in entity.uav.order2surveil.items():
                v.assigned_areas_id = region_k
            for _, v in entity.uav.order2relay.items():
                v.assigned_areas_id = region_k
            # TODO:
        
        if self.config_debug_flag == True:
            for area_k, v  in entity.scout_area.areas_id_2_scout_area_dict.items():
                tracker.debug_tracker.instance.pf(area_k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        return



    def sub_configuration_unassigned2tasker(self, ):
        for k, v in entity.uav.order2unassigned.items():
            v.role = protocol.uav.role.tasker.value
            entity.uav.order2tasker[k] = v
            # v = entity.uav.order2unassigned.pop(k) # RuntimeError: dictionary changed size during iteration
        entity.uav.order2unassigned.clear()




