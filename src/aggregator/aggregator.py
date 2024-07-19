from src                                import entity
from src                                import coord 
from src                                import protocol 
from src                                import tracker 


class Aggregator:
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

    def aggregate(self, ) -> None:
        decision_str = entity.decision.decision
        if False:
            pass
        elif protocol.decision.decision(decision_str) == protocol.decision.decision.configure:
            self.aggregate_take_off_and_set_off()
            pass
        elif protocol.decision.decision(decision_str) == protocol.decision.decision.start:
            self.aggregate_take_off_and_set_off()
            pass
        elif protocol.decision.decision(decision_str) == protocol.decision.decision.execute:
            if self.debug_flag == True:
                tracker.debug_tracker.instance.current_position_print()
                # tracker.debug_tracker.instance.enter_pause()
            self.aggregate_changed()
            pass
        elif protocol.decision.decision(decision_str) == protocol.decision.decision.back:
            self.aggregate_back_off()
            pass        
        else:
            tracker.debug_tracker.instance.current_position_print()
            raise ValueError("Invalid input: GCS JSON decision.")
            pass
        pass

    def aggregate_changed(self, ):
        for k, v in entity.uav.order2uav.items():
            if v.changed_5msg == True:
                take_off_and_set_off_xyz_path_phase = v.xyz_path_phase[:-1]
                
                v.way_point_xyz = []
                for xyz_path in take_off_and_set_off_xyz_path_phase:
                    v.way_point_xyz += xyz_path
                v.way_point_xyz[0][0] = v.cur_x
                v.way_point_xyz[0][1] = v.cur_y
                v.way_point_xyz[1][0] = v.way_point_xyz[2][0]
                v.way_point_xyz[1][1] = v.way_point_xyz[2][1]
                
                v.way_point_gps = []
                for xyz in v.way_point_xyz:
                    lat, lon = coord.ned.instance.xy_to_gps(
                        x=xyz[0],
                        y=xyz[1],
                    )
                    gps = [lat, lon, xyz[2]]
                    v.way_point_gps.append(gps)
                
                v.way_point__id = []
                # for i in range(len(v.xyz_path_phase))
                for i, phase_str in enumerate(v.plan_phase_list[:-1]):
                    if protocol.mission.phase(phase_str) == protocol.mission.phase.split_in_and_out:
                        totol_key_point_num = len(v.xyz_path_phase[i])
                        wp_id_list = list(range(totol_key_point_num))
                        pass
                    else:
                        totol_travel_point_num = len(v.xyz_path_phase[i])
                        wp_id_list = [0] * totol_travel_point_num
                    v.way_point__id += wp_id_list
                
                tracker.debug_tracker.instance.pf(v.__dict__)
                tracker.debug_tracker.instance.current_position_print()

            else:
                pass
        pass
    def aggregate_take_off_and_set_off(self, ):
        
        for k, v in entity.uav.order2uav.items():
            
            take_off_and_set_off_xyz_path_phase = v.xyz_path_phase[:-1]
            
            v.way_point_xyz = []
            for xyz_path in take_off_and_set_off_xyz_path_phase:
                v.way_point_xyz += xyz_path
            
            v.way_point_gps = []
            for xyz in v.way_point_xyz:
                lat, lon = coord.ned.instance.xy_to_gps(
                    x=xyz[0],
                    y=xyz[1],
                )
                gps = [lat, lon, xyz[2]]
                v.way_point_gps.append(gps)
            
            v.way_point__id = []
            # for i in range(len(v.xyz_path_phase))
            for i, phase_str in enumerate(v.plan_phase_list[:-1]):
                if protocol.mission.phase(phase_str) == protocol.mission.phase.split_in_and_out:
                    totol_key_point_num = len(v.xyz_path_phase[i])
                    wp_id_list = list(range(totol_key_point_num))
                    pass
                else:
                    totol_travel_point_num = len(v.xyz_path_phase[i])
                    wp_id_list = [0] * totol_travel_point_num
                v.way_point__id += wp_id_list
            
            tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        pass

    def aggregate_back_off(self, ):
        
        for k, v in entity.uav.order2uav.items():
            
            back_off_xyz_path_phase = v.xyz_path_phase[-1:]
            
            v.way_point_xyz = []
            for xyz_path in back_off_xyz_path_phase:
                v.way_point_xyz += xyz_path
            
            v.way_point_gps = []
            for xyz in v.way_point_xyz:
                lat, lon = coord.ned.instance.xy_to_gps(
                    x=xyz[0],
                    y=xyz[1],
                )
                gps = [lat, lon, xyz[2]]
                v.way_point_gps.append(gps)

            v.way_point__id = []
            # for i in range(len(v.xyz_path_phase))
            for i, phase_str in enumerate(v.plan_phase_list[-1:]):
                if protocol.mission.phase(phase_str) == protocol.mission.phase.release_climb:
                    totol_key_point_num = len(v.xyz_path_phase[i])
                    wp_id_list = list(range(totol_key_point_num))
                    pass
                else:
                    totol_travel_point_num = len(v.xyz_path_phase[i])
                    wp_id_list = [0] * totol_travel_point_num
                v.way_point__id += wp_id_list
        pass





