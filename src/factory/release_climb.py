
from src                            import tracker
from src                            import entity
from src                            import protocol
from src.factory.server_abstract    import Server_Abstract
from typing                         import Dict, List, Tuple

class Release_Climb(Server_Abstract):
    def __init__(self) -> None:
        self.clear()
        self.initialize()
        pass

    def initialize(self) -> None:
        self.mount_debug_flag   = True
        self.config_debug_flag  = True
        self.phase_enum         = protocol.mission.phase.release_climb
        pass

    def start(self) -> None:
        pass

    def clear(self) -> None:
        self.airspace: entity.airspace.Airspace = None
        pass

    def mount_input(self, input_list):
        if self.mount_debug_flag == True:
            tracker.debug_tracker.instance.pf('input_list')
            tracker.debug_tracker.instance.pf(input_list) 
            tracker.debug_tracker.instance.current_position_print()
        self.airspace: entity.airspace.Airspace = input_list[0]
        pass

    def configuration_process_run(self, config_data=None):
        if self.config_debug_flag == True:
            tracker.debug_tracker.instance.pf('config_data')
            tracker.debug_tracker.instance.pf(config_data) 
            tracker.debug_tracker.instance.current_position_print()

        for k, v in entity.uav.order2uav.items():
            str = v.plan_phase_list[v.plan_phase_index]
            if protocol.mission.phase(str) != self.phase_enum:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: protocol.mission.phase(str) != self.phase_enum")
            xyz =   [
                    v.cur_x, 
                    v.cur_y, 
                    self.airspace.get_high_from_enum_value(v.role)
                    ]
            xyz_path = [xyz]
            v.xyz_path_phase[v.plan_phase_index] = xyz_path
            # self.own_z_next_xy(
            #     # v.xyz_path_phase[v.plan_phase_index-1],
            #     v.xyz_path_phase[v.plan_phase_index  ],
            #     v.tasker_key_point_xylist_list, # 除了任务机，其他广宇|通信都没有
            # )
            pass

        if self.config_debug_flag == True:
            for k, v  in entity.uav.order2uav.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        pass



    # def own_xy_previous_z(self, p_xyzlist_list:List, c_xyzlist_list:List):
    def own_z_next_xy(self, c_xyzlist_list:List, n_xyzlist_list:List):
        nxy  = n_xyzlist_list[-1][:2]
        xyz = [
            *nxy,
            c_xyzlist_list[0][2],
        ]
        c_xyzlist_list.append(xyz)
        pass

