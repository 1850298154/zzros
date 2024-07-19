

from src.factory.server_abstract        import Server_Abstract
from src.factory.draw_area              import Draw_Area
from src.factory.release_climb          import Release_Climb
from src.factory.split_in_and_out       import Split_In_And_Out
from src.factory.return_and_recovery    import Return_And_Recovery
from src.factory.accompanying_cover     import Accompanying_Cover



from src.server_queue                   import random_access_queue 
from src                                import entity 
from src                                import tracker
from src                                import entity



from .base_runner import Base_Runner

class Config_Runner(Base_Runner):
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        pass

    def start(self) -> None:
        pass

    def run_tasks(self, plan) -> None:
        self.sub_run_init_uav(plan)
        self.sub_run_init_scout_area(plan)
        self.sub_run_server_configuration_process_run(plan)
        pass

    def sub_run_init_uav(self, plan) -> None:
        tracker.debug_tracker.instance.pf('sub_run_init_uav')
        tracker.debug_tracker.instance.pf('order2uav    v.__dict__')
        for k, v in entity.uav.order2uav.items():
            # v: entity.uav.UAV
            v.mount_run_config_sub_run_init_uav_pos_phase(plan)
            tracker.debug_tracker.instance.pf(v.__dict__)
        tracker.debug_tracker.instance.current_position_print()
        pass

    def sub_run_init_scout_area(self, plan) -> None:
        tracker.debug_tracker.instance.pf('sub_run_init_scout_area')
        tracker.debug_tracker.instance.pf('areas_id_2_scout_area_dict    v.__dict__')
        for k, v in entity.scout_area.areas_id_2_scout_area_dict.items():
            # v: entity.scout_area.Scout_Area
            v.mount_run_config_sub_run_init_area()
            tracker.debug_tracker.instance.pf(v.__dict__)
        tracker.debug_tracker.instance.current_position_print()
        # tracker.debug_tracker.instance.enter_pause()
        pass

    def sub_run_server_configuration_process_run(self, plan) -> None:
        tracker.debug_tracker.instance.current_position_print()
        for si in random_access_queue.instance.queue:
            # si: Server_Abstract
            # si.configuration_process_run()
            if False:
                pass
            elif si.__class__.__name__ == "Draw_Area":                
                draw_area: Draw_Area = si
                draw_area.configuration_process_run()
            elif si.__class__.__name__ == "Release_Climb":
                release_climb: Release_Climb = si
                release_climb.configuration_process_run()
            elif si.__class__.__name__ == "Split_In_And_Out":
                split_in_and_out: Split_In_And_Out = si
                split_in_and_out.configuration_process_run()
            elif si.__class__.__name__ == "Return_And_Recovery":
                return_and_recovery: Return_And_Recovery = si
                return_and_recovery.configuration_process_run()
            elif si.__class__.__name__ == "Accompanying_Cover":
                accompanying_cover: Accompanying_Cover = si
                accompanying_cover.configuration_process_run()
            else:
                tracker.debug_tracker.instance.current_position_print()
                tracker.violation_detector.instance.raise_key_error(
                    "Servers are not listed : " + str(si.__class__.__name__)
                )
            for k, v  in entity.uav.order2uav.items():
                v.plan_phase_index += 1
                # tracker.debug_tracker.instance.pf(k)
                # tracker.debug_tracker.instance.pf(v.plan_phase_index)
        pass


