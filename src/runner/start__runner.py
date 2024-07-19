

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

class Start__Runner(Base_Runner):
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag = True
        pass

    def start(self) -> None:
        pass

    def run_tasks(self, plan) -> None:
        self.sub_run_start(plan)
        pass

    def sub_run_start(self, plan) -> None:
        ele =  random_access_queue.instance.popleft()  # 弹出第一个叫做：规划-划区域-分层分组
        tracker.debug_tracker.instance.pf(ele)
        ele =  random_access_queue.instance.popleft()  # 弹出第一个叫做：规划-划区域-分层分组
        tracker.debug_tracker.instance.pf(ele)
        tracker.debug_tracker.instance.current_position_print()

        # for si in random_access_queue.instance.queue:
        #     # si: Server_Abstract
        #     # si.execution_process_run()     
        #     if si.__class__.__name__ == "Draw_Area":                
        #         draw_area: Draw_Area = si
        #         draw_area.execution_process_run()
        #     elif si.__class__.__name__ == "Release_Climb":
        #         release_climb: Release_Climb = si
        #         release_climb.execution_process_run()
        #     elif si.__class__.__name__ == "Split_In_And_Out":
        #         split_in_and_out: Split_In_And_Out = si
        #         split_in_and_out.execution_process_run()
        #     elif si.__class__.__name__ == "Return_And_Recovery":
        #         return_and_recovery: Return_And_Recovery = si
        #         return_and_recovery.execution_process_run()
        #     elif si.__class__.__name__ == "Accompanying_Cover":
        #         accompanying_cover: Accompanying_Cover = si
        #         accompanying_cover.execution_process_run()
        #     else:
        #         tracker.debug_tracker.instance.current_position_print()
        #         tracker.violation_detector.instance.raise_key_error(
        #             "Servers are not listed : " + str(si.__class__.__name__)
        #         )
        pass