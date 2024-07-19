

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


from .config_runner import Config_Runner
from .execut_runner import Execut_Runner
from .start__runner import Start__Runner


class Runner:
    def __init__(self) -> None:
        self.initialize()
        pass

    def initialize(self) -> None:
        self.debug_flag     = True
        self.config_runner  = Config_Runner()
        self.start__runner  = Start__Runner()
        self.execut_runner  = Execut_Runner()
        pass

    def start(self) -> None:
        pass

    def run_tasks(self, plan) -> None:
        if entity.decision.decision == "configure":
            self.config_runner.run_tasks(plan)
            pass
        elif entity.decision.decision == "start":
            self.start__runner.run_tasks(plan)
            pass
        elif entity.decision.decision == "execute":
            self.execut_runner.run_tasks(plan)
            pass
        elif entity.decision.decision == "back":
            pass
        else:
            pass
        pass