from src                                import entity
from src                                import protocol 
from src                                import tracker 
from src.factory.server_factory         import ServerFactory
from src.server_queue                   import random_access_queue 

class Dispatcher:
    def __init__(self) -> None:
        pass

    def initialize(self) -> None:
        pass

    def start(self) -> None:
        pass

    def dispatch(self, plan) -> None:
        decision_str = entity.decision.decision
        if protocol.decision.decision(decision_str) == protocol.decision.decision.configure:
            for one_plan in plan:
                sf = ServerFactory()
                si = sf.create_phase_server(one_plan[0])
                si.mount_input(input_list= one_plan[1:])
                random_access_queue.instance.append(si)
            random_access_queue.instance.print()
        pass
