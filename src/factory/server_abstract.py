
from src                         import tracker
class Server_Abstract:
    def __init__(self) -> None:
        pass

    def initialize(self) -> None:
        self.mount_debug_flag = True
        self.config_debug_flag = True
        pass

    def start(self) -> None:
        pass

    def mount_input(self, input_list):
        if self.mount_debug_flag == True:
            tracker.debug_tracker.instance.pf('input_list')
            tracker.debug_tracker.instance.pf(input_list) 
            tracker.debug_tracker.instance.current_position_print()

        pass

    def configuration_process_run(self, config_data=None):
        pass

    def execution_process_run(self):
        
        pass
