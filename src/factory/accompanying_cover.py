
from src                         import tracker
from src.factory.server_abstract import Server_Abstract
from src                         import entity


class Accompanying_Cover(Server_Abstract):
    def __init__(self) -> None:
        self.clear()
        self.initialize()
        pass

    def initialize(self) -> None:
        self.mount_debug_flag = True
        self.config_debug_flag = True
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

        pass



