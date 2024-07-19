
from typing import List, Tuple, Dict, Union, Callable

from src                            import tracker
from src                            import protocol


class Airspace:
    def __init__(self) -> None:
        self.clear()
        pass
    
    def initialize(self) -> None:
        pass
    
    def start(self) -> None:
        pass
    
    def clear(self) -> None:
        self.com_relay_fly_high = None
        self.area_search_fly_high = None
        self.fly_high = None
        self.fly_high_difference = None
        pass
    
    def get_high_from_enum_value(self, role):
        z = 0
        if False:
            pass
        elif protocol.uav.role(role) == protocol.uav.role.unassigned:
            tracker.debug_tracker.instance.current_position_print()
            raise ValueError("Invalid input: area_point_content must be greater than 3 points.")
            pass
        elif protocol.uav.role(role) == protocol.uav.role.relay:
            z = self.com_relay_fly_high
            pass
        elif protocol.uav.role(role) == protocol.uav.role.surveil:
            z = self.area_search_fly_high
            pass
        elif protocol.uav.role(role) == protocol.uav.role.tasker:
            z = self.fly_high
            pass
        else:
            tracker.debug_tracker.instance.current_position_print()
            raise ValueError("Invalid input: GCS JSON role.")
            pass
        
        return z



mission_phase_type_2_airspace__pair_list = []