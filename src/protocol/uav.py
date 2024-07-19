
from enum import Enum

class role(Enum):
    # unassigned  = "unassigned"
    # relay       = "relay"
    # surveil     = "surveil"
    # tasker      = "tasker"
    unassigned  = 0
    relay       = 1
    surveil     = 2
    tasker      = 3
    
class can_fly_status(Enum):
    bad         = 0
    good        = 1


