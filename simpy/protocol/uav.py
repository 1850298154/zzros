
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

class task_status(Enum):
    # unassigned  = "unassigned"
    # relay       = "relay"
    # surveil     = "surveil"
    # scan        = "scan"
    # watch       = "watch"
    # attack      = "attack"
    # wait        = "wait"
    unassigned  = 0
    relay       = 1
    surveil     = 2
    scan        = 3
    watch       = 4
    attack      = 5
    wait        = 6
    dead        = 7

# # 根据字符串值获取枚举成员
# def task_status_get_enum_from_value(value):
#     for member in task_status:
#         if member.value == value:
#             return member
#     raise ValueError('No enum member with value {}'.format(value))