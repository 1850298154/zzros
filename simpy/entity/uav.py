
import numpy as np
import math
from math import floor, ceil, cos, sin, tan
from typing import Literal, Union, Dict, List, Tuple

from simpy                          import protocol

def norm(x):
    return np.linalg.norm(x)


class UAV:
    def __init__(self) -> None:
        self.clear()
        self.proporty()
    
    def clear(self):
        #################### from jq json #################### 
        #################### from parse_uav_init #################### 
        self.order                          = 0
        self.areas_id                       = 0
        self.role                           = 0
        self.points:List[Dict[str, float]]  = []
        #################### from parse_uav_init #################### 
        self.id                             = 0
        self.xyzlist_list                   = []
        self.wp_id_list                     = []
        self.xyzlist_index                  = 0
        self.active                         = True
        self.task_status                    = protocol.uav.task_status.unassigned
        #################### from current location #################### 
        self.pos                            = np.array([0, 0, 0])
        #################### from return #################### 
        self.finished_point_id              = 0
        self.scan_all_finished              = False
        #################### from task #################### 
        self.target_id                      = 0
        self.wait_message_has_send__make_sure_only_once          = 0
        pass
    
    def proporty(self):
        self.yaw    =   0
        self.pitch  =   0
        self.roll   =   0
        #   for 3d visualization

        #    (p2) ***  *** (p1)
        #          \\  //    
        #           \\//    
        #        --------->>
        #           //\\    
        #          //  \\    
        #    (p3) ***  *** (p4)
        
        self._size = 10 # 飞机大小暂时未用
        self._angle = [45, 135, -135, -45]
        self._points = [np.array([self._size*np.cos(angle*np.pi/180), self._size*np.sin(angle*np.pi/180), 0, 1]).T for angle in self._angle] # len = 4  # self._points = [[7.0710678118654755, 7.0710678118654755, 0.0, 1.0], [-7.071067811865475, 7.0710678118654755, 0.0, 1.0], [-7.071067811865475, -7.0710678118654755, 0.0, 1.0], [7.0710678118654755, -7.0710678118654755, 0.0, 1.0]]

        # for camera FOV
        # self.theta = 40
        # self._HFOV = 24.9
        # self._VFOV = 14.7
        self._pt_fov:List[np.ndarray] = []
        self._corner_fov:List[np.ndarray] = []
        self._ground_fov:List[np.ndarray] = []

        self.head_pt = np.array([20, 0, 0, 1])
        self.y_pt = np.array([0, 10, 0, 1])
        self.z_pt = np.array([0, 0, 10, 1])

        self.optical_axis =  np.array([ 22.71447636,   0.        , -27.0700588 ,   1.        ])

        self.scout_range = 5 # 半径


    def transformation_matrix(self):
        x,y,z = self.pos
        roll = self.roll*np.pi/180
        pitch = self.pitch*np.pi/180
        yaw = self.yaw*np.pi/180
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch)
              * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll), z]
            ])

    def calc_yaw(self, direction:np.ndarray):
        self.yaw = np.arctan2(direction[1], direction[0]) * 180 / np.pi
    
    def move(self, dt, velocity):
        # if self.scan_all_finished: # IndexError: index 3 is out of bounds for axis 0 with size 3
        if self.xyzlist_index == len(self.xyzlist_list): # IndexError: index 3 is out of bounds for axis 0 with size 3
            return
        wps = self.xyzlist_list
        goal_pos            = wps[self.xyzlist_index]
        direction_vec       = goal_pos - self.pos
        direction_vec_norm  = norm(direction_vec)
        direction_vec       = direction_vec / max(direction_vec_norm,0.01)
        # if self.task.task_id[0] != 'building': # building任务的yaw角调整在scan()函数内
        self.calc_yaw(direction_vec)
        
        
        if direction_vec_norm < dt*velocity:
            self.pos = goal_pos
            self.update_task_state() # 用更新之后的坐标来更新任务状态
        else:
            self.pos = self.pos+dt*direction_vec*velocity


    def scan(self, height_limit = 35):
        if self.active == False:
            return None
        # scan for targets
        for targetid, target in targetid_2_target.items():
            dis_condition = (norm(target.pos[:2] - self.pos[:2]) < self.scout_range*1.2)
            hig_condition = (abs(target.pos[-1] - self.pos[-1]) <= height_limit)
            # (task_id not in self.SharedInf.task_dict.keys())
            liv_condition = target.active
            if dis_condition and hig_condition and liv_condition: # FIXME need to update
                self.task_status = protocol.uav.task_status.watch
                self.target_id = target.id
                
                target.discovered = True
                return target


    def update_task_state(self):
        self.xyzlist_index
        self.finished_point_id = self.wp_id_list[self.xyzlist_index]
        self.xyzlist_index += 1
        if self.xyzlist_index == len(self.wp_id_list):
            self.scan_all_finished = True
            if self.role == 3:
                self.task_status = protocol.uav.task_status.wait
        pass

class Target():
    def __init__(self, id: int, pos: np.ndarray) -> None:
        self.id = id  # 0
        self.pos = pos  # array([327.13206433, 402.07519494,  30.        ])
        self.active = True
        self.discovered = False


order2uav           : Dict[int, UAV] = {}
order2unassigned    : Dict[int, UAV] = {}
order2relay         : Dict[int, UAV] = {}
order2surveil       : Dict[int, UAV] = {}
order2tasker        : Dict[int, UAV] = {}
order2waiter        : Dict[int, UAV] = {}
order2bad           : Dict[int, UAV] = {}


targetid_2_target:Dict[int, Target] = {
    # 0  :    Target(   0 ,  3 * np.array([  0,  0,  0])  ) ,
    # 1  :    Target(   1 ,  3 * np.array([  10,  10,  0])  ), 
    # 2  :    Target(   2 ,  3 * np.array([  20,  20,  0])  ), 
    # 3  :    Target(   3 ,  3 * np.array([  30,  30,  0])  ), 
    # 4  :    Target(   4 ,  3 * np.array([  40,  40,  0])  ), 
    # 5  :    Target(   5 ,  3 * np.array([  50,  50,  0])  ), 
    # 6  :    Target(   6 ,  3 * np.array([  60,  60,  0])  ), 
    # 7  :    Target(   7 ,  3 * np.array([  70,  70,  0])  ), 
    # 8  :    Target(   8 ,  3 * np.array([  80,  80,  0])  ), 
    # 9  :    Target(   9 ,  3 * np.array([  90,  90,  0])  ),
    
    1  :    Target(   1 ,  3 * np.array([  10,  10,  0])  ), 
    2  :    Target(   2 ,  3 * np.array([  20,  20,  0])  ), 
    4  :    Target(   4 ,  3 * np.array([  40,  40,  0])  ), 
    7  :    Target(   7 ,  3 * np.array([  70,  70,  0])  ), 
    9  :    Target(   9 ,  3 * np.array([  90,  90,  0])  ),
}