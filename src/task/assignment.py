from copy import copy
import numpy as np
from typing import Literal, Union, Dict, List, Tuple


class Coalition:
    def __init__(self, uav_id_list, task_id) -> None:
        self.uav_id_list:List[int] = uav_id_list
        self.task_id = task_id
        self.cost = np.inf

    def calc_cost(self):
        # TODO update here
        if len(self.uav_id_list) == 0:
            self.cost = np.inf
        else:
            self.cost = (1000*np.random.rand()+1000)/len(self.uav_id_list)
    
    def __deepcopy__(self):
        return (Coalition(uav_id_list=copy(self.uav_id_list), task_id=self.task_id))

class Assignment:
    def __init__(self, coalition_dict):
        self.coalition_dict:Dict[int,Coalition]=coalition_dict # {task_id: Coalition}
        self.uav_decision_dict:Dict[int,int]={} # {uav_id: task_id}
        self.update_agent_decision_dict()

    def update_agent_decision_dict(self):
        self.uav_decision_dict:Dict[int,int]={}
        for coalition in self.coalition_dict.values():
            for uav_id in coalition.uav_id_list:
                self.uav_decision_dict.update({uav_id:coalition.task_id})

    def calc_max_cost(self):
        max_cost = 0
        # max_cost, max_cost_coalition_id = 0, list(self.coalition_dict.keys())[0]
        for task_id, coalition in self.coalition_dict.items():
            if coalition.cost == np.inf:
                coalition.calc_cost()
            if coalition.cost >= max_cost:
                max_cost = coalition.cost
                max_cost_coalition_id = task_id
        self.max_cost = max_cost
        return max_cost, max_cost_coalition_id


def mydict_deepcopy(coalition_dict:Dict[Tuple,Coalition]):
    coalition_dict_copy:Dict[Tuple,Coalition]={}
    for key,value in coalition_dict.items():
        coalition_dict_copy.update({key:value.__deepcopy__()})
    return coalition_dict_copy