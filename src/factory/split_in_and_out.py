
import math

from src                            import tracker
from src                            import entity
from src                            import protocol
from src.factory.server_abstract    import Server_Abstract
from typing                         import Dict, List, Tuple

class Split_In_And_Out(Server_Abstract):
    def __init__(self) -> None:
        self.clear()
        self.initialize()
        pass

    def initialize(self) -> None:
        self.mount_debug_flag   = True
        self.config_debug_flag  = True
        self.excute_debug_flag  = True
        self.phase_enum         = protocol.mission.phase.split_in_and_out
        pass

    def start(self) -> None:
        self.airspace: entity.airspace.Airspace = None
        pass

    def clear(self) -> None:
        pass

    def mount_input(self, input_list):
        if self.mount_debug_flag == True:
            tracker.debug_tracker.instance.pf('input_list')
            tracker.debug_tracker.instance.pf(input_list) 
            tracker.debug_tracker.instance.current_position_print()
        self.airspace: entity.airspace.Airspace = input_list[0]
        pass

    def execution_process_run(self, ):
        
        if self.config_debug_flag == True:
            for k, v  in entity.uav.order2uav.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
            # tracker.debug_tracker.instance.enter_pause()
        
        self.sub_execution_target_consume_tasker_8_recycle_waiter()
        self.sub_execution_target_consume_surveil()
        self.sub_execution_cppkey_supple_tasker_scan()
        self.sub_execution_key_point_station()
        
        
        pass

    def sub_execution_key_point_station(self, ):
        for k, v in entity.uav.order2uav.items():
            if v.changed_5msg == True:
                role = v.role
                xyz_path = []
                xylist_list = None
                if False:
                    pass
                elif protocol.uav.role(role) == protocol.uav.role.unassigned:
                    tracker.debug_tracker.instance.current_position_print()
                    raise ValueError("Invalid input: aggregate_changed  has unassigned.")
                    pass
                elif protocol.uav.role(role) == protocol.uav.role.relay:
                    xylist_list = [v.relay_station_xylist]
                    pass
                elif protocol.uav.role(role) == protocol.uav.role.surveil:
                    xylist_list = [v.surveil_station_xylist]
                    pass
                elif protocol.uav.role(role) == protocol.uav.role.tasker:
                    xylist_list = v.tasker_key_point_xylist_list
                    pass
                for xy in xylist_list:
                    xyz =   [
                            *xy, 
                            self.airspace.get_high_from_enum_value(v.role)
                            ]
                    xyz_path.append(xyz)
                v.xyz_path_phase[v.excute_phase_index] = xyz_path
            else:
                pass
        
        if self.excute_debug_flag == True:
            for k, v  in entity.uav.order2uav.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        pass

    def sub_execution_target_consume_tasker_8_recycle_waiter(self, ):
        new_bad__kv:Dict[int, entity.uav.UAV] = {}
        new_wait_kv:Dict[int, entity.uav.UAV] = {}
        # 任务机自杀报废 扫描任务回收
        for k, v in entity.uav.order2tasker.items():
            changed_condition   = ( v.changed_5msg == True )
            role_condition      = ( protocol.uav.role(v.role)                       == protocol.uav.role.tasker )
            bad_condition       = ( protocol.uav.can_fly_status(v.can_fly_status)   == protocol.uav.can_fly_status.bad )
            # 任务完成的无人机 回收到 等待任务池
            if changed_condition and role_condition and not bad_condition:
                key_point_xylist_list       = v.tasker_key_point_xylist_list
                unfinished_xylist_list      = key_point_xylist_list[v.finished_point_id:] # 扫描任务完成 回收任务机
                if unfinished_xylist_list == []:
                    v.scan_all_finished         = True
                    # entity.uav.order2waiter[k]  = v                                 # 任务机自杀 报废
                    new_wait_kv[k]  = v                                 # 任务机自杀 报废
                else:
                    tracker.debug_tracker.instance.current_position_print()
                    raise ValueError("Invalid input: Bad bad_condition     can_fly_status")
                    pass # 任务机未完成 不应该出现在这个逻辑 # 在发出的changed信号中 # 要么完成任务 # 要么自杀 

            # 打击消耗的无人机
            if changed_condition and role_condition and bad_condition:
                # entity.uav.order2bad[k]     = v                                 # 任务机自杀 报废
                new_bad__kv[k]               = v                                 # 任务机自杀 报废
                key_point_xylist_list       = v.tasker_key_point_xylist_list
                unfinished_xylist_list:List = key_point_xylist_list[v.finished_point_id:] # 扫描任务回收
                if unfinished_xylist_list == []:
                    continue
                # unfinished_xylist_list[0]   = [v.break_x, v.break_y]
                unfinished_xylist_list.insert(0, [v.break_x, v.break_y])
                entity.task.tasker_key_point_xylist_list_poollist.append(unfinished_xylist_list)


        # 打击目标 消耗 任务机自杀 报废
        for k, v in new_bad__kv.items():
            entity.uav.order2bad[k]         = v    
            entity.uav.order2tasker.pop(k)
            entity.uav.order2uav.pop(k)
            pass
        # 完成任务 放到 等待分配任务的无人机池
        for k, v in new_wait_kv.items():
            entity.uav.order2waiter[k]      = v
            entity.uav.order2tasker.pop(k)
            pass


        if self.config_debug_flag == True:
            tracker.debug_tracker.instance.pf('entity.task.tasker_key_point_xylist_list_poollist')
            tracker.debug_tracker.instance.pf(entity.task.tasker_key_point_xylist_list_poollist)
            tracker.debug_tracker.instance.current_position_print()
            for k, v  in entity.uav.order2uav.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
            for k, v  in entity.task.target_id_2_target.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        pass


    def sub_execution_target_consume_surveil(self, ):
        target_id_2_target = entity.task.target_id_2_target
        if target_id_2_target.__len__() <= 0:
            return

        # 打击目标 消耗 广宇搜索机 看
        for k, v in entity.uav.order2surveil.items():
            surveil = v
        
        
        
        # 贪心
        for k, v in target_id_2_target.items():
            v.dist_cost = ((surveil.cur_x-v.x)**2 + (surveil.cur_y-v.y)**2)**(1/2) 
            pass
        min_key     = min(target_id_2_target, key=lambda k: target_id_2_target[k].dist_cost)
        # min_target  = target_id_2_target[min_key]
        min_target  = target_id_2_target.pop(min_key)
        surveil.surveil_station_xylist  = [min_target.x, min_target.y, ]
        surveil.target_id       = min_key
        surveil.changed_5msg    = True


    def sub_execution_cppkey_supple_tasker_scan(self, ):
        
        wait_uav_list = [
            v
            for k, v in entity.uav.order2waiter.items()
        ]
        
        # start_key_point_xy = [
        #     unfinished_xylist_list[0][:2]
        #     for unfinished_xylist_list in 
        #     entity.task.tasker_key_point_xylist_list_poollist
        # ]
        
        # 任务分发
        from scipy.optimize import linear_sum_assignment
        import numpy as np
        
        unassigned_uav_list     = wait_uav_list
        unassigned_task_list    = entity.task.tasker_key_point_xylist_list_poollist
        # 只有 打击阵亡 且 未完成扫描任务 的是 unassigned_task_list
        uav_N, start_M = len(unassigned_uav_list) , len(unassigned_task_list)  # 0, 1 # (1, 1) # (1, 0)
        cost = np.zeros((uav_N, start_M))
        for i, uav in enumerate(unassigned_uav_list):
            for j, task in enumerate(unassigned_task_list):
                travel_dis = np.linalg.norm(
                                np.array([uav.cur_x, uav.cur_y]) - 
                                np.array(task[0][:2])
                            )
                # cost[i,j] = travel_dis + task.total_distance
                cost[i,j] = travel_dis
        
        # 如下是两种 重分配的方式 ： 1.打击过后  2.分割任务
        # 1.打击过后
        # 可以分配的无人机==unassigned_uav_list  <  打击阵亡==unassigned_task_list
        if uav_N <= start_M:  # # 0, 1  ## (1, 0) 2024-3-27 # (1, 0)
            uav_idx, task_idx = linear_sum_assignment(cost)
        else:
            task_idx, uav_idx = linear_sum_assignment(cost.T) # (array([], dtype=int64), array([], dtype=int64))

        if self.config_debug_flag == True:
            tracker.debug_tracker.instance.pf('entity.task.tasker_key_point_xylist_list_poollist')
            tracker.debug_tracker.instance.pf(entity.task.tasker_key_point_xylist_list_poollist)
            tracker.debug_tracker.instance.current_position_print()
        task_j_list = []
        for i,j in zip(uav_idx, task_idx): #直接把任务给帮忙的没任务无人机
            uav:entity.uav.UAV  = unassigned_uav_list[i] 
            task                = unassigned_task_list[j]
            uav.finished_point_id               = 0 # 20
            uav.tasker_key_point_finished_id    = 0 # 0
            uav.scan_all_finished               = 0 # 1
            uav.tasker_key_point_xylist_list    = task  
            entity.uav.order2tasker[uav.order]  = uav
            entity.uav.order2waiter.pop(uav.order)
            task_j_list.append(j)
            # uav.task = task  # {'task_id': ('plain', 1), 'init_assign': 1, 'key_points': array([[-56.39009774,   0.68102275,   6.        ],       [-20.19836008,  68.06279052,   6.        ],       [-19.39042567,  65.3402789 ,   6.        ],       [-54.13449383,   0.65378184,   6.        ],       [-51.87888992,   0.62654093,   6.        ],       [-18.58249127,  62.61776728,   6.        ],       [-17.77455687,  59.89525566,   6.        ],       [-49.62328601,   0.59930002,   6.        ],       [-47.3676821 ,   0.57205911,   6.        ],       [-16.96662246,  57.17274404,   6.        ],       [-16.15868806,  54.45023242,   6.        ],       [-45.11207819,   0.5448182 ,   6.        ],       [-42.85647428,   0.51757729,   6.        ],       [-15.35075366,  51.7277208 ,   6.        ],       [-14.54281926,  49.00520918,   6.        ],       [-40.60087037,   0.49033638,   6.        ],       [-38.34526646,   0.46309547,   6.        ],       [-13.73488485,  46.28269756,   6.        ],       [-12.92695045,  43.56018594,   6.        ],       [-36.08966255,   0.43585456,   6.        ],       [-33.83405864,   0.40861365,   6.        ],       [-12.11901605,  40.83767431,   6.        ],       [-11.31108164,  38.11516269,   6.        ],       [-31.57845473,   0.38137274,   6.        ],       [-29.32285082,   0.35413183,   6.        ],       [-10.50314724,  35.39265107,   6.        ],       [ -9.69521284,  32.67013945,   6.        ],       [-27.06724691,   0.32689092,   6.        ]]), 'cur_index': 0, 'total_index': 28, 'finished': False, 'assign_id': 10, 'finished_point_ID': 0, 'uav_data': {'fly_status': 0, 'order': 1, 'task_status': 5, 'scout_end': 0, 'finished_point_ID': 0, 'lon': 8.5450487, 'lat': 47.3973261, 'alt': 2.603, 'break_lon': 8.5450006, 'break_lat': 47.3973184, 'break_alt': 11.0}}
            # self.SharedInf.uav_dict[task.assign_id].task = None
            # task.assign_id = uav.id
            # task.update(uav.pos)  # zytTODO: 不需要更新位置
        new_list = []
        for i, v in enumerate(entity.task.tasker_key_point_xylist_list_poollist):
            if i in task_j_list:
                pass
            else:
                new_list.append(v)
        entity.task.tasker_key_point_xylist_list_poollist = new_list
        if self.config_debug_flag == True:
            tracker.debug_tracker.instance.pf('entity.task.tasker_key_point_xylist_list_poollist')
            tracker.debug_tracker.instance.pf(entity.task.tasker_key_point_xylist_list_poollist)
            tracker.debug_tracker.instance.current_position_print()
        
        # for b in task_j_list.sort()[::-1]:
        
        
        if self.config_debug_flag == True:
            for k, v  in entity.uav.order2uav.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        
        pass


    def configuration_process_run(self, config_data=None):
        if self.config_debug_flag == True:
            tracker.debug_tracker.instance.pf('config_data')
            tracker.debug_tracker.instance.pf(config_data) 
            tracker.debug_tracker.instance.current_position_print()

        for k, v in entity.uav.order2uav.items():
            str = v.plan_phase_list[v.plan_phase_index]
            if protocol.mission.phase(str) != self.phase_enum:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: protocol.mission.phase(str) != self.phase_enum")
            # 加入丝滑操作， 当 高度差 大于 某个 值  再进行插入 下降或者上升。
            xyz_path = []
            
            role        = v.role
            xylist_list = None
            if False:
                pass
            elif protocol.uav.role(role) == protocol.uav.role.unassigned:
                tracker.debug_tracker.instance.current_position_print()
                raise ValueError("Invalid input: area_point_content must be greater than 3 points.")
                pass
            elif protocol.uav.role(role) == protocol.uav.role.relay:
                xylist_list = [v.relay_station_xylist]
                pass
            elif protocol.uav.role(role) == protocol.uav.role.surveil:
                xylist_list = [v.surveil_station_xylist]
                pass
            elif protocol.uav.role(role) == protocol.uav.role.tasker:
                xylist_list = v.tasker_key_point_xylist_list
                pass
            
            for xy in xylist_list:
                xyz =   [
                        *xy, 
                        self.airspace.get_high_from_enum_value(v.role)
                        ]
                xyz_path.append(xyz)
            v.xyz_path_phase[v.plan_phase_index] = xyz_path
            self.own_xy_previous_z(
                v.xyz_path_phase[v.plan_phase_index-1],
                v.xyz_path_phase[v.plan_phase_index  ],
            )
            pass

        if self.config_debug_flag == True:
            for k, v  in entity.uav.order2uav.items():
                tracker.debug_tracker.instance.pf(k)
                tracker.debug_tracker.instance.pf(v.__dict__)
            tracker.debug_tracker.instance.current_position_print()
        pass

    def own_xy_previous_z(self, p_xyzlist_list:List, c_xyzlist_list:List):
        pz  = p_xyzlist_list[-1][2]
        xyz = [
            *c_xyzlist_list[0][:2],
            pz,
        ]
        # c_xyzlist_list.insert(0, xyz)
        p_xyzlist_list.append(xyz)
        pass





