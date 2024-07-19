
import os
import sys
sys.path.append(
    os.path.dirname(os.path.abspath(__file__))
    + '/..'
    )

if True:
    pass
else:
    # 打印出Python解释器的所有搜索路径
    print('path')
    for path in sys.path:
        print(path)
    pass



import json
import numpy as np

from simpy.gateway.gateway          import Gateway
from simpy                          import entity
from simpy                          import tracker
from simpy.parser.parser            import protocol
from simpy.parser.parser            import Parser
from simpy.responder.responder      import Responder
from typing import Literal, Union, Dict, List, Tuple



class Simpy:
    def __init__(self) -> None:
        self.mlist = [
                # 1*1
                # '{"decision":"configure","mission":[{"night_scene":0,"inclusive_phase":["draw_area","release_climb","split_in_and_out","return_and_recovery"],"draw_area":{"scan_width":5,"scout_area":[{"areas_id":1,"areas_type":0,"area_target_num":2,"area_fire_num":2,"area_point_num":3,"area_point_content":[{"lon":0.0,"lat":0.0,"alt":0.0},{"lon":0.0,"lat":8.993216059187304e-04,"alt":0.0},{"lon":8.993216059187304e-04,"lat":8.993216059187304e-04,"alt":0.0},{"lon":8.993216059187304e-04,"lat":0.0,"alt":0.0}]}],"areas_no_fly":[],"areas_electronic_fence":[]},"release_climb":{"com_relay_fly_high":290.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3},"split_in_and_out":{"com_relay_fly_high":210.0,"area_search_fly_high":110.0,"fly_high":10.0,"fly_high_difference":3},"return_and_recovery":{"com_relay_fly_high":290.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3,"recycling_area":{"type":0,"points":[]}}}],"UAVs":[{"order":0,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":1,"areas_id":0,"role":1,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":2,"areas_id":0,"role":2,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":3,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":4,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0}]}' ,
                # 3*3
                # '{"decision":"configure","mission":[{"night_scene":0,"inclusive_phase":["draw_area","release_climb","split_in_and_out","return_and_recovery"],"draw_area":{"scan_width":5,"scout_area":[{"areas_id":1,"areas_type":0,"area_target_num":2,"area_fire_num":2,"area_point_num":3,"area_point_content":[{"lon":0.0,"lat":0.0,"alt":0.0},{"lon":0.0,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":0.0,"alt":0.0}]}],"areas_no_fly":[],"areas_electronic_fence":[]},"release_climb":{"com_relay_fly_high":290.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3},"split_in_and_out":{"com_relay_fly_high":210.0,"area_search_fly_high":110.0,"fly_high":10.0,"fly_high_difference":3},"return_and_recovery":{"com_relay_fly_high":290.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3,"recycling_area":{"type":0,"points":[]}}}],"UAVs":[{"order":0,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":1,"areas_id":0,"role":1,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":2,"areas_id":0,"role":2,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":3,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":4,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0}]}',
                # 3*3 hight
                # '{"decision":"configure","mission":[{"night_scene":0,"inclusive_phase":["draw_area","release_climb","split_in_and_out","return_and_recovery"],"draw_area":{"scan_width":5,"scout_area":[{"areas_id":1,"areas_type":0,"area_target_num":2,"area_fire_num":2,"area_point_num":3,"area_point_content":[{"lon":0.0,"lat":0.0,"alt":0.0},{"lon":0.0,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":0.0,"alt":0.0}]}],"areas_no_fly":[],"areas_electronic_fence":[]},"release_climb":{"com_relay_fly_high":310.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3},"split_in_and_out":{"com_relay_fly_high":390.0,"area_search_fly_high":190.0,"fly_high":10.0,"fly_high_difference":3},"return_and_recovery":{"com_relay_fly_high":310.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3,"recycling_area":{"type":0,"points":[]}}}],"UAVs":[{"order":0,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":1,"areas_id":0,"role":1,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":2,"areas_id":0,"role":2,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":3,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":4,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0}]}',
                # 3*3 20uav
                # '{"decision":"configure","mission":[{"night_scene":0,"inclusive_phase":["draw_area","release_climb","split_in_and_out","return_and_recovery"],"draw_area":{"scan_width":5,"scout_area":[{"areas_id":1,"areas_type":0,"area_target_num":2,"area_fire_num":2,"area_point_num":3,"area_point_content":[{"lon":0.0,"lat":0.0,"alt":0.0},{"lon":0.0,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":0.0,"alt":0.0}]}],"areas_no_fly":[],"areas_electronic_fence":[]},"release_climb":{"com_relay_fly_high":310.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3},"split_in_and_out":{"com_relay_fly_high":390.0,"area_search_fly_high":190.0,"fly_high":10.0,"fly_high_difference":3},"return_and_recovery":{"com_relay_fly_high":310.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3,"recycling_area":{"type":0,"points":[]}}}],"UAVs":[{"order":0,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":1,"areas_id":0,"role":1,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":2,"areas_id":0,"role":2,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":3,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":4,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":5,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":6,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":7,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":8,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":9,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":10,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":11,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":12,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":13,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":14,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":15,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":16,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":17,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":18,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":19,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0}]}',
                # # 3*3 15uav
                # '{"decision":"configure","mission":[{"night_scene":0,"inclusive_phase":["draw_area","release_climb","split_in_and_out","return_and_recovery"],"draw_area":{"scan_width":5,"scout_area":[{"areas_id":1,"areas_type":0,"area_target_num":2,"area_fire_num":2,"area_point_num":3,"area_point_content":[{"lon":0.0,"lat":0.0,"alt":0.0},{"lon":0.0,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":0.0,"alt":0.0}]}],"areas_no_fly":[],"areas_electronic_fence":[]},"release_climb":{"com_relay_fly_high":310.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3},"split_in_and_out":{"com_relay_fly_high":390.0,"area_search_fly_high":190.0,"fly_high":10.0,"fly_high_difference":3},"return_and_recovery":{"com_relay_fly_high":310.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3,"recycling_area":{"type":0,"points":[]}}}],"UAVs":[{"order":0,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":1,"areas_id":0,"role":1,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":2,"areas_id":0,"role":2,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":3,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":4,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":5,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":6,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":7,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":8,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":9,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":10,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":11,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":12,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":13,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":14,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0}]}',
                # 3*3 10uav
                '{"decision":"configure","mission":[{"night_scene":0,"inclusive_phase":["draw_area","release_climb","split_in_and_out","return_and_recovery"],"draw_area":{"scan_width":5,"scout_area":[{"areas_id":1,"areas_type":0,"area_target_num":2,"area_fire_num":2,"area_point_num":3,"area_point_content":[{"lon":0.0,"lat":0.0,"alt":0.0},{"lon":0.0,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":2.6979648177561914e-03,"alt":0.0},{"lon":2.6979648177561914e-03,"lat":0.0,"alt":0.0}]}],"areas_no_fly":[],"areas_electronic_fence":[]},"release_climb":{"com_relay_fly_high":310.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3},"split_in_and_out":{"com_relay_fly_high":390.0,"area_search_fly_high":190.0,"fly_high":10.0,"fly_high_difference":3},"return_and_recovery":{"com_relay_fly_high":310.0,"area_search_fly_high":190.0,"fly_high":90.0,"fly_high_difference":3,"recycling_area":{"type":0,"points":[]}}}],"UAVs":[{"order":0,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":1,"areas_id":0,"role":1,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":2,"areas_id":0,"role":2,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":3,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":4,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":5,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":6,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":7,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":8,"areas_id":0,"role":0,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0},{"order":9,"areas_id":0,"role":3,"can_fly_status":1,"lon":0.0,"lat":0.0,"alt":0.0}]}',
                # 3*3 20uav
                # '',
                #
                '{    "decision":"start"}',
                # '{"decision":"execute","mission":[{"static_target":{"target_todo":[{"id":1001,"expect_num":1,"lon":0.1,"lat":0.1,"alt":0}]}}],"UAVs":[{"order":0,"areas_id":1,"target_id":1001,"role":0,"finished_point_id":1,"can_fly_status":0,"lon":0.1,"lat":0.1,"alt":10,"break_lon":0.1,"break_lat":0.1,"break_alt":10},{"order":2,"areas_id":1,"target_id":1001,"role":0,"finished_point_id":1,"can_fly_status":0,"lon":0.5,"lat":0.5,"alt":10,"break_lon":0.1,"break_lat":0.1,"break_alt":10}]}',
                # '{"decision":"execute","mission":[{"static_target":{"target_todo":[]}}],"UAVs":[{"order":3,"areas_id":1,"target_id":1001,"role":0,"finished_point_id":777777777,"can_fly_status":1,"lon":0.1,"lat":0.1,"alt":10,"break_lon":0.1,"break_lat":0.1,"break_alt":10}]}',
                # '',
                # ,
            ]
        self.gateway    = Gateway()
        self.parser     = Parser()
        self.responder  = Responder()
        self.vis_init()
        self.init_uavs()
        pass

    def vis_init(self):
        import matplotlib.pyplot as plt
        from simpy.render.vis import Visualizer
        # 设置字体为系统已有的中文字体，比如SimHei或Microsoft YaHei
        plt.rcParams['font.sans-serif']     = ['SimHei']  # 设置 SimHei 字体
        plt.rcParams['axes.unicode_minus']  = False  # 解决负号显示为方块的问题

        fig=plt.figure(figsize=(6,7), dpi=125)
        fig.canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        # self.visualizer = Visualizer(fig, self.SharedInf, self.args["save_fig"])  # 初始化空的: 2D信息+3D显示 info_ax,ax
        self.visualizer = Visualizer(fig, SharedInf=None, save_flag=None)  # 初始化空的: 2D信息+3D显示 info_ax,ax
        tracker.debug_tracker.instance.current_position_print()            


    def init_uavs(self):
        json_data = json.loads(self.mlist[0])
        self.gateway.send(json_data)
        json_dict   = self.gateway.recv() # config 配置成功
        self.parser.parse_uav_init(json_dict=json_dict)
        
        json_data = json.loads(self.mlist[1])
        self.gateway.send(json_data)
        _           = self.gateway.recv() # start 收到空
        tracker.debug_tracker.instance.current_position_print()            
        

    def move_scan_tasker_status_updata(self):
        for uavid, uav in entity.uav.order2uav.items():
            if protocol.uav.role(uav.role) == protocol.uav.role.tasker:
                if uav.task_status == protocol.uav.task_status.scan:
                    uav.wait_message_has_send__make_sure_only_once = 0
                    target = uav.scan()
                    if target != None:
                        tracker.debug_tracker.instance.pf('target != None')            
                        tracker.debug_tracker.instance.current_position_print()            
                        ret_json_dict = self.responder.respond_scan_target(
                            uav,
                            target,
                        )
                        self.gateway.send(ret_json_dict)
                        get_json_dict = self.gateway.recv()
                        self.parser.parse_scan_target(get_json_dict)

                        self.can_skip = False
                        entity.msg.uav  += [ f'无人机{uav.id}发现' ]
                        entity.msg.gcs  += [ f'对无人机{uav.id}进行识别打击确认' ]
                        entity.msg.jq   += [ f'调度广域搜索机进行监视打击结果' ]
                        
                        uav.task_status = protocol.uav.task_status.watch
                    else:
                        uav.move(
                            dt          =self.dt, 
                            velocity    =self.tasker_velocity,        
                            )
                elif uav.task_status == protocol.uav.task_status.watch:
                    uav.wait_message_has_send__make_sure_only_once = 0
                    surveil_uav = list(entity.uav.order2surveil.values())[0]
                    surveil_reach_dist = np.linalg.norm(surveil_uav.pos[:2] - uav.pos[:2])
                    if surveil_reach_dist < 20:
                        uav.task_status     =   protocol.uav.task_status.attack
                        uav.xyzlist_index   =   0   
                        uav.xyzlist_list    =   [entity.uav.targetid_2_target[uav.target_id].pos.tolist()]    
                        uav.wp_id_list      =   [0] 
                        
                        entity.msg.uav  += [ f'广域搜索机{surveil_uav.id}监测目标' ]
                        entity.msg.gcs  += [ f'让任务机飞机打击' ]
                        entity.msg.jq   += [ f'' ]
                        
                    pass
                elif uav.task_status == protocol.uav.task_status.attack:
                    uav.wait_message_has_send__make_sure_only_once = 0
                    uav.move(
                        dt          =self.dt, 
                        velocity    =self.attach_velocity,        
                        )
                    uav_target_dist = np.linalg.norm(entity.uav.targetid_2_target[uav.target_id].pos[:2] - uav.pos[:2])
                    if uav_target_dist < 2:
                        uav.task_status                                    = protocol.uav.task_status.dead
                        entity.uav.targetid_2_target[uav.target_id].active = False
                        entity.msg.uav  += [ f'任务机飞机{uav.id}打击阵亡' ]
                        entity.msg.gcs  += [ f'' ]
                        entity.msg.jq   += [ f'' ]
                        
                elif uav.task_status == protocol.uav.task_status.dead:
                    pass
                elif uav.task_status == protocol.uav.task_status.wait:
                    if uav.wait_message_has_send__make_sure_only_once == 0:
                        tracker.debug_tracker.instance.current_position_print()            
                        uav.wait_message_has_send__make_sure_only_once =1 
                        ret_json_dict = self.responder.respond_wait(
                            uav,
                        )
                        self.gateway.send(ret_json_dict)
                        get_json_dict = self.gateway.recv()
                        self.parser.parse_wait(get_json_dict)

                        self.can_skip = False
                        # entity.msg.uav  += [ f'' ]
                        # entity.msg.gcs  += [ f'' ]
                        # entity.msg.jq   += [ f'' ]
                        
                        if uav.task_status == protocol.uav.task_status.scan:
                            uav.move(
                                dt          =self.dt, 
                                velocity    =self.tasker_velocity,        
                                )
                            entity.msg.uav  += [ f'无人机{uav.id}完成一段扫描任务' ]
                            entity.msg.gcs  += [ f'' ]
                            entity.msg.jq   += [ f'重新分配新的任务' ]
                        else:
                            entity.msg.uav  += [ f'无人机{uav.id}完成扫描任务' ]
                            entity.msg.gcs  += [ f'' ]
                            entity.msg.jq   += [ f'' ]
                        # uav.task_status = protocol.uav.task_status.scan
                    pass
                else:
                    tracker.debug_tracker.instance.current_position_print()
                    raise ValueError("Invalid input: GCS JSON decision.")
                    uav.move(
                        dt          =self.dt, 
                        velocity    =self.tasker_velocity,        
                        )
            else:
                uav.move(
                    dt          =self.dt, 
                    velocity    =self.surveil_velocity,        
                    )
        
        pass


    def init_frame(self, ):
        self.eps         = 1000
        self.ignore_range_list = [
                # [0,1000],
                ######################## 3*3 5uav
                # t = 62'
                # t = 339'
                # t = 349'
                # t = 677'
                # t = 1000'
                [1,60],     # watch target
                [90,335],   #
                [365,673], # help
                [703,1000], # help
                ########################
                # [0,62],     # debug
            ]# 5  : 3*3
        self.T_period    = 10
        
        pass

    def init_speed_time_interval_system_testing(self):
        #################### move scan 系统改变 #################### 
        self.level_max_can_skip     = True
        self.can_skip               = True
        self.dt                     = 1
        self.tasker_velocity        = 2*5
        self.surveil_velocity       = 2*10
        self.attach_velocity        = 2*10
        pass

    def init_speed_time_interval_real_record(self):
        self.level_max_can_skip     = False
        self.T_period               = 1
        self.dt                     = 1
        self.tasker_velocity        = 2*5
        self.surveil_velocity       = 2*10
        self.attach_velocity        = 2*10        
        pass

    def run(self):
        self.init_frame()
        self.init_speed_time_interval_system_testing()
        # self.init_speed_time_interval_real_record()


        self.visualizer.update()
        import matplotlib.pyplot as plt
        plt.pause(10)
        # input('================')
        
        # 运行主控制逻辑，协调各个组件的工作流程
        for t in range(self.eps):
            
            period_condition = t % self.T_period != 0 
            range_condition = any(
                ignore_range[0] <= t < ignore_range[1]
                for ignore_range in self.ignore_range_list
            )
            
            self.move_scan_tasker_status_updata()
            
            if self.level_max_can_skip:
                if self.can_skip:
                    if period_condition or range_condition:
                        continue
                else:
                    self.can_skip = True # 只显示一次，又可以继续跳过
            self.visualizer.update()
            
            # self.plot_2d() # 不能直接 把 LQY 的代码拿过来直接用
            import matplotlib.pyplot as plt
            # plt.pause(0.1)
            
            tracker.debug_tracker.instance.pf('          ------------>  t = '+str(t))
            tracker.debug_tracker.instance.current_position_print()            
            
            
            # 处理执行结果      # 负责处理任务执行的结果。它可以将结果发送到指定的目标（例如数据库、消息队列或其他系统），用于进一步处理或存储。

s = Simpy()
s.run()