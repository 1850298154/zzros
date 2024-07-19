import os
import random
import yaml
import math
import datetime
from typing import Literal, Union, Dict, List, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches

from .robotR import UAV, Target, SharedInformation, Building, Plain, PolygonRegion, DefenseTask, Task
from .planner import InitPlanner, OnlinePlanner

#用于模拟虚拟世界中的多个无人机和目标
class World(object):
    def __init__(self, args:Dict, uav_id_2_xylist, scan_areas_id=0) -> None: # # 加载到self._config # 并构造四个对象容器 uav_dict,target_dict,building_dict,plain # ,load_world_config(),construct_world()
        self.CONSTANTS_RADIUS_OF_EARTH = 6371000.       # meters (m)
        self.args = args
        self.output_type = -1
        self.load_world_config() # 加载到self._config  # 从配置文件world_config.yaml
        self.construct_world(uav_id_2_xylist)   # SharedInf创建+更新  # 构造robots,building,plain,planner  #  从self._config读出map,width,length,timestep,random_seed信息-->到self成员变量
        now = datetime.datetime.now()  # datetime.datetime(2024, 2, 26, 9, 56, 14, 638899)
        # 生成文件名
        self.logfile = "log/" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".log"  # self.logfile == 'log/2024-02-26_09-56-14.log'
        self.assignment = {} # uav_id: task_id
        self.scan_areas_id = scan_areas_id
    
    def load_world_config(self):
        # folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config\\')  # folder_path = 'f:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\src\\config\\'
        # folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config')  # folder_path = 'f:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\src\\config\\'
        # config_path = os.path.join(folder_path, self.args["world_config_file"])  # config_path = 'f:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\src\\config\\world_config.yaml'
        config_path = self.args["world_config_file"]  # config_path = 'f:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\src\\config\\world_config.yaml'
        current_directory = os.path.dirname(os.path.abspath(__file__))
        config_path = current_directory + '/' + config_path
        # print('config_path')
        # print(config_path)
        with open(config_path, "r", encoding='utf-8') as stream:
            try:
                self._config = yaml.safe_load(stream)  # self._config = {'world': {'timestep': 0.1, 'random_seed': 10}, 'planner': {'search_height': 30}, 'map': {'width': 600, 'length': 600, 'height': 40, 'building': {...}}, 'robots': {'UAV': {...}, 'Target': {...}}}
            except yaml.YAMLError as exc:
                print(exc)
    
    @property
    def config(self) -> Dict:
        return self._config

    @property
    def uav_dict(self) -> Dict[int, UAV]: # Dict{uav_id, UAV}
        return self._uav_dict
    
    @property
    def target_dict(self) -> Dict[int, Target]: # Dict{target_id, Target}
        return self._target_dict

    @property
    def building_dict(self) -> Dict[int, Building]: # Dict{building_id, Building}
        return self._building_dict

    @property
    def plain(self) -> Union[PolygonRegion, Plain]: # Dict{building_id, Building}
        return self._plain
    
    @property
    def random_seed(self)->int:
        return self._random_seed

    @property
    def map_width(self)->float:
        return self._map_width

    @property
    def map_length(self)->float:
        return self._map_length

    @property
    def timestep(self) -> int:
        return self._timestep

    def construct_world(self, uav_id_2_xylist):
        self._map_width = self.config['map']['width']  # 600
        self._map_length = self.config['map']['length'] # 600
        self._timestep = self.config['world']['timestep'] # 0.1
        self._random_seed = self.config['world']['random_seed'] # 10
        np.random.seed(self.random_seed) # 10
        random.seed(self.random_seed) # 10
        # scout_range 来自 uav , 之后传给 plain , 从 plain 再给 Poly plain
        self.SharedInf = SharedInformation(
                            self.config,
                            self.args
                            )  #  两个配置字典：config,args # 两个对象容器： # task_dict,back_task_dict
        self.construct_robots(uav_id_2_xylist)  # _uav_dict,_target_dict
        self.construct_building()  # 一个对象容器： _building_dict
        self.construct_plain()  # 只有一个_plain 最后用多边形进行
        self.construct_planner()  # 两个规划器 init_planer,online_planner
        self.SharedInf.update(self.uav_dict, self.target_dict, self.building_dict, self.plain)  # 加入构造好的实例对象： # uav_dict,target_dict,building_dict,plain # 目的是 同步共享

    def construct_robots(self, uav_id_2_xylist:Dict):
        self._uav_dict:Dict[int, UAV]={}
        self.tasked_uav_dict:Dict[int, UAV]={}
        self._target_dict:Dict[int, Target]={}

        uav_config:Dict = self.config["robots"]["UAV"]  # uav_config = {'num': 2, 'velocity': 20, 'scout_range': -1, 'theta': 40, 'HFOV': 24.9, 'VFOV': 14.7, 'x': 250, 'y': 400, 'z': 30}
        # for uav_id in range(uav_config["num"]):
        for uav_id, xylist in uav_id_2_xylist.items():
            self._uav_dict[uav_id] = UAV(id=uav_id,
                                        #  pos=np.array([uav_config["x"], uav_config["y"], uav_config["z"]]) + np.array([100*np.random.rand(),100*np.random.rand(),0]), # 第三个random高度为0, 可以设置, 控制无人机的起飞高度
                                         pos=np.array([xylist[0], xylist[1], 0]), # 第三个random高度为0, 可以设置, 控制无人机的起飞高度
                                         SharedInf=self.SharedInf
                                         )  # 除了 Robot 的 id,pos,SharedInf # 还有 UAV(Robot): 的 --> 任务航信,<1>,,init_pos,<2>,,start_point,travel_traj,search_traj,<3>,,task,,,仿真 3d visualization,<1>,,yaw,pitch,roll,<2>,,velocity,dt,,,视场角 camera FOV,<1>,,scout_range,offset,,建筑物 avoiding building,<1>,,cur_near_building,pre_near_building
        
        X,Y,Z = self.config["robots"]["Target"]["x"], self.config["robots"]["Target"]["y"], self.config["robots"]["Target"]["z"]
        for target_id in range(self.config["robots"]["Target"]["num"]):
            self._target_dict[target_id] = Target(id=target_id,
                                               pos=np.array([X[target_id], Y[target_id], Z[target_id]]),
                                               SharedInf=self.SharedInf
                                               )  # 除了 Robot 的 id,pos,SharedInf # 还有 Target(Robot): 的  被无人机发现 discovered

        
    
    def construct_building(self):
        self._building_dict:Dict[int, Building] = {}
        b_config:Dict = self.config["map"]["building"]
        b_id = 0
        for x, y in zip(b_config["x"], b_config["y"]):
            self._building_dict[b_id] = Building(id=b_id, anchor=np.array([x,y]), width=b_config["width"], length=b_config["length"], height=b_config["height"], floors=b_config["floors"])
            b_id += 1
    
    def construct_plain(self):
        # scout_range = self.uav_dict[0].scout_range
        scout_range = list(self.uav_dict.values())[0].scout_range
        self._plain = Plain(anchor=np.array([0,0]), width=500, length=500, scout_range=scout_range)  # # 地面范围,<1>,anchor,width,length,size,uncertainty_map,<2>,x1,x2,y1,y2,,# 飞机入口和宽度,<1>,entrance,uav_num,scout_range


    def construct_planner(self):
        self.init_planer = InitPlanner(self.SharedInf)  # 除了PlannerBase 的  SharedInf # 还有 InitPlanner(PlannerBase) 的 assignment
        self.online_planner = OnlinePlanner(self.SharedInf)  # 除了PlannerBase 的  SharedInf # 还有 OnlinePlanner(PlannerBase) 的  attack_flag 布尔


    # def add_uav_from_scheduler(self, uav_data):
        # x, y = self.gps_to_xy(uav_data["latitude"], uav_data["longitude"])
        # uav = UAV(id=uav_data["order"], pos=np.array([x, y, uav_data["alt"]]),
        #                                  search_height=self.gcs_config["investigateSearch"]["alt"],
        #                                  config=self.config["robots"]["UAV"])
        # home_pt = np.array(self.gps_to_xy(uav_data["homelatitude"], uav_data["homelongitude"]))
        # uav.add_home_point(home_pt)
        # uav.add_fly_mode(int(uav_data["flyMode"]))
        # self.uav_dict.update({uav_data["order"]: uav})
    def add_uav_from_scheduler(self, uav, height_difference_i=0):
        uav_id = int(uav["order"])
        x, y = self.gps_to_xy(uav["lat"], uav["lon"])
        self.tasked_uav_dict[uav_id] = UAV(id=uav_id,
                                        pos=np.array([x, y, uav["alt"]]),
                                        SharedInf=self.SharedInf,
                                        latlon=(uav["lat"], uav["lon"]),
                                        height_difference_i=height_difference_i,
                                        )

    def simulate(self):
        pass

    def init_plan(self):  # 航线分配 + {uav:task} # init_planer + assignment
        self.init_planer.plan() #  <1>分配完整蛇形航线, <2>构造任务记录信息池, <3>偏移, <4>演示动画,   # <1>uavs_traj_plan_edge(),<2>construct_task(),<3>task_offset(),<4>debug_display_polygon()

        # test 去掉 开头 结尾 在 多边形 边上 的 短尾巴
        for task in self.SharedInf.task_dict.values():
            self.online_planner.remove_edge_search_part(task)
        # import traceback
        # traceback.print_exc() # NoneType: None
        # print('self.init_planer.debug_display_polygon()')
        # self.init_planer.debug_display_polygon() # <4>演示动画,   #<4>debug_display_polygon()
        self.init_planer.new_keypoint_debug_display_polygon() # <4>演示动画,   #<4>debug_display_polygon()

        turn_count = 0
        for task in self.SharedInf.task_dict.values():
            if task.total_index > 2:
                turn_count += (task.total_index - 2)
        # print(f"Turn Count: {turn_count}")  # 用于调试 # 目前不需要 
        self.assignment = {uav.id:uav.task.task_id for uav in self.uav_dict.values()}  # {0: ('plain', 0), 1: ('plain', 1)}
        return

    def online_plan(self) -> Tuple[int, Dict]:
        """ type: "返回类型 0规划 1不需要处理 2需要处理某些无人机中断并重新上传"
        """
        self.online_planner.plan()  # 重新将待分配uav和task组合 # 检查了去重
        self.online_planner.debug_display_polygon()
        flag = 1
        new_assign =  {uav.id:uav.task.task_id for uav in self.uav_dict.values() if uav.active == True} # 新纪录  # {1: ('plain', 1), 2: ('plain', 2), 3: ('plain', 3), 4: ('plain', 4)}  ############# {10: ('plain', 18), 1: ('plain', 1), 17: ('plain', 17)} # 把 17号 任务分给了 10
        # FIXME 这里需要更新判断逻辑，尤其是当一个任务被拆分的时候
        changed_uav = {} # uav_id: (old task, new task)
        for uav_id, task_id in self.assignment.items(): # 老记录  # assignment=={1: ('plain', 1), 2: ('plain', 2), 3: ('plain', 3), 4: ('plain', 4)}  # 2test老记录 # {10: ('plain', 10), 1: ('plain', 1), 17: ('plain', 17)}  # 3test   assignment=={10: ('plain', 10)}
            uav = self.SharedInf.uav_dict[uav_id]
            if not uav.active:  # 无人机活着 # 一定在 new_assign 中 # 进行后面处理
                continue
            new_task_id = new_assign[uav_id]  # ('plain', 4) # 2test ('plain', 18) ## 3test   new_assign=={10: ('plain', 10)}
            if new_task_id != task_id:  # ('plain', 4) != ('plain', 4) # if 没有分配任务 && 找不到完成的任务  then changed_uav = {}  # 2test 老 ('plain', 10)  # 3test  new_task_id==('plain', 10)
                flag = 2
                changed_uav[uav_id] = (task_id, new_task_id) #  changed_uav  ==  {10: (('plain', 10), ('plain', 39)), 17: (('plain', 17), ('plain', 40))}
        
        self.assignment = new_assign # if 没有分配任务 && 找不到完成的任务  then changed_uav = {} #   # uav_id 和  uav的任务id ('plain', 4)  # 用来记录旧任务 ， 和 新任务比较   # {10: ('plain', 1)}  # {10: ('back', 10)}
        
        return int(flag), changed_uav  # changed_uav ==  {27: (('plain', 27), ('plain', 26))}   # 2test 从xx改变到 xx {10: (('plain', 10), ('plain', 18))}  # 3test {10: (('plain', 18), ('plain', 1))}  # {10: (('plain', 1), ('back', 10))}

    def vis_init(self):
        import matplotlib.pyplot as plt
        from src.render.vis import Visualizer
        fig=plt.figure(figsize=(6,7), dpi=125)
        fig.canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        self.visualizer = Visualizer(fig, self.SharedInf, self.args["save_fig"])  # 初始化空的: 2D信息+3D显示 info_ax,ax

    def update_world_vis(self):
        self.visualizer.update()

    # def status_assigned_mission(self, uav_dict, uav, recv_data):
    def status_assigned_mission(self, uav_dict, recv_data):
        """
        mission_type
            0.待分配  
            1.平原扫描
            2.伴随式掩护  
        fly_status
            0.已损坏
            1.飞行中
        task_status
            0.待分配;
            1.平原扫描;
            2.双C编队掩护;
            3.追踪;
            4.放弃跟踪;
            5.打击;
            6.放弃打击;
        scout_end
            0未结束 
            1结束
        """
        mission = recv_data["teams"][0]["mission"]

        uav_dict["mission_new"] = mission["mission_new"]
        uav_dict["mission_type"] = mission["mission_type"]
        uav_dict["destination_gps"] =  {            
            # "lon" : mission["destination_gps"]["lon"],
            # "lat" : mission["destination_gps"]["lat"],
            # "alt" : mission["destination_gps"]["alt"]
            # # # # # # # # # # # # # # # # # # # # # # # # 
            # 60所 来的时候 去掉了 destination_gps 
            # destination_gps 主要是用来 将 所有机头 统一指向 一个方向的时候使用 
            # 龙舟-每个飞机 需要 destination_gps 指明方向
            "lon" : mission.get("destination_gps",{}).get("lon", -1),
            "lat" : mission.get("destination_gps",{}).get("lat", -1),
            "alt" : mission.get("destination_gps",{}).get("alt", -1)
        }
        # uav_dict["order"] = int(uav.id)
        uav_dict["task_status"] = mission["mission_type"] # 无人机只要派出执行任务 # 一定是要做的 mission # 阵亡的无人机不分配任务 
        uav_dict["scout_end"] = 0

        return uav_dict
    
    def not_first_received_send_assist_point(self, takeoff, uav:UAV, recv_data): # 非线性打击 平原扫描 # 打击目标后 # 补充无人机进行扫描的点
        points = []
        
        
        if uav.active == True and self.output_type == 2:  # 不活跃、或者不更新（output_type==1 即  uav_data["type"] = self.output_type-1 ）
            task:Task = uav.task # 2test  task.task_id == ('plain', 18)
            # if uav.task.task_id[0] == "back":
            #     uav_data["finishType"] = 1
            
            # wp_id = 0
            # # takeoff = False # zyt 防止重复点， 起飞高度要升高到 更高，防止碰撞
            # if takeoff:  # 起飞  #现在是false
            #     pt = {}
            #     pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
            #     pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
            #     pt["alt"] = float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
            #     pt["yaw"] = float(0)  # "yaw": "航向（double）"
            #     wp_id += 1
            #     points.append(pt)
            
            # # scan_all_finished 不走这一条线路  走另一个条分支
            # if uav.task.task_id[0] == "back":
            #     pass # 需要加逻辑 在这里加
            
            wp_id = 0 # 直接升高 为了给飞机 当前位置   然后 飞机中间件 好指向方向
            pt = {}
            pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
            for u in recv_data["teams"][0]["UAVs"]:
                if u['order'] == uav.id:
                    lat = u['lat']
                    lon = u['lon']

            # pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
            pt["lat"], pt["lon"] = lat, lon
            # pt["alt"] =  5 + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞

            try:
                fly_high_difference = float(recv_data["teams"][0]["mission"]["fly_high_difference"])
                # search_height = float(self.config["planner"]["search_height"])
                # pt["alt"] = fly_high_difference + search_height
            except KeyError:
                fly_high_difference = 5
                print("未找到 'fly_high_difference' 键，无法计算高度差。 默认 ", fly_high_difference)

            # pt["alt"] =  float(recv_data["teams"][0]["mission"]["fly_high_difference"]) + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
            # pt["alt"] =  fly_high_difference + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
            pt["alt"] = float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
            pt["yaw"] = float(0)  # "yaw": "航向（double）"
            points.append(pt)
            
            # task.finished_point_ID 多一个 起点  比key
            # wp_id=finish = 10 
            # wp_id=finish = 11 
            # 发送的
            # wp_id=10 金纬度和yaw是对上的
            #实际上 我要发送 index 是上一个
            for index in range(0, task.total_index): # key的下标 
                # 其他人的 起飞点  就 不用 帮忙扫描了
                # index 数量要少
                # index == task.finished_point_ID-1 就需要了
                # index == 0 的时候  task.finished_point_ID == 1
                # if index < task.finished_point_ID or task.finished_point_ID == 0: # 从完成的 task.finished_point_ID 开始 发送航点
                # if index < task.finished_point_ID-1 or task.finished_point_ID == 0: # 从完成的 task.finished_point_ID 开始 发送航点 # 2test task.finished_point_ID==-1
                if index < task.finished_point_ID-1 and task.finished_point_ID >= 2: # 1. finished==0=1都不能跳过key , index从0开始  # 2. finished==2 , index作为key从1开始 , 0跳过 1发送   # 从完成的 task.finished_point_ID 开始 发送航点 # 2test task.finished_point_ID==-1 # task.finished_point_ID == -1    ###    if index < task.finished_point_ID-1 and task.finished_point_ID >= 1:  
                    wp_id += 1 # 从0 开始 # 不过发送的第一个可以 1、2... # 中间件思佳拿着个发给我 finished_point_id
                    continue

                wp_id += 1
                pt = {}
                if index == task.cur_index:
                    p1 = self.xy_to_gps(task.target_pos[0], task.target_pos[1])
                    p2 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                    # direction_vec = task.target_pos[:2] - uav.pos[:2]
                    direction_vec = np.array(p2) - np.array(p1)
                    # yaw = (np.arctan2(direction_vec[1], direction_vec[0]) * 180 - 90) / np.pi
                    yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360 # 经纬度不要搞反了
                    # yaw = (90 * index) % 360
                # elif index < task.total_index-1:
                #     # direction_vec = task.key_points[index+1] - task.key_points[index]
                #     p1 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                #     p2 = self.xy_to_gps(task.key_points[index+1][0], task.key_points[index+1][1])
                #     direction_vec = np.array(p2) - np.array(p1)
                #     yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360
                #     # yaw = (90 * index) % 360
                # else:
                #     yaw = 0
                    # yaw = (90 * index) % 360
                else:
                    p1 = self.xy_to_gps(task.key_points[index-1][0], task.key_points[index-1][1])
                    p2 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                    direction_vec = np.array(p2) - np.array(p1)
                    yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360
                
                pt["wp_id"] = int(wp_id)  # "wp_id": "航迹点 ID（int）",
                pt["lat"], pt["lon"] = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])  # "lat": "航迹点纬度（double）",
                pt["alt"] = float(task.key_points[index][2])  # "lon": "航迹点经度(double)",
                pt["yaw"] = float(yaw)  # "yaw": "航向（double）"

                # 应该是想  给 断点  但是 断点 位置不对
                # if index == task.finished_point_ID: 
                #     pt["lat"], pt["lon"] = task.uav_data["lat"], task.uav_data["lon"]
                    

                # wp_id += 1
                points.append(pt)
                
            if False:
                wp_id += 1
                pt = {}
                pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
                # for u in recv_data["teams"][0]["UAVs"]:
                #     if u['order'] == uav.id:
                #         lat = u['lat']
                #         lon = u['lon'] # 当前经纬度

                # pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
                pt["lat"], pt["lon"] = uav.init_latlon
                # pt["alt"] =  5 + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞

                try:
                    fly_high_difference = float(recv_data["teams"][0]["mission"]["fly_high_difference"])
                    # search_height = float(self.config["planner"]["search_height"])
                    # pt["alt"] = fly_high_difference + search_height
                except KeyError:
                    fly_high_difference = 5
                    print("second send 未找到 'fly_high_difference' 键，无法计算高度差。 默认 ", fly_high_difference)

                # pt["alt"] =  float(recv_data["teams"][0]["mission"]["fly_high_difference"]) + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
                pt["alt"] =  fly_high_difference + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
                pt["yaw"] = float(0)  # "yaw": "航向（double）"
                points.append(pt)
        return points


    def first_received_send_point(self, takeoff, uav, recv_data): # 非线性打击 平原扫描 # 第一次航迹规划
        points = []
        
        
        if uav.active == True and self.output_type == 2:  # 不活跃、或者不更新（output_type==1 即  uav_data["type"] = self.output_type-1 ）
            task:Task = uav.task
            # if uav.task.task_id[0] == "back":
            #     uav_data["finishType"] = 1
            
            wp_id = 0
            # # takeoff = False # zyt 防止重复点， 起飞高度要升高到 更高，防止碰撞
            # if takeoff:  # 起飞 # 一定是  True  因为是 first_received
            #     pt = {}
            #     pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
            #     pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
            #     pt["alt"] = float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
            #     pt["yaw"] = float(0)  # "yaw": "航向（double）"
            #     wp_id += 1 # 老方法臫takeoff
            #     points.append(pt)
            
            pt = {}
            pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
            # for u in recv_data["teams"][0]["UAVs"]:
            #     if u['order'] == uav.id:
            #         lat = u['lat']
            #         lon = u['lon'] # 当前经纬度

            # pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
            pt["lat"], pt["lon"] = uav.init_latlon
            # pt["alt"] =  5 + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞

            try:
                fly_high_difference = float(recv_data["teams"][0]["mission"]["fly_high_difference"])
                # search_height = float(self.config["planner"]["search_height"])
                # pt["alt"] = fly_high_difference + search_height
            except KeyError:
                fly_high_difference = 5
                print("first_received 未找到 'fly_high_difference' 键，无法计算高度差。 默认 ", fly_high_difference)

            # pt["alt"] =  float(recv_data["teams"][0]["mission"]["fly_high_difference"]) + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
            # pt["alt"] =  fly_high_difference + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
            pt["alt"] = float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
            pt["yaw"] = float(0)  # "yaw": "航向（double）"
            points.append(pt)
            
            for index in range(task.cur_index, task.total_index):
                pt = {}
                wp_id += 1

                if index == task.cur_index:
                    p1 = self.xy_to_gps(task.target_pos[0], task.target_pos[1])
                    p2 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                    # direction_vec = task.target_pos[:2] - uav.pos[:2]
                    direction_vec = np.array(p2) - np.array(p1)
                    # yaw = (np.arctan2(direction_vec[1], direction_vec[0]) * 180 - 90) / np.pi
                    yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360 # 经纬度不要搞反了
                    # yaw = (90 * index) % 360
                # elif index < task.total_index-1:
                #     # direction_vec = task.key_points[index+1] - task.key_points[index]
                #     p1 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                #     p2 = self.xy_to_gps(task.key_points[index+1][0], task.key_points[index+1][1])
                #     direction_vec = np.array(p2) - np.array(p1)
                #     yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360
                #     # yaw = (90 * index) % 360
                # else:
                #     yaw = 0
                    # yaw = (90 * index) % 360
                else:
                    p1 = self.xy_to_gps(task.key_points[index-1][0], task.key_points[index-1][1])
                    p2 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                    direction_vec = np.array(p2) - np.array(p1)
                    yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360
                
                pt["wp_id"] = int(wp_id)  # "wp_id": "航迹点 ID（int）",
                pt["lat"], pt["lon"] = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])  # "lat": "航迹点纬度（double）",
                pt["alt"] = float(task.key_points[index][2])  # "lon": "航迹点经度(double)",
                pt["yaw"] = float(yaw)  # "yaw": "航向（double）"

                # wp_id += 1# 老方法臫takeoff
                points.append(pt)
        
            # 规划最后一个航点  要求返航
            if False:
                wp_id += 1
                pt = {}
                pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
                # for u in recv_data["teams"][0]["UAVs"]:
                #     if u['order'] == uav.id:
                #         lat = u['lat']
                #         lon = u['lon'] # 当前经纬度

                # pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
                pt["lat"], pt["lon"] = uav.init_latlon
                # pt["alt"] =  5 + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞

                try:
                    fly_high_difference = float(recv_data["teams"][0]["mission"]["fly_high_difference"])
                    # search_height = float(self.config["planner"]["search_height"])
                    # pt["alt"] = fly_high_difference + search_height
                except KeyError:
                    fly_high_difference = 5
                    print("未找到 'fly_high_difference' 键，无法计算高度差。 默认 ", fly_high_difference)

                # pt["alt"] =  float(recv_data["teams"][0]["mission"]["fly_high_difference"]) + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
                pt["alt"] =  fly_high_difference + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
                pt["yaw"] = float(0)  # "yaw": "航向（double）"
                points.append(pt)
        
        return points
    
    
    def nonlinear_strike_json_wrapper_UDP_dict(self, first_received, takeoff, changed_uav:Dict, recv_data) -> Dict: # changed_uav == {27: (('plain', 27), ('plain', 26))}
        output = {}
        output["UAVs"] = []
        # output["version"] = self.online_planner._version
        # output["error"] = 0
        
        # output["plan"] = []

        # scan_all_finished = recv_data["teams"][0]["mission"]["scan_all_finished"]
        scan_all_finished = recv_data["teams"][0]["mission"].get("scan_all_finished", 0) # 默认 没有扫完 0

        for uav in self.uav_dict.values():
            if uav.id not in changed_uav.keys() and scan_all_finished==0: # 27 in changed_uav.keys  # 27 需要帮助其他人 
                continue  # 只要任务没有改变 无人机阵亡也算没有改变 # 就不用发送新任务信息
            elif scan_all_finished==1:
                pass #继续做处理 : 让所有无人机  全部回家
            if uav.task == None:
                continue  # 没有任务就不发
            if uav.task.task_id[0] == "back" and scan_all_finished == 0:
                print('uav.task.task_id[0] == "back" and scan_all_finished == 0:')
                print('uav.id','uav.task.task_id[1]')
                print(uav.id ,uav.task.task_id[1])
                continue
            uav_data = {}
            
            uav_data["order"] = int(uav.id)
            uav_data = self.status_assigned_mission(uav_dict=uav_data, recv_data=recv_data)
            # uav_data = self.status_assigned_mission(uav_dict=uav_data, uav=uav, recv_data=recv_data)
            
            # # # # # # # # # # # # # # # # # # 取消了 3个 字段 
            # tasktype={"plain":1,"target":3,"back":4,}
            # uav_data["type"] = self.output_type-1  # "type": "0 地面站不更新 1 地面站更新",
            # uav_data["task"] = tasktype[uav.task.task_id[0]]  # "task": "任务类型(int) 1 侦察任务 2 动态跟踪任务 3 自杀式打击任务",
            # uav_data["taskID"] = uav.task.task_id[1]  # "taskID": "任务序号(int)",  "taskID": 0,
            # uav_data["task_id"] = uav.task.task_id[1]  # "taskID": "任务序号(int)",  "taskID": 0,
            # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
            # uav_data["homeLat"], uav_data["homeLon"] = self.xy_to_gps(uav.init_pos[0], uav.init_pos[1])
            # uav_data["finishType"] = 0
            """            
            points = []
            
            
            if uav.active == True and self.output_type == 2:  # 不活跃、或者不更新（output_type==1 即  uav_data["type"] = self.output_type-1 ）
                task = uav.task
                # if uav.task.task_id[0] == "back":
                #     uav_data["finishType"] = 1
                
                wp_id = 0
                # takeoff = False # zyt 防止重复点， 起飞高度要升高到 更高，防止碰撞
                if takeoff:  # 起飞
                    pt = {}
                    pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
                    pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
                    pt["alt"] = float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
                    pt["yaw"] = float(0)  # "yaw": "航向（double）"
                    wp_id += 1
                    points.append(pt)
                
                
                for index in range(task.cur_index, task.total_index):
                    pt = {}
                    if index == task.cur_index:
                        p1 = self.xy_to_gps(task.target_pos[0], task.target_pos[1])
                        p2 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                        # direction_vec = task.target_pos[:2] - uav.pos[:2]
                        direction_vec = np.array(p2) - np.array(p1)
                        # yaw = (np.arctan2(direction_vec[1], direction_vec[0]) * 180 - 90) / np.pi
                        yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360 # 经纬度不要搞反了
                        # yaw = (90 * index) % 360
                    # elif index < task.total_index-1:
                    #     # direction_vec = task.key_points[index+1] - task.key_points[index]
                    #     p1 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                    #     p2 = self.xy_to_gps(task.key_points[index+1][0], task.key_points[index+1][1])
                    #     direction_vec = np.array(p2) - np.array(p1)
                    #     yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360
                    #     # yaw = (90 * index) % 360
                    # else:
                    #     yaw = 0
                        # yaw = (90 * index) % 360
                    else:
                        p1 = self.xy_to_gps(task.key_points[index-1][0], task.key_points[index-1][1])
                        p2 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                        direction_vec = np.array(p2) - np.array(p1)
                        yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360
                    
                    pt["wp_id"] = int(wp_id)  # "wp_id": "航迹点 ID（int）",
                    pt["lat"], pt["lon"] = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])  # "lat": "航迹点纬度（double）",
                    pt["alt"] = float(task.key_points[index][2])  # "lon": "航迹点经度(double)",
                    pt["yaw"] = float(yaw)  # "yaw": "航向（double）"

                    wp_id += 1
                    points.append(pt)
"""
            if first_received == True:
                points = self.first_received_send_point(takeoff, uav, recv_data)
            else:
                scan_all_finished = recv_data["teams"][0]["mission"]["scan_all_finished"]
                if scan_all_finished == 0:            
                    points = self.not_first_received_send_assist_point(takeoff, uav, recv_data)
                elif scan_all_finished == 1 and uav.task.task_id[0] == "back":
                    points = self.scan_all_finished_go_back_home(uav, recv_data)
                else:
                    print('ERROR   scan_all_finished == ',scan_all_finished)

            height_difference_i = uav.height_difference_i
            try:
                fly_high_difference = float(recv_data["teams"][0]["mission"]["fly_high_difference"])
                # search_height = float(self.config["planner"]["search_height"])
                # pt["alt"] = fly_high_difference + search_height
            except KeyError:
                fly_high_difference = 0
                print("first_received 未找到 'fly_high_difference' 键，无法计算高度差。 默认 ", fly_high_difference)
            points = self.create_height_difference(height_difference_i, fly_high_difference, points)

            uav_data["points"] = points

            # if uav.id in changed_uav.keys():
            #     output["plan"].append(uav_data)
            output["UAVs"].append(uav_data)

        return output
    
    def create_height_difference(self, height_difference_i, fly_high_difference, points):
        print('height_difference_i, fly_high_difference')
        print(height_difference_i, fly_high_difference)
        for p in points:
            p["alt"] += height_difference_i * fly_high_difference
        
        return points

    
    def scan_all_finished_go_back_home(self, uav, recv_data):
        points = []


        wp_id = 0 # 直接升高 为了给飞机 当前位置   然后 飞机中间件 好指向方向
        pt = {}
        pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
        for u in recv_data["teams"][0]["UAVs"]:
            if u['order'] == uav.id:
                lat = u['lat']
                lon = u['lon']

        # pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
        pt["lat"], pt["lon"] = lat, lon
        # pt["alt"] =  5 + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞

        try:
            fly_high_difference = float(recv_data["teams"][0]["mission"]["fly_high_difference"])
            # search_height = float(self.config["planner"]["search_height"])
            # pt["alt"] = fly_high_difference + search_height
        except KeyError:
            fly_high_difference = 5
            print("first_received 未找到 'fly_high_difference' 键，无法计算高度差。 默认 ", fly_high_difference)

        # pt["alt"] =  float(recv_data["teams"][0]["mission"]["fly_high_difference"]) + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
        # pt["alt"] =  fly_high_difference + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
        pt["alt"] = float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
        pt["yaw"] = float(0)  # "yaw": "航向（double）"
        points.append(pt)
        



        wp_id = 1
        pt = {}
        pt["wp_id"] = int(wp_id)  #  "wp_id": "航迹点 ID（int）",
        # for u in recv_data["teams"][0]["UAVs"]:
        #     if u['order'] == uav.id:
        #         lat = u['lat']
        #         lon = u['lon'] # 当前经纬度

        # pt["lat"], pt["lon"] = self.xy_to_gps(uav.pos[0], uav.pos[1])  # "lat": "航迹点纬度（double）",
        pt["lat"], pt["lon"] = uav.init_latlon
        # pt["alt"] =  5 + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞

        try:
            fly_high_difference = float(recv_data["teams"][0]["mission"]["fly_high_difference"])
            # search_height = float(self.config["planner"]["search_height"])
            # pt["alt"] = fly_high_difference + search_height
        except KeyError:
            fly_high_difference = 5
            print("second send 未找到 'fly_high_difference' 键，无法计算高度差。 默认 ", fly_high_difference)

        # pt["alt"] =  float(recv_data["teams"][0]["mission"]["fly_high_difference"]) + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
        # pt["alt"] =  fly_high_difference + float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
        pt["alt"] = float(self.config["planner"]["search_height"])  #  "alt": "无人机高度（double）",  # 可以修改 设置不同高度 # 防止相撞
        pt["yaw"] = float(0)  # "yaw": "航向（double）"
        points.append(pt)
        return points
    # forgo
    def json_wrapper(self, takeoff, changed_uav:Dict) -> Dict:
        output = {}
        output["version"] = self.online_planner._version
        output["error"] = 0
        output["type"] = self.output_type
        output["plan"] = []
        for uav in self.uav_dict.values():
            uav_data = {}
            uav_data["order"] = int(uav.id)
            uav_data["homeLat"], uav_data["homeLon"] = self.xy_to_gps(uav.init_pos[0], uav.init_pos[1])
            uav_data["finishType"] = 0
            points = []
            
            if uav.active == True and self.output_type == 2:
                task = uav.task
                if uav.task.task_id[0] == "back":
                    uav_data["finishType"] = 1
                
                wp_id = 0
                # 这个函数被弃用
                takeoff = False # zyt 防止重复点， 起飞高度要升高到 更高，防止碰撞
                if takeoff:
                    pt = {}
                    pt["wp_id"] = int(wp_id)
                    pt["latitude"], pt["longitude"] = self.xy_to_gps(uav.pos[0], uav.pos[1])
                    pt["alt"] = float(self.config["planner"]["search_height"])
                    pt["yaw"] = float(0)
                    wp_id += 1
                    points.append(pt)
                
                
                for index in range(task.cur_index, task.total_index):
                    pt = {}
                    if index == task.cur_index:
                        p1 = self.xy_to_gps(task.target_pos[0], task.target_pos[1])
                        p2 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                        # direction_vec = task.target_pos[:2] - uav.pos[:2]
                        direction_vec = np.array(p2) - np.array(p1)
                        # yaw = (np.arctan2(direction_vec[1], direction_vec[0]) * 180 - 90) / np.pi
                        yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360 # 经纬度不要搞反了
                        # yaw = (90 * index) % 360
                    # elif index < task.total_index-1:
                    #     # direction_vec = task.key_points[index+1] - task.key_points[index]
                    #     p1 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                    #     p2 = self.xy_to_gps(task.key_points[index+1][0], task.key_points[index+1][1])
                    #     direction_vec = np.array(p2) - np.array(p1)
                    #     yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360
                    #     # yaw = (90 * index) % 360
                    # else:
                    #     yaw = 0
                        # yaw = (90 * index) % 360
                    else:
                        p1 = self.xy_to_gps(task.key_points[index-1][0], task.key_points[index-1][1])
                        p2 = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                        direction_vec = np.array(p2) - np.array(p1)
                        yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360
                    
                    pt["wp_id"] = int(wp_id)
                    pt["latitude"], pt["longitude"] = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])
                    pt["alt"] = float(task.key_points[index][2])
                    pt["yaw"] = float(yaw)

                    wp_id += 1
                    points.append(pt)
            # uav_data["points"] = points # 包含0号 初始高度和飞行高度一样，  但是1号点经纬度不变， 高度增加
            uav_data["points"] = points[1:] # 包含0号 初始高度和飞行高度一样，  但是1号点经纬度不变， 高度增加

            # if uav.id in changed_uav.keys():
            #     output["plan"].append(uav_data)
            output["plan"].append(uav_data)

        return output

    def initialization_UDP(self,data):
        # 将原先的分成两个
        # # if data["teams"][0]["mission"]["mission_new"] == 1:
        mission = data["teams"][0]["mission"]
        if mission["mission_new"] == 1:
            print('     mission["mission_new"] == 1    ')
        else:
            print('     ERROR ::  第一次收到数据必须为新任务      ')
            print('     def initialization_UDP(self,data):      ')
            print('     mission["mission_new"] != 1    ', mission["mission_new"])
        
        if mission["mission_type"] == 1:
            print('     mission["mission_type"] == 1    ')
            self.init_nonlinear_strike(data=data["teams"][0]) # 默认只接受一个任务
        elif mission["mission_type"] == 2: 
            print('     mission["mission_type"] != 1    ', mission["mission_type"])
            # self.init_plan_accompany_shield(data=data["teams"][0]) # 默认只接受一个任务
            print('     放到外面 update_multi_mission_switch 做  这里只做  扫描平原')

    # def more_plan_accompany_shield_2_json(self, recv_data, first_received):
    #     print('             def more_plan_accompany_shield_2_json(self, data):  待开发ing   ')
    #     print('recv_data')
    #     print(recv_data)
    #     data = recv_data["teams"][0]
    #     print('data')
    #     print(data)
        
    #     # --------- Init GCS ------------------
    #     gcs_gps = data["mission"]["gcs_gps"] 
    #     from types import MappingProxyType
    #     gcs_gps_read_only = MappingProxyType(gcs_gps)
    #     if first_received:
    #         self.cur_gcs = {
    #             'x':0,
    #             'y':0,
    #             '2Dpt': np.array([0,0]),
    #             'gcsROD': gcs_gps_read_only,
    #         }
    #     else:
    #         x, y = self.gps_to_xy(
    #             gcs_gps_read_only["lat"], 
    #             gcs_gps_read_only["lon"]
    #         )
    #         self.cur_gcs = {
    #             'x':x,
    #             'y':y,
    #             '2Dpt': np.array([x,y]),
    #             'gcsROD': gcs_gps_read_only,
    #         }
                    
    #     # --------- Init UAV ------------------
    #     self.cur_uavs = {}
    #     # for uav in data["mission"]["UAVs"]:
    #     for uav in data["UAVs"]:
    #         uav_id = int(uav["order"])
    #         x, y = self.gps_to_xy(uav["lat"], uav["lon"])
    #         uav_read_only=MappingProxyType(uav)
    #         self.cur_uavs[uav_id] = {
    #             'x':x,
    #             'y':y,
    #             '2Dpt': np.array([x,y]),
    #             'uavROD': uav_read_only,
    #         }
        
    #     # --------- defenseTask Plan UAV ------------------
    #     self.cur_uavs = self.defenseTask.init_more_plan(self.cur_gcs, self.cur_uavs, data, first_received)                
    #     # --------- Wrapped in json ------------------
    #     return self.accompany_shield_json_wrapper_dict(self.cur_uavs, recv_data) # 返回包装好的dict 发送前会调用 json wrap


    # def init_plan_accompany_shield_2_json(self, recv_data,first_received):
    def init_more_plan_accompany_shield_2_json(self, recv_data,first_received):
        print('recv_data')
        print(recv_data)
        data = recv_data["teams"][0]
        print('data')
        print(data)
        
        # --------- Init DefenseTask --------------
        if first_received == True:
            # uav_height = data.get("mission",{}).get("fly_high", 18)
            default = self.config['planner']['search_height']
            uav_height = data.get("mission",{}).get("fly_high", default)
            self.defenseTask = DefenseTask('task_type','task_ID',uav_height=uav_height)

        # --------- Define gps2xy reference ---
        # 将 地面站|被保护的对象 作为 坐标轴原点 建立NED北东地坐标系==MAVLINK
        gcs_gps = data["mission"]["gcs_gps"] 
        if first_received:    
            self.ref_lat = gcs_gps["lat"]  # 41.7041358
            self.ref_lon = gcs_gps["lon"]  # 123.4332115
            self.ref_lat_rad = math.radians(float(self.ref_lat))    # 0.7278744814088395
            self.ref_lon_rad = math.radians(float(self.ref_lon))    # 2.1543159469855286

        # --------- Init GCS ------------------
        from types import MappingProxyType
        gcs_gps_read_only = MappingProxyType(gcs_gps)
        # self.cur_gcs = {
        #     'x':0,
        #     'y':0,
        #     '2Dpt': np.array([0,0]),
        #     'gcsROD': gcs_gps_read_only,
        # }
        if first_received:
            self.cur_gcs = {
                'x':0,
                'y':0,
                '2Dpt': np.array([0,0]),
                'gcsROD': gcs_gps_read_only,
            }
        else:
            x, y = self.gps_to_xy(
                gcs_gps_read_only["lat"], 
                gcs_gps_read_only["lon"]
            )
            self.cur_gcs = {
                'x':x,
                'y':y,
                '2Dpt': np.array([x,y]),
                'gcsROD': gcs_gps_read_only,
            }

        # --------- Init UAV ------------------
        self.cur_uavs = {}
        # for uav in data["mission"]["UAVs"]:
        for uav in data["UAVs"]:
            uav_id = int(uav["order"])
            x, y = self.gps_to_xy(uav["lat"], uav["lon"])
            uav_read_only=MappingProxyType(uav)
            self.cur_uavs[uav_id] = {
                'x':x,
                'y':y,
                '2Dpt': np.array([x,y]),
                'uavROD': uav_read_only,
            }
        
        # --------- defenseTask Plan UAV ------------------
        self.cur_uavs = self.defenseTask.init_more_plan(self.cur_gcs, self.cur_uavs, data, first_received)                
        # --------- Wrapped in json ------------------
        return self.accompany_shield_json_wrapper_dict(self.cur_uavs, self.cur_gcs, recv_data, first_received) # 返回包装好的dict 发送前会调用 json wrap
        
    def accompany_shield_json_wrapper_dict(self, cur_uavs, cur_gcs, recv_data, first_received):
        output = {}
        output["UAVs"] = []
        # for uav in self.uav_dict.values():
        for uav in cur_uavs.values():
            # if uav.id not in changed_uav.keys(): # 27 in changed_uav.keys  # 27 需要帮助其他人 
            #     continue  # 只要任务没有改变 无人机阵亡也算没有改变 # 就不用发送新任务信息
            uav_data = {}
            
            # uav_data["order"] = int(uav.id)
            uav_data["order"] = int(uav["uavROD"]["order"])
            uav_data = self.status_assigned_mission(uav_dict=uav_data, recv_data=recv_data)
            points = []
            
            wp_id = 0
            pt = {}
            pt["wp_id"] = int(wp_id)  # "wp_id": "航迹点 ID（int）",
            # pt["lat"], pt["lon"] = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])  # "lat": "航迹点纬度（double）",
            pt["lat"], pt["lon"] = (uav['uavROD']['lat'], uav['uavROD']['lon'])  # "lat": "航迹点纬度（double）",
            # pt["alt"] = float(task.key_points[index][2])  # "lon": "航迹点经度(double)",
            pt["alt"] = float(uav['protection_point'][2])  # "lon": "航迹点经度(double)",
            pt["yaw"] = float(0)  # "yaw": "航向（double）"
            points.append(pt)
            
            wp_id += 1
            pt = {}
            pt["wp_id"] = int(wp_id)  # "wp_id": "航迹点 ID（int）",
            # pt["lat"], pt["lon"] = self.xy_to_gps(task.key_points[index][0], task.key_points[index][1])  # "lat": "航迹点纬度（double）",
            pt["lat"], pt["lon"] = self.xy_to_gps(uav['protection_point'][0], uav['protection_point'][1])  # "lat": "航迹点纬度（double）",
            # pt["alt"] = float(task.key_points[index][2])  # "lon": "航迹点经度(double)",
            pt["alt"] = float(uav['protection_point'][2])  # "lon": "航迹点经度(double)",
            # start =  np.array(  [   
            #                         uav['x'], 
            #                         uav['y']   
            #                     ] 
            #                 )
            # end   =  np.array(uav['protection_point'][:2])
            
            start =  np.array(cur_gcs['2Dpt'])
            end   =  np.array(uav['protection_point'][:2])
            direction_vec = end - start
            # yaw = (-np.arctan2(direction_vec[0], direction_vec[1]) * 180 / np.pi + 90) % 360 
            yawR = np.arctan2( direction_vec[1], direction_vec[0] )
            yawD = yawR * 180 / np.pi 
            """我们知道正切函数的周期是 π \pi π，因此反正切函数一般也只取一个周期， y = a r c t a n ( x ) y=arctan(x) y=arctan(x)的定义域是 R \mathbb{R} R，值域是 ( − π 2 , π 2 ) (-\frac{\pi}{2}, \frac{\pi}{2}) (−2π​,2π​)，但是我们的要求是 θ ∈ [ − π , π ] \theta\in[-\pi, \pi] θ∈[−π,π]呀！
                别担心，Numpy早就贴心的准备好解决办法了，它不仅有np.arctan()函数，还准备了np.arctan2()函数，能够满足我们的需求。下面就来看看这两个函数有什么区别。
                the point (x2, x1). (Note the role reversal: the
                “y-coordinate” is the first function parameter, the 
                “x-coordinate” is the second.) 
                https://blog.csdn.net/B_DATA_NUIST/article/details/105792308 """
            pt["yaw"] = float(yawR)  # "yaw": "航向（double）"
            pt["yawD"] = float(yawD)  # "yaw": "航向（double）"
            points.append(pt)

            uav_data["points"] = points
            output["UAVs"].append(uav_data)

        return output
        
    def init_nonlinear_strike(self, data): # 第一个区域的第一个点作为坐标原点  # 初始化所有uav
        # 传入的是一个任务
        # data = data["teams"][0]  # 默认只接受一个任务
        
        # --------- Define gps2xy reference ---
        # self.ref_lat = data["mission"]["points"][0]["lat"]  # 41.7041358
        # self.ref_lon = data["mission"]["points"][0]["lon"]  # 123.4332115
        # self.ref_lat_rad = math.radians(float(self.ref_lat))    # 0.7278744814088395
        # self.ref_lon_rad = math.radians(float(self.ref_lon))    # 2.1543159469855286
        print('data   def init_nonlinear_strike(self, data): ')
        print(data)
        first_area_0 = data["mission"]["areas"][0]
        self.ref_lat = first_area_0[0]["lat"]  # 41.7041358
        self.ref_lon = first_area_0[0]["lon"]  # 123.4332115
        self.ref_lat_rad = math.radians(float(self.ref_lat))    # 0.7278744814088395
        self.ref_lon_rad = math.radians(float(self.ref_lon))    # 2.1543159469855286

        # --------- Init SharedInf ------------------
        # 默认
        default = self.config['planner']['search_height']  # SharedInf 来自 self.config
        
        # 尝试读取 地面站 json
        uav_height = data.get("mission",{}).get("fly_high", default)  # 二选一
        print('uav_height')
        print(uav_height)

        # 全局 同步 共享数据
        self.config['planner']['search_height'] = uav_height  # 保险起见 给 self.config 更新一下  # {'world': {'timestep': 0.1, 'random_seed': 10}, 'planner': {'search_height': 15}, 'map': {'width': 600, 'length': 600, 'height': 40, 'building': {...}}, 'robots': {'UAV': {...}, 'Target': {...}}}
        self.SharedInf.world_config['planner']['search_height'] = uav_height  # {'world': {'timestep': 0.1, 'random_seed': 10}, 'planner': {'search_height': 15}, 'map': {'width': 600, 'length': 600, 'height': 40, 'building': {...}}, 'robots': {'UAV': {...}, 'Target': {...}}}
        """
        id(self.SharedInf.world_config)
        1691610334336
        1691610334336
        id(self.config)
        """


        # --------- Init UAV ------------------
        self._uav_dict = {}
        # for uav in data["mission"]["UAVs"]:
        # for uav in data["UAVs"]:
        for i, uav in enumerate(data["UAVs"]):
            uav_id = int(uav["order"])
            x, y = self.gps_to_xy(uav["lat"], uav["lon"])
            self._uav_dict[uav_id] = UAV(id=uav_id,
                                         pos=np.array([x, y, uav["alt"]]),
                                         SharedInf=self.SharedInf,
                                         latlon=(uav["lat"], uav["lon"]),
                                         height_difference_i=i,
                                         )

        # --------- Init Plain ----------------
        # if data["mission"]["topology"] == 0 or data["mission"]["topology"] == '0':
        if True: # 取消拓扑字段 # 默认永远是多边形
            # scout_range = self.plain.scout_range  # 5.976508219484396
            default_scout_range = self._uav_dict[uav_id].scout_range
            # default = 12 #5.976508219484396
            scan_width = data.get("mission",{}).get("scan_width", default_scout_range * 2)  # 对接地面站 json
            scout_range = scan_width / 2
            # vertices = [self.gps_to_xy(p["lat"], p["lon"]) for p in data["mission"]["points"]]  # [(0.0, 0.0), (-101.08728450689489, -18.255704826644262), (-187.93016793916104, 73.48031196475847), (-162.542976061968, 158.9895444600783), (-8.293250285281932, 164.44864190284252)]
            # self._plain = PolygonRegion(vertices=vertices, scout_range=scout_range, uav_num=len(data["mission"]["UAVs"]))
            allocate_area = data["mission"]["areas"][self.scan_areas_id]  # 选择 world 中的 扫描地区 # 按顺序初始化
            # vertices = [self.gps_to_xy(p["lat"], p["lon"]) for p in first_area_0]  # [(0.0, 0.0), (-101.08728450689489, -18.255704826644262), (-187.93016793916104, 73.48031196475847), (-162.542976061968, 158.9895444600783), (-8.293250285281932, 164.44864190284252)]
            vertices = [self.gps_to_xy(p["lat"], p["lon"]) for p in allocate_area]  # [(0.0, 0.0), (-101.08728450689489, -18.255704826644262), (-187.93016793916104, 73.48031196475847), (-162.542976061968, 158.9895444600783), (-8.293250285281932, 164.44864190284252)]
            self._plain = PolygonRegion(vertices=vertices, scout_range=scout_range, uav_num=len(data["UAVs"]))
            ## ## multi        # forgo
            self.multi_plain:Dict[int, PolygonRegion] = {}
            areas = data["mission"]["areas"]
            for i, each_area in enumerate(areas):
                vertices = [self.gps_to_xy(p["lat"], p["lon"]) for p in each_area]
                self.multi_plain[i]=PolygonRegion(vertices=vertices, scout_range=scout_range, uav_num=len(data["UAVs"]))
        
        # ---------- Init Building Dict -------
        self._building_dict = {}

        # --------- Init Targets --------------
        self._target_dict = {}

        # --------- Update Planner ------------
        idx = np.random.choice(list(self.uav_dict.keys()))
        selected_uav = self.uav_dict[idx]
        self.init_planer.update_params(offset=selected_uav.offset,
                                       search_height=self.config["planner"]["search_height"])
        #  self.config["planner"]["search_height"]  这个参数 后续应该 是 地面站指定 : 高度 + 宽度   
        # 答案 # 发送的高度 直接从这里面获取 # 

        # --------- Update Plain --------------
        self._plain.update_start_point(selected_uav.pos[:2])
        ## ## multi
        for i, first_area_0 in enumerate(areas):
            self.multi_plain[i].update_start_point(selected_uav.pos[:2])

        # --------- Update SharedInf ----------
        # self.SharedInf.update(self.uav_dict, self.target_dict, self.building_dict, self.plain)
        self.SharedInf.update(self.uav_dict, self.target_dict, self.building_dict, self.plain, self.multi_plain)

    # gcs input # forgo
    def initialization(self, data):
        # --------- Define gps2xy reference ---
        self.ref_lat = data["missions"][0]["points"][0]["latitude"]
        self.ref_lon = data["missions"][0]["points"][0]["longitude"]
        self.ref_lat_rad = math.radians(self.ref_lat)
        self.ref_lon_rad = math.radians(self.ref_lon)

        # --------- Init Plain ----------------
        if data["missions"][0]["type"] == 0:
            scout_range = self.plain.scout_range
            vertices = [self.gps_to_xy(p["latitude"], p["longitude"]) for p in data["missions"][0]["points"]]
            self._plain = PolygonRegion(vertices=vertices, scout_range=scout_range, uav_num=len(data["uavs"]))
        
        # ---------- Init Building Dict -------
        self._building_dict = {}

        # --------- Init UAV ------------------
        self._uav_dict = {}
        for uav in data["uavs"]:
            uav_id = int(uav["order"])
            x, y = self.gps_to_xy(uav["latitude"], uav["longitude"])
            self._uav_dict[uav_id] = UAV(id=uav_id,
                                         pos=np.array([x, y, uav["alt"]]),
                                         SharedInf=self.SharedInf)

        # --------- Init Targets --------------
        self._target_dict = {}

        # --------- Update Planner ------------
        idx = np.random.choice(list(self.uav_dict.keys()))
        selected_uav = self.uav_dict[idx]
        self.init_planer.update_params(offset=selected_uav.offset,
                                       search_height=self.config["planner"]["search_height"])

        # --------- Update Plain --------------
        self._plain.update_start_point(selected_uav.pos[:2])

        # --------- Update SharedInf ----------
        self.SharedInf.update(self.uav_dict, self.target_dict, self.building_dict, self.plain)

    def gps_to_xy(self, lat, lon):  # 41.7041358  123.4332115
        lat_rad = math.radians(lat)  # 0.7278744814088395
        lon_rad = math.radians(lon)  # 2.1543159469855286

        sin_lat = math.sin(lat_rad)  # 0.6652842477494599   # 赤道面上投影
        cos_lat = math.cos(lat_rad)  # 0.746590161799923    # 轴投影
        ref_sin_lat = math.sin(self.ref_lat_rad)
        ref_cos_lat = math.cos(self.ref_lat_rad)

        cos_d_lon = math.cos(lon_rad - self.ref_lon_rad)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)  # 1.0
        c = math.acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c))

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * self.CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - self.ref_lon_rad) * self.CONSTANTS_RADIUS_OF_EARTH)

        return x, y


    def xy_to_gps(self, x, y):
        x_rad = float(x) / self.CONSTANTS_RADIUS_OF_EARTH
        y_rad = float(y) / self.CONSTANTS_RADIUS_OF_EARTH
        c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

        ref_sin_lat = math.sin(self.ref_lat_rad)
        ref_cos_lat = math.cos(self.ref_lat_rad)

        if abs(c) > 0:
            sin_c = math.sin(c)
            cos_c = math.cos(c)

            lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
            lon_rad = (self.ref_lon_rad + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))

            lat = math.degrees(lat_rad)
            lon = math.degrees(lon_rad)

        else:
            lat = math.degrees(self.ref_lat)
            lon = math.degrees(self.ref_lon)

        return float(lat), float(lon)
    
    def get_workload(self):
        sum = self.plain.get_workload() 
        # sum = self.plain.get_workload() * self.traverseNum
        
        # for b in self.building_dict.values():
        #     sum += b.get_workload()

        # return sum * self.level # FIXME
        return sum
    
    # forgo
    def write_log(self, s):
        with open(self.logfile, "a") as f:
            print(s, file=f)

    def plot_2d(self):
        ax = self.ax
        ax.cla()
        # ax.set_xlim(0, self.SharedInf.world_config["map"]["width"])
        # ax.set_ylim(0, self.SharedInf.world_config["map"]["length"])


        plain = self.SharedInf.plain
        polygon = patches.Polygon(plain.vertices, closed = True, fill = False)
        ax.add_artist(polygon)
        l = len(self.uav_dict)
        # rb = cm.get_cmap('rainbow', l)
        rb = plt.get_cmap('rainbow', l)

        for i, uav in self.SharedInf.uav_dict.items():
            if uav.active == False:
                continue
            ax.scatter(uav.pos[0], uav.pos[1], color=rb(i%l))
            task = uav.task
            traj = np.vstack((uav.pos, task.key_points[task.cur_index:task.total_index]))
            ax.plot(traj[:,0], traj[:,1], color=rb(i%l), linewidth=1)
            ax.text(uav.pos[0]+1, uav.pos[1]+1, s=f"{uav.id}")
        for task in self.SharedInf.task_dict.values():
            if not task.finished:
                cur_i = max(task.cur_index-1, 0)
                ax.plot(task.key_points[cur_i:task.total_index,0], task.key_points[cur_i:task.total_index,1], color="grey", zorder = 1, ls="dashed", linewidth=3)
        for building in self.SharedInf.building_dict.values():
            
            
            if building.discovered:
                color = "Crimson"
            else:
                color = "grey"
            polygon = patches.Polygon(building.anchors, closed = True, fill = False, color=color)
            ax.add_artist(polygon)
        
        # calculate distance between uavs
        uav_dist = self.calc_uav_dist()

        # plot distance matrix
        ax2:plt.Axes = self.ax2
        ax2.cla()
        ax2.imshow(uav_dist, cmap='viridis')
        for i in range(uav_dist.shape[0]):
            for j in range(uav_dist.shape[1]):
                ax2.text(j, i, f'{uav_dist[i, j]:.2f}', ha='center', va='center', color='white')
        
        plt.pause(0.01)
    