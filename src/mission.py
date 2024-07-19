import numpy as np
from src.worldR import World
from src.robotR import UAV, Target, SharedInformation, Building, Plain, PolygonRegion, DefenseTask, Task
from src.task.planner import InitPlanner, OnlinePlanner
import shapely
import math
from typing import Dict, List, Tuple


# 用于收发UDF的world类接口  # 判断 第一次 
# class MissionSwitch_JsonDict_Producer(World):
class MissionSwitch_JsonDict_Producer(object):
    def __init__(self) -> None:
        self.CONSTANTS_RADIUS_OF_EARTH = 6371000.       # meters (m)

        self.args = {"world_config_file":"world_config.yaml",
                "render": False,
                "save_fig": False,
                "attack": 1}
        # super().__init__(args)
        self.world = World(self.args)
        self.world_dict:Dict[int, World] = {}
        self.uav_dict:Dict[int, UAV] = {}
        self.polygons:Dict[int, shapely.Polygon] = {} # world_id: polygon
        self.t = 0
    

    def main_mission_switch_ret_JSON_dict(self, data:str, first_received:bool):
        # 将原先的分成两个
        # # if data["teams"][0]["mission"]["mission_new"] == 1:
        mission = data["teams"][0]["mission"]
        # if mission["mission_new"] == 1:
        #     print('     mission["mission_new"] == 1    ')
        # else:
        #     print('     ERROR ::  第一次收到数据必须为新任务      ')
        #     print('     def initialization_UDP(self,data):      ')
        #     print('     mission["mission_new"] != 1    ', mission["mission_new"])
        
        if mission["mission_type"] == 1:
            print('     mission["mission_type"] == 1    ')
            # self.world.init_nonlinear_strike(data=data["teams"][0]) # 默认只接受一个任务
            # return self.update_nonlinear_strike_2_json(data, first_received)
            return self.update_all_world_nonlinear_strike_2_json(data=data, first_received=first_received)
        elif mission["mission_type"] == 2: 
            print('     mission["mission_type"] != 1    ', mission["mission_type"])
            # self.world.init_accompany_shield(data=data["teams"][0]) # 默认只接受一个任务
            return self.update_accompany_shield_2_json(data, first_received) # 完成 第一次 或 再次 规划 # 结果 Dict 发送前调用 库wrap json
            
    def update_accompany_shield_2_json(self, data:str, first_received:bool):
        if first_received==True:
            # return self.world.init_plan_accompany_shield_2_json(data, first_received)  # 包括 要发送的 Dict # 发送前 调用 库 wrap json
            return self.world.init_more_plan_accompany_shield_2_json(data, first_received)  # 包括 要发送的 Dict # 发送前 调用 库 wrap json
        else:
            return self.world.init_more_plan_accompany_shield_2_json(data, first_received)  # 包括 要发送的 Dict # 发送前 调用 库 wrap json
            # return self.world.more_plan_accompany_shield_2_json(data, first_received)
    
    def update_all_world_nonlinear_strike_2_json(self, data:str, first_received:bool):
        return self.update_one_world_nonlinear_strike_2_json(data, first_received)  # 在内部完成 初次规划 再次规划 分开
        # 收
        if first_received==True:
            # 加载地面站数据和消息
            # 无人机分配到每个地区
            # 第一次离线初始化调度
            # 等价于 self.world.initialization_UDP(data) 

            self.all_world_load_GSC_data_messages(data, first_received)
            all_world_json_dict = {}
            takeoff = True
            for world in self.world_dict:
                self.world: World = world
                self.world.init_plan()
                changed_uav = {uav_id:() for uav_id in self.world.SharedInf.uav_dict.keys()} # uav_id: (old task, new task)
                one_world_json_dict = self.world.nonlinear_strike_json_wrapper_UDP_dict(first_received, takeoff, changed_uav, recv_data=data)
                all_world_json_dict += one_world_json_dict
            return all_world_json_dict
            
        elif  first_received==False:
            takeoff = False  # 只有第一次起飞 first_received==True
            return self.all_world_load_GSC_data_messages(data, first_received)
            


    def all_world_load_GSC_data_messages(self, data:str, first_received:bool):

        self.init_ref_lon_lat(data=data["teams"][0])

        # one_area = data["teams"][0]["mission"]["areas"][0]
        all_area = data["teams"][0]["mission"]["areas"] # list
        self.world_dict:Dict[int, World] = {} # 清空以前的缓存
        for scan_areas_id__world_id, one_area in enumerate(all_area):
            self.world_dict[scan_areas_id__world_id] = World(self.args, scan_areas_id__world_id)
            # 加载地面站数据和消息
            self.world:World = self.world_dict[scan_areas_id__world_id]
            self.world.init_nonlinear_strike(data=data["teams"][0])
            xys = [self.gps_to_xy(p["lat"], p["lon"]) for p in one_area]
            self.polygons[scan_areas_id__world_id] = shapely.Polygon(xys)

        
        # 1. 无人机分配到每个地区
        # 2. 删除无人机 
        # 3. 共享信息记得更新
        # 4. world中uav个数更新 等等
        
        # 1. 无人机分配到每个地区
        # calculate the total workload of each world(plain+building FIXME
        world_workload:Dict[int, float] = {world_id:world.get_workload() for world_id, world in self.world_dict.items()} # {1703468321573: 316656.20072370826, 1703468321574: 184721.47709301207}

        # assign uav for each world(at least 1)
        # uav_num = len(data["uavs"]) # 3
        uavs =  data["teams"][0]["UAVS"]
        uav_num = len(uavs) # 3
        if uav_num < len(self.world_dict):
            raise ValueError("uav number is less than plain number")
        # 区域需要的无人机个数
        uav_num_dict = self.split_integer(world_workload, uav_num)  # {1703468321573: 2, 1703468321574: 1}
        self.uav_id_2_world_id = {} # uav_id: world_id
        for height_difference_i, uav_data in enumerate(uavs): # 无人机原生数据 # {'order': 3, 'flyMode': 10, 'longitude': 123.4357, 'latitude': 41.7022999, 'alt': 0.002, 'homelongitude': 123.4357, 'homelatitude': 41.7022997, 'donePoints': []}
            xy = self.gps_to_xy(uav_data["lat"], uav_data["lon"])  # (-15.122568854863138, -397.7044767686871)
            world_dis_dict = {world_id:shapely.distance(self.polygons[world_id], shapely.Point(xy)) for world_id in self.world_dict.keys()}   #   # self.world_dict == {1703468321573: <src.worldR.World object at 0x000002503F736860>, 1703468321574: <src.worldR.World object at 0x000002503F812D10>}  #  # self.polygons  =={1703468321573: <POLYGON ((-73.397 -196.409, 87.822 11.573, -142.87 238.262, -339.723 25.829...>, 1703468321574: <POLYGON ((-74.053 -196.799, 87.177 10.751, -142.993 238.228, -339.812 25.69...>}  #  world_dis_dict  ==  {1703468321573: 209.56101762897347, 1703468321574: 209.36992961832493}
            world_sort = sorted(world_dis_dict.items(), key=lambda x:x[1])  # [(1703468321574, 209.36992961832493), (1703468321573, 209.56101762897347)]
            # 需要优化  ：  应该使用 匈牙利 选择最近的uav 加入到 距离多边形形心
            # 目前是选择最近的 world 收下 当前无人机
            for world_id in world_sort: 
                if uav_num_dict[world_id[0]] > 0: # 3个uav 2个world # world需要uav 则加入uav
                    self.uav_id_2_world_id[uav_data["order"]] = world_id[0] #{3: 1703468321574}
                    uav_num_dict[world_id[0]] -= 1 # {1703468321573: 2, 1703468321574: 0}
                    self.world_dict[world_id[0]].add_uav_from_scheduler(uav_data, height_difference_i)
                    break
            # 每个世界中加入 调度的无人机
        # 每个world都有一个 初始规划器 和 在线规划器
        
        # 2. 删除无人机 
        # 3. 共享信息记得更新
        # 4. world中uav个数更新 等等
        for world in self.world_dict.values():
            world.uav_dict = world.tasked_uav_dict
            world.SharedInf.update(world.uav_dict, world.target_dict, world.building_dict, world.plain)
            world._plain.uav_num = len(world.tasked_uav_dict)
            
        
        
        return 

    def init_ref_lon_lat(self, data):
        # self.ref_lat = data["landSets"][0]["list"][0]["latitude"]  # 41.702436
        # self.ref_lon = data["landSets"][0]["list"][0]["longitude"]  # 123.4404905
        # self.ref_lat_rad = math.radians(self.ref_lat)  # 0.7278448143022139
        # self.ref_lon_rad = math.radians(self.ref_lon)  # 2.1544429895017814
        first_area_0 = data["mission"]["areas"][0]
        self.ref_lat = first_area_0[0]["lat"]  # 41.7041358
        self.ref_lon = first_area_0[0]["lon"]  # 123.4332115
        self.ref_lat_rad = math.radians(float(self.ref_lat))    # 0.7278744814088395
        self.ref_lon_rad = math.radians(float(self.ref_lon))    # 2.1543159469855286

    def gps_to_xy(self, lat, lon):  # (41.702436,123.4404905)
        lat_rad = math.radians(lat) # 0.7278448143022139
        lon_rad = math.radians(lon) # 2.1544429895017814

        sin_lat = math.sin(lat_rad) # 0.6652620982867568
        cos_lat = math.cos(lat_rad) # 0.7466098985300834
        ref_sin_lat = math.sin(self.ref_lat_rad) # 0.7466098985300834
        ref_cos_lat = math.cos(self.ref_lat_rad) # 0.6652620982867568

        cos_d_lon = math.cos(lon_rad - self.ref_lon_rad) # 1.0

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0) # 0.9999999999999999
        c = math.acos(arg) # 1.4901161193847656e-08

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c)) # 1.0

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * self.CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - self.ref_lon_rad) * self.CONSTANTS_RADIUS_OF_EARTH)

        return x, y  # 0.0 , 0.0


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
            lat = self.ref_lat
            lon = self.ref_lon

        return float(lat), float(lon)
    

    def split_integer(self, a_dict:Dict, n) -> Dict: # {1703468321573: 316656.20072370826, 1703468321574: 184721.47709301207} # n=3
        sum = 0 # sum = 0：初始化一个变量 sum 为 0，用来存储字典 a_dict 中所有值的总和。
        for v in a_dict.values(): # for v in a_dict.values():：遍历字典 a_dict 中的所有值。
            sum += v # == 501377.67781672033 # ++ 316656.20072370826 # + 184721.47709301207
        n2 = n
        
        for k, v in a_dict.items(): # v 是每个区域的小面积
            # a = max(1, math.floor(v*n/sum))：根据当前值 v 在总和 sum 中的比例，计算出应该分配多少给当前键对应的值，并且取最大值为 1。
            a = max(1, math.floor(n*v/sum))  # v*n/sum == 1.8947165863223527 == 316656.20072370826 * 3 / 501377.67781672033  # (184721.47709301207, 3, 501377.67781672033)  # v*n/sum == 1.1052834136776473 == 184721.47709301207 * 3 / 501377.67781672033
            n2 -= a # 3-1==2  # 2-1 == 1
            a_dict[k] = a # {1703468321573: 1, 1703468321574: 184721.47709301207}
        # a_dict=={1703468321573: 1, 1703468321574: 1}
        while n2 > 0: # 1
            for k in a_dict.keys():
                a_dict[k] += 1 # 随便+1
                n2 -= 1
                if n2 == 0:
                    break
        return a_dict  # {1703468321573: 2, 1703468321574: 1}


    def update_one_world_nonlinear_strike_2_json(self, data:str, first_received:bool):
        # 收
        if first_received==True:
            # init uav & enviroment
            try:
                # 这里写可能会发生异常的代码
                self.world.initialization_UDP(data) # load data from GCS # 分两种mission # 第一种 <1>uav, <2>target, <3>no-building, <4>plain
            except Exception as e:
                # 发生异常时执行以下代码
                print()
                print()
                print("-------------------------------------------  self.world.initialization_UDP(data)")
                print("No   area points   received")
                print("No   UAVs          received")
                print("No   xxxx          received")
                # print('len(data["mission"]["points"])')
                # print(len(data["mission"]["points"]))
                print('data')
                print(data)
                # print('  Exception in update() :  data  ')  # IndexError:List index out of range
                print('                 self.world.initialization_UDP(data)  ')  # IndexError:List index out of range
                # import json
                # print(json.dumps(data).encode('utf-8'))  # IndexError:List index out of range
                # print(f"!!!发生异常接收到的数据如上：{e}")  
                import traceback
                # 打印异常信息
                traceback.print_exc()
                print("-------------------------------------------")
                print()
                print()
                return None
                # raise  # 将异常重新抛出
            # self.world.initialization_UDP(data) # load data from GCS
            
            try:
                # 这里写可能会发生异常的代码
                self.world.init_plan()
            except Exception as e:
                print()
                print()
                print("-------------------------------------------  self.world.init_plan()")
                print("The received area is too small")
                print("The received area is Not convex polygons (if code :  assert len(line_p) == 2)")
                # print('  Exception in update() :  data  ')  # IndexError:List index out of range
                print('                 self.world.init_plan()  ')  # self.bound_points[1], self.bound_points[-1], self.bound_points[-2]]  IndexError:List index out of range
                # key_points = [self.bound_points[0], self.bound_points[1], self.bound_points[-1], self.bound_points[-2]]
                import traceback
                # 打印异常信息
                traceback.print_exc()
                print("-------------------------------------------")
                print()
                print()
                return None
            # self.world.init_plan()
            self.world.output_type = 2
            takeoff = True
            changed_uav = {uav_id:() for uav_id in self.world.SharedInf.uav_dict.keys()} # uav_id: (old task, new task)
        elif first_received==False:
            # update current uav & enviroment data
            for uav_data in data["teams"][0]["UAVs"]:
                uav_id = int(uav_data["order"])
                uav:UAV = self.world.uav_dict[uav_id]
                if uav_data["fly_status"] == 0:  # "flystatus":"飞行状态（bool）0已损坏1飞行中",
                    if uav.active == True:  # {'id': 26, 'pos': array([1.11083733e-02, 8.50261079e-03, 3.00000000e+01]), 'SharedInf': <src.robotR.SharedInformation object at 0x000001BE4F601360>, 'active': True, 'init_pos': array([1.11083733e-02, 8.50261079e-03, 3.00000000e+01]), 'yaw': 0, 'pitch': 0, 'roll': 0, 'velocity': 20, 'dt': 0.1, 'scout_range': 5.976508219484396, 'offset': 19.222708411399005, 'start_point': array([-84.72439101,  66.55395562,  30.        ]), 'travel_traj': array([[ 1.11083733e-02,  8.50261079e-03,  3.00000000e+01],   [-8.47243910e+01,  6.65539556e+01,  3.00000000e+01]]), ...}
                        # uav.task.unassign()  # 对task取消其当前的任务分配  :  key_points, assign_id= -1 , cur_pos
                        uav.task.unassign_save_site(uav_data["finished_point_ID"], uav_data["scout_end"],  uav_data)  # 标志终端+断点信息 #赋值 self.assign_id = -1  #  self.finished_point_ID  #  self.finished = True
                        uav.task = None # 不能
                    uav.active = False  # 将其标记为不活跃状态，
                else:  # fly_status == 1  #  判断是否 完成自己的任务，准备帮助其他人
                    # Complete your own tasks and be prepared to help others
                    is_complete = uav.task.complete_own_help_others(uav_data["finished_point_ID"], uav_data["scout_end"],  uav_data)
                if not uav.active:
                    continue # 无人机 不活跃
                x, y = self.world.gps_to_xy(uav_data["lat"], uav_data["lon"])
                # print('x, y')
                # print(x, y)
                uav.update_from_gcs(np.array([x,y,uav_data["alt"]]))  # 无人机还活着 # 更新 自己的位置  self.pos
                uav.update_task_state(uav_data["finished_point_ID"], uav_data["scout_end"])       # 更新完wp_id 判断finished    # 处理两种特殊任务 target 或 back # self.active = False 
                if is_complete == True:
                    print('is_complete == True')
                    print('uav.task.task_id')
                    print(uav.task.task_id)
                    if uav.task.task_id[0] == "back":
                        pass
                    elif uav.task.task_id[0] != "back":
                        uav.task = None  # 任务完成了 #  直接把任务 指针赋值为 None  # 在下面分配新任务

            self.world.output_type, changed_uav = self.world.online_plan()  # 在线规划  # changed_uav == {27: (('plain', 27), ('plain', 26))}  # 2test {10: (('plain', 10), ('plain', 18))}
            takeoff = False  # 只有第一次起飞 first_received==True
        # 发
        return self.world.nonlinear_strike_json_wrapper_UDP_dict(first_received, takeoff, changed_uav, recv_data=data)
