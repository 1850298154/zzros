from src.worldR import World


# 用于收发UDF的world类接口  # 判断 第一次 
class MissionSwitch_JsonDict_Producer(World):
    def __init__(self) -> None:
        args = {"world_config_file":"world_config.yaml",
                "render": False,
                "save_fig": False,
                "attack": 1}
        super().__init__(args)
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
            # self.init_nonlinear_strike(data=data["teams"][0]) # 默认只接受一个任务
            return self.update_nonlinear_strike_2_json(data, first_received)
        elif mission["mission_type"] == 2: 
            print('     mission["mission_type"] != 1    ', mission["mission_type"])
            # self.init_accompany_shield(data=data["teams"][0]) # 默认只接受一个任务
            return self.update_accompany_shield_2_json(data, first_received) # 完成 第一次 或 再次 规划 # 结果 Dict 发送前调用 库wrap json
            
    def update_accompany_shield_2_json(self, data:str, first_received:bool):
        if first_received==True:
            # return self.init_plan_accompany_shield_2_json(data, first_received)  # 包括 要发送的 Dict # 发送前 调用 库 wrap json
            return self.init_more_plan_accompany_shield_2_json(data, first_received)  # 包括 要发送的 Dict # 发送前 调用 库 wrap json
        else:
            return self.init_more_plan_accompany_shield_2_json(data, first_received)  # 包括 要发送的 Dict # 发送前 调用 库 wrap json
            # return self.more_plan_accompany_shield_2_json(data, first_received)
    
    def update_nonlinear_strike_2_json(self, data:str, first_received:bool):
        # 收
        if first_received==True:
            # init uav & enviroment
            try:
                # 这里写可能会发生异常的代码
                self.initialization_UDP(data) # load data from GCS # 分两种mission # 第一种 <1>uav, <2>target, <3>no-building, <4>plain
            except Exception as e:
                # 发生异常时执行以下代码
                print()
                print()
                print("-------------------------------------------  self.initialization_UDP(data)")
                print("No   area points   received")
                print("No   UAVs          received")
                print("No   xxxx          received")
                # print('len(data["mission"]["points"])')
                # print(len(data["mission"]["points"]))
                print('data')
                print(data)
                # print('  Exception in update() :  data  ')  # IndexError:List index out of range
                print('                 self.initialization_UDP(data)  ')  # IndexError:List index out of range
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
            # self.initialization_UDP(data) # load data from GCS
            
            try:
                # 这里写可能会发生异常的代码
                self.init_plan()
            except Exception as e:
                print()
                print()
                print("-------------------------------------------  self.init_plan()")
                print("The received area is too small")
                print("The received area is Not convex polygons (if code :  assert len(line_p) == 2)")
                # print('  Exception in update() :  data  ')  # IndexError:List index out of range
                print('                 self.init_plan()  ')  # self.bound_points[1], self.bound_points[-1], self.bound_points[-2]]  IndexError:List index out of range
                # key_points = [self.bound_points[0], self.bound_points[1], self.bound_points[-1], self.bound_points[-2]]
                import traceback
                # 打印异常信息
                traceback.print_exc()
                print("-------------------------------------------")
                print()
                print()
                return None
            # self.init_plan()
            self.output_type = 2
            takeoff = True
            changed_uav = {uav_id:() for uav_id in self.SharedInf.uav_dict.keys()} # uav_id: (old task, new task)
        elif first_received==False:
            # update current uav & enviroment data
            for uav_data in data["teams"][0]["UAVs"]:
                uav_id = int(uav_data["order"])
                uav = self.uav_dict[uav_id]
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
                x, y = self.gps_to_xy(uav_data["lat"], uav_data["lon"])
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

            self.output_type, changed_uav = self.online_plan()  # 在线规划  # changed_uav == {27: (('plain', 27), ('plain', 26))}  # 2test {10: (('plain', 10), ('plain', 18))}
            takeoff = False  # 只有第一次起飞 first_received==True
        # 发
        return self.nonlinear_strike_json_wrapper_UDP_dict(first_received, takeoff, changed_uav, recv_data=data)
