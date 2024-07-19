import sys
# sys.path.append(".")
# sys.path.append("..")
# sys.path.append("../..")
# sys.path.append("../../..")
from math import floor, ceil, cos, sin, tan
from typing import Literal, Union, Dict, List, Tuple

import numpy as np

import shapely
from shapely.affinity import scale as shapely_scale
from shapely.plotting import plot_polygon, plot_line, plot_points


if __name__ == '__main__':
    from utils import dis_between_points, move_alone_line, norm, point_in_rectangle, isCollisionFreeVertex
    from task.bug_planner import BugPlanner
else:
    # # ImportError: attempted relative import with no known parent package
    # from .utils import dis_between_points, move_alone_line, norm, point_in_rectangle, isCollisionFreeVertex
    from .utils import dis_between_points, move_alone_line, norm, point_in_rectangle, isCollisionFreeVertex
    # from .task.bug_planner import BugPlanner


class SharedInformation(object):
    def __init__(self, world_config:Dict, args) -> None:
        self.world_config = world_config  # self.world_config = {'world': {'timestep': 0.1, 'random_seed': 10}, 'planner': {'search_height': 30}, 'map': {'width': 600, 'length': 600, 'height': 40, 'building': {...}}, 'robots': {'UAV': {...}, 'Target': {...}}}
        self.args:Dict = args  # self.args:Dict = {'world_config_file': 'world_config.yaml', 'render': False, 'save_fig': False, 'attack': 1}
        self.task_dict:Dict[Tuple[str, int], "Task"] = {} # contains all the tasks
        self.back_task_dict:Dict[Tuple[str, int], "Task"] = {} # contains all the back tasks
    
    # def update(self, uav_dict:Dict[int, "UAV"], target_dict:Dict[int, "Target"], building_dict:Dict[int, "Building"], plain:Union["Plain", "PolygonRegion"]): # " " to solve circular dependency
    def update(
            self, 
            uav_dict:Dict[int, "UAV"], 
            target_dict:Dict[int, "Target"], 
            building_dict:Dict[int, "Building"], 
            plain:Union["Plain", "PolygonRegion"],
            multi_plain:Dict[int, "PolygonRegion"] = None,
        ): # " " to solve circular dependency
        self.uav_dict = uav_dict
        self.target_dict = target_dict
        self.building_dict = building_dict
        self.plain = plain
        self.multi_plain = multi_plain

class Robot(object):
    def __init__(self, id:int, pos:np.ndarray, SharedInf:SharedInformation) -> None:
        self.id = id  # 0
        self.pos = pos  # array([327.13206433, 402.07519494,  30.        ])
        self.SharedInf = SharedInf
        self.active = True

    def move(self):
        pass


class UAV(Robot):
    def __init__(self, id: int, pos: np.ndarray, SharedInf: SharedInformation, latlon=tuple(), height_difference_i=-1, yaw=0, pitch=0, roll=0) -> None:
        super().__init__(id, pos, SharedInf)
        # print('id')
        # print(id)
        # print('pos')
        # print(pos)
        self.height_difference_i = height_difference_i
        self.init_latlon = latlon  # array([327.13206433, 402.07519494,  30.        ])
        self.init_pos = pos  # array([327.13206433, 402.07519494,  30.        ])
        self.yaw = yaw  # 0
        self.pitch = pitch  # 0
        self.roll = roll  # 0

        self.velocity = self.SharedInf.world_config['robots']['UAV']['velocity']  # 20
        self.dt = self.SharedInf.world_config['world']['timestep']  # 0.1
        self.fly_high = self.SharedInf.world_config['planner']['search_height']  # 从 地面站 json 修改 后，  在读取

        self.scout_range = self.SharedInf.world_config['robots']['UAV']['scout_range'] # actual scout_range is calculated by the camera parameters  # 后续更新这个值，现在只是随便设置的。实际根据高度等调整
        self.offset = -1.0 # actual scout_range is calculated by the camera parameters
        # self.ideal_scout_range = self.scout_range
        self.start_point:np.ndarray = None 
        self.travel_traj:np.ndarray = None
        self.search_traj:np.ndarray = None  # uav的search_traj这个变量没有用，是因为一些历史原因所以才保留了这个变量。每个uav的轨迹都应该是参考task.keypoints  # 的search_traj是init planner得到的结果，这个是因为之前的一套代码是这样写的。现在这个项目里面涉及的有动态分配，所以都按task来

        self.task:Union[Task, None] = None

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
        self.theta = self.SharedInf.world_config['robots']['UAV']['theta'] # 40
        self._HFOV = self.SharedInf.world_config['robots']['UAV']['HFOV']  # 24.9
        self._VFOV = self.SharedInf.world_config['robots']['UAV']['VFOV']  # 14.7
        self._pt_fov:List[np.ndarray] = []
        self._corner_fov:List[np.ndarray] = []
        self._ground_fov:List[np.ndarray] = []

        self.head_pt = np.array([20, 0, 0, 1])
        self.y_pt = np.array([0, 10, 0, 1])
        self.z_pt = np.array([0, 0, 10, 1])
        self.calc_fov()
        
        # for avoiding building
        self.cur_near_building = None
        self.pre_near_building = None


    def scan(self, height_limit = 35):
        if self.active == False:
            return

        # scan for targets
        for target in self.SharedInf.target_dict.values():
            task_id = ("target", target.id)
            if (norm(target.pos[:2] - self.pos[:2]) < self.scout_range*1.2) and (abs(target.pos[-1] - self.pos[-1]) <= height_limit) and (task_id not in self.SharedInf.task_dict.keys()): # FIXME need to update
                target.discovered = True
                self.SharedInf.task_dict[task_id] = Task(task_id=task_id, init_assign=self.id, key_points=np.reshape(target.pos, (1,3)))
        
        # scan for buildings
        for building in self.SharedInf.building_dict.values():
            # FIXME need to be more precise 
            if (norm(building.center - self.pos[:2]) < self.scout_range + building.width/2) and (not building.discovered):
                building.discovered = True
                idx = 0
                for task_id in self.SharedInf.task_dict.keys():
                    if task_id[0] == "building":
                        idx += 1
                task_id = ("building", idx)
                self.SharedInf.task_dict[task_id] = BuildingTask(task_id=task_id, init_assign=self.id, key_points=building.generate_key_points(self.pos), building_id=building.id)

                # 把building所在位置平面区域全部标记为已探测
                mapwidth, maplength = self.SharedInf.plain.width, self.SharedInf.plain.length
                for i in range(floor(building.anchor[0])-1, ceil(building.anchor[0]+building.width)+1):
                    for j in range(floor(building.anchor[1])-1, ceil(building.anchor[1]+building.length)+1):

                        x_lb=max(min(floor(i), mapwidth),0)
                        y_lb=max(min(floor(j), maplength),0)
                        x_ub=max(min(ceil(i), mapwidth),0)
                        y_ub=max(min(ceil(j), maplength),0)
                        self.SharedInf.plain.uncertainty_map[x_lb,y_lb]=0
                        self.SharedInf.plain.uncertainty_map[x_ub,y_lb]=0
                        self.SharedInf.plain.uncertainty_map[x_lb,y_ub]=0
                        self.SharedInf.plain.uncertainty_map[x_ub,y_ub]=0

        R=self.scout_range


        # mark the searched area of plain
        if self.task.task_id[0] != "building":
            self.theta = self.SharedInf.world_config['robots']['UAV']['theta']
            mapwidth, maplength = self.SharedInf.plain.width, self.SharedInf.plain.length
            
            # x4 ---- x3                                   x4 ---- x3
            # | * * * |                                    | * * * |
            # | * * * |     ---->  then convert to GCS    | * * * |
            # | * * * |                                  | * * * |
            # x1 ---- x2                                x1 ---- x2
            T = self.transformation_matrix()
            for i in range(floor(self._ground_fov[0][0])-1, ceil(self._ground_fov[1][0])+1):
                for j in range(floor(self._ground_fov[0][1])-1, ceil(self._ground_fov[3][1])+1):
                    # if self.SharedInf.plain.is_In_region(self.pos) and self.task.task_id[0] == "plain":
                    if self.task.task_id[0] == "plain":
                        # max(min())形式将x或y限定在矩形框内
                        p = np.matmul(T, np.array([i, j, -self.pos[2], 1])) # 相对坐标转化为绝对坐标
                        x_lb=max(min(floor(p[0]), mapwidth),0)
                        y_lb=max(min(floor(p[1]), maplength),0)
                        x_ub=max(min(ceil(p[0]), mapwidth),0)
                        y_ub=max(min(ceil(p[1]), maplength),0)
                        self.SharedInf.plain.uncertainty_map[x_lb,y_lb]=0
                        self.SharedInf.plain.uncertainty_map[x_ub,y_lb]=0
                        self.SharedInf.plain.uncertainty_map[x_lb,y_ub]=0
                        self.SharedInf.plain.uncertainty_map[x_ub,y_ub]=0
        
        else: # self.task.task_id[0] == "building":
            building = self.SharedInf.building_dict[self.task.building_id]
            x1, y1 = building.anchor
            x2, y2 = x1 + building.width, y1 + building.length
            if self.pos[0] <= x1:
                idx = 3
                center = self.pos[1:]
            elif x1 < self.pos[0] <= x2:
                center = self.pos[[0,2]]
                if self.pos[1] <= y1:
                    idx = 0
                else: #elif self.pos[1] >= y2:
                    idx = 2
            elif self.pos[0] > x2:
                center = self.pos[1:]
                idx = 1
            
            uncertainty_map = building.uncertainty_map[idx]
            bbox = building.surface_bbox[idx]

            floor_height = building.height/building.floors
            R = R/2
            for i in range(floor(-R), ceil(R+1)):
                for j in range(floor(-floor_height)-1, 1):
                    # max(min())形式将x或y限定在矩形框内
                    x_lb=max(min(floor(center[0]+i- bbox[0][0]), bbox[0][1]-bbox[0][0]), 0)
                    y_lb=max(min(floor(center[1]+j- bbox[1][0]), bbox[1][1]-bbox[1][0]), 0)
                    x_ub=max(min(ceil(center[0]+i- bbox[0][0]), bbox[0][1]-bbox[0][0]), 0)
                    y_ub=max(min(ceil(center[1]+j- bbox[1][0]), bbox[1][1]-bbox[1][0]), 0)

                    uncertainty_map[x_lb,y_lb]=0
                    uncertainty_map[x_ub,y_lb]=0
                    uncertainty_map[x_lb,y_ub]=0
                    uncertainty_map[x_ub,y_ub]=0
            
            # adjust uav
            if norm(self.pos[:2] - building.center) <= building.width/2*1.414 + building.safe_margin:
                self.yaw = building.search_angle[idx]
            else:
                direction_vec = self.task.target_pos - self.pos
                self.yaw = np.arctan2(direction_vec[1], direction_vec[0]) * 180 / np.pi
            self.theta = 90
        self.calc_fov()


    def find_nearest_building(self, building_list:List['Building']):
        min_distance = float('inf')
        nearest_building = None
        for building in building_list:
            dist = norm(building.center - self.pos[:2])
            if dist < min_distance:
                min_distance = dist
                nearest_building = building 
        if norm(nearest_building.center - self.pos[:2]) < self.scout_range + nearest_building.width/2 + 2: #2为inflated size of bugplanner
            if self.active:
                nearest_building.discover_iter_plain[self.id] += 1
                return nearest_building
        else:
            return None

    def move(self):
        if self.active == False:
            return
        
        dt = self.SharedInf.world_config['world']['timestep']
        # if self.id ==6 :
        #     self.SharedInf.visualizer.plot_path(self.task.key_points,col='-r')
        
        if self.task.task_id[0] == 'plain' or self.task.task_id[0] == 'target' or self.task.task_id[0] == 'building':
            building_list = []
            nearest_building = None
            for building in self.SharedInf.building_dict.values():
                if building.discovered == True:
                    building_list.append(building)
            if building_list != []:
                nearest_building = self.find_nearest_building(building_list)
            if nearest_building is not None :
                
                # todo
                # 修改新的motion strategy
                self.cur_near_building = nearest_building
                if nearest_building.discover_iter_plain[self.id] == 1 :
                    # 判断有几个goal point在建筑物内
                    self.pre_near_building = nearest_building
                    invalid_points_index = [] 
                    for i in range(self.task.cur_index, self.task.total_index):
                        # if i >= len(self.task.key_points):
                        #     continue
                        if not isCollisionFreeVertex(
                            obstacles=[nearest_building.shape],
                            xy = self.task.key_points[i][:2]):
                            invalid_points_index.append(i)
                    print(f"uav = {self.id}, tasks = {self.task.task_id}, nearest_building = {nearest_building.id}")
                    if self.task.task_id[0] == 'building' and nearest_building.id == self.task.building_id:
                        goal_pos = self.task.target_pos
                    else:
                    #case1：cur_index包括之后都没有
                        if invalid_points_index == []:
                            # decide whether trajectory points are in the building
                            # print(f"uav {self.id} discover building !")
                            
                            # add a reference goal point to improve scan effectiveness
                            ref_dir_vec = np.array(self.task.target_pos[:2]) - np.array(self.pos[:2])
                            ref_dir_vec = ref_dir_vec/max(norm(ref_dir_vec),0.01)
                            intersection_point = self.pos[:2] + ref_dir_vec * (nearest_building.width + 2 ) # 2 for inflated size
                            len_ratio = 0.2
                            ref_goal = intersection_point + ref_dir_vec * len_ratio * norm(np.array(self.task.target_pos[:2]) - np.array(intersection_point))
                            
                            
                            self.bug_planner = BugPlanner(start_point=np.array(self.pos[:2]),
                                                        goal_point=np.array(ref_goal), # self.task.target_pos[:2]
                                                        step_size=self.velocity*dt,
                                                        inflated_size=2,
                                                        obstacle_list=[[np.array([nearest_building.center[0],nearest_building.center[1]]),\
                                                            nearest_building.length,\
                                                           nearest_building.width]])
                            print(f"uav {self.id}, current task: {self.task.task_id}")
                            self.bug_planner.run()
                            # new_path = self.bug_planner.path[1:-1]
                            new_path = self.bug_planner.path
                            new_path = np.array([list(path) for path in new_path])
                            z_vector = np.full((new_path.shape[0],1), self.pos[2])
                            new_path_vector = np.concatenate((new_path,z_vector), axis=1)
                            self.task.key_points = np.insert(self.task.key_points, self.task.cur_index, new_path_vector, axis=0) # 插入探索过程中的点
                            # self.task.total_index += len(self.bug_planner.path[1:-1])
                            self.task.total_index += len(self.bug_planner.path)
                            
                        # case2: 有部分无效点
                        else:
                            # temp_cur_index = self.task.cur_index
                            if self.task.cur_index not in invalid_points_index:
                                invalid_points_index.insert(0,self.task.cur_index)
                            # decide whether trajectory points are in the building
                            print(f"uav {self.id} discover building !")
                            invalid_index_cur = invalid_points_index[-1] + 1
                            # while norm(np.array(self.task.key_points[invalid_index_cur][:2]) - nearest_building.center) <= 2 + nearest_building.width: # 2 for inflated size
                            #     invalid_index_cur += 1
                            
                            # add a reference goal point to improve scan effectiveness
                            ref_dir_vec = np.array(self.task.key_points[invalid_index_cur][:2]) - np.array(self.pos[:2])
                            ref_dir_vec = ref_dir_vec/max(norm(ref_dir_vec),0.01)
                            intersection_point = self.pos[:2] + ref_dir_vec * (nearest_building.width + 2 ) # 2 for inflated size
                            len_ratio = 0.2
                            ref_goal = intersection_point + ref_dir_vec * len_ratio * norm(np.array(self.task.key_points[invalid_index_cur][:2]) - np.array(intersection_point))
                                                  
                                
                            self.bug_planner = BugPlanner(start_point=np.array(self.pos[:2]),
                                                        goal_point=np.array(ref_goal), #self.task.key_points[invalid_index_cur][:2]
                                                        step_size=self.velocity*dt,
                                                        inflated_size=2,
                                                        obstacle_list=[[np.array([nearest_building.center[0],nearest_building.center[1]]),\
                                                            nearest_building.length,\
                                                            nearest_building.width]])
                            
                            self.bug_planner.run()
                            self.task.key_points = np.delete(self.task.key_points,invalid_points_index,axis=0)
                            self.task.total_index -= len(invalid_points_index)
                            new_path = self.bug_planner.path[1:-1]
                            new_path = self.bug_planner.path
                            new_path = np.array([list(path) for path in new_path])
                            z_vector = np.full((new_path.shape[0],1), self.pos[2])
                            new_path_vector = np.concatenate((new_path,z_vector), axis=1)
                            self.task.key_points = np.insert(self.task.key_points,self.task.cur_index,new_path_vector,axis=0)
                            # self.task.total_index += len(self.bug_planner.path[1:-1])
                            self.task.total_index += len(self.bug_planner.path)

                        goal_pos = self.task.target_pos
                        
                        print(f"uav:{self.id} start avoiding !!!")
                        # if self.id ==1 :
                        #     self.SharedInf.visualizer.plot_path(new_path)
                        #     plt.show()
                else:
                    goal_pos = self.task.target_pos      
            else:
                self.cur_near_building = None
                goal_pos = self.task.target_pos
        else:
            goal_pos = self.task.target_pos
        
        # decide whether safe
        if self.pre_near_building is not None:
            if norm(self.pre_near_building.center - self.pos[:2]) >= self.pre_near_building.width/2 :
                print(f"uav:{self.id} is safe !!!!!!!")
            
        
        
        # decide wheter this nearest building sets default 0
        if self.cur_near_building is not self.pre_near_building:
            print(f"uav id ==== {self.id}, near_building ==== {self.pre_near_building.discover_iter_plain[self.id]}") 
            self.pre_near_building.discover_iter_plain[self.id] = 0
            # for corner in self.bug_planner.min_obstacle.corners: corner.visited = False
            print(f"uav id ==== {self.id}, near_building ==== {self.pre_near_building.discover_iter_plain[self.id]}")
            print(f"uav {self.id} finishing avoding!!!!")
            self.pre_near_building = None
        
        
        direction_vec=goal_pos-self.pos
        direction_vec=direction_vec/max(norm(direction_vec),0.01)
        if self.task.task_id[0] != 'building': # building任务的yaw角调整在scan()函数内
            self.calc_yaw(direction_vec)
        self.pos=self.pos+dt*direction_vec*self.velocity

        self.update_task_state() # 用更新之后的坐标来更新任务状态

    def update_from_gcs(self, pos:np.ndarray):
        self.pos = pos
        
    def update_task_state(self, finished_point_ID=0, scout_end=False):
        self.task.update(self.pos, finished_point_ID, scout_end) # 如果完成所有航点 self.finished = True # 更新完成的航点 self.cur_index = finished_point_ID  # 调用 task.update() 方法来更新任务状态。
        if self.task.finished == True:  # 后面两个任务暂无使用
            if self.task.task_id[0] == "target":
                self.active = False # 已经完成suicide # 不可以撤回  # 标记无人机为不活跃状态
                self.SharedInf.target_dict[self.task.task_id[1]].active = False  # 还没有完成打击任务 # 取消任务分配
            if self.task.task_id[0] == "back":
                # self.active = False
                self.active = True # 还是让活着  不让降落
            self.task.assign_id = -1  # 任务完成  无分配的uav
            # self.task = None


    def calc_fov(self):
        #   x4  D  x3
        #       |
        #   A-------B
        #       |
        #   x1  C  x2

        # d = self.pos[2]  # d = 30.0  # array([327.13206433, 402.07519494,  30.        ])
        d = self.fly_high
        alpha = (self.theta - 0.5*self._VFOV)*np.pi/180  # (40 - 0.5*14.7)*np.pi/180 
        theta = self.theta*np.pi/180  # 0.6981317007977318
        if self.theta != 90:

            tan2 = tan(theta)*tan(theta)  # 0.7040881910418472
            self._pt_fov.append(np.array([d*tan(alpha), 0, -d, 1])) # pt_A
            self._pt_fov.append(np.array([d*(tan2*tan(alpha)+2*tan(theta)-tan(alpha))/(1+tan2),
                                          0,
                                          d*(tan2-2*tan(theta)*tan(alpha)-1)/(1+tan2),
                                        #   d*(tan3+2*tan2*tan(alpha)+3*tan(theta))/(tan(theta)+tan3),
                                          1])) #pt_B
            P = (self._pt_fov[0] + self._pt_fov[1])/2
            self._pt_fov.append(np.array([P[0], P[2]*tan(self._HFOV/2*np.pi/180), P[2], 1])) # pt_C
            self._pt_fov.append(np.array([P[0],-P[2]*tan(self._HFOV/2*np.pi/180), P[2], 1])) # pt_D
            # len==4  # self._pt_fov==[[19.222708411399005, 0.0, -30.0, 1.0], [26.206244304904025, 0.0, -24.140117607446633, 1.0], [22.714476358151515, -5.976508219484396, -27.070058803723317, 1.0], [22.714476358151515, 5.976508219484396, -27.070058803723317, 1.0]]
            self._corner_fov.append(np.array([self._pt_fov[0][0], self._pt_fov[2][1], self._pt_fov[0][2], 1])) # x1
            self._corner_fov.append(np.array([self._pt_fov[1][0], self._pt_fov[2][1], self._pt_fov[1][2], 1])) # x2
            self._corner_fov.append(np.array([self._pt_fov[1][0], self._pt_fov[3][1], self._pt_fov[1][2], 1])) # x3
            self._corner_fov.append(np.array([self._pt_fov[0][0], self._pt_fov[3][1], self._pt_fov[0][2], 1])) # x4
            # len==4  # self._corner_fov==[[19.222708411399005, -5.976508219484396, -30.0, 1.0], [26.206244304904025, -5.976508219484396, -24.140117607446633, 1.0], [26.206244304904025, 5.976508219484396, -24.140117607446633, 1.0], [19.222708411399005, 5.976508219484396, -30.0, 1.0]]
            for p in self._corner_fov:
                p_dot = p.copy()
                p_dot[2] = -d
                self._ground_fov.append(p_dot) # pt_A'  # len==4 # self._ground_fov == [[19.222708411399005, -5.976508219484396, -30.0, 1.0], [26.206244304904025, -5.976508219484396, -30.0, 1.0], [26.206244304904025, 5.976508219484396, -30.0, 1.0], [19.222708411399005, 5.976508219484396, -30.0, 1.0]]
        
            self.optical_axis = P  # len==4 # self.optical_axis == array([ 22.71447636,   0.        , -27.0700588 ,   1.        ])
            # self.debug_display()

            self.scout_range = (np.abs(self._corner_fov[0][1] - self._corner_fov[3][1])/2)  # 只是一半的宽度，最后除以2 。  self.scout_range == 5.976508219484396
            self.offset = self._corner_fov[0][0]  # self.offset = -1.0
        else:
            self.optical_axis = np.array([d, 0, 0, 1])

        

    def debug_display(self):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(8,8))
        ax = plt.subplot(1,1,1, projection="3d")
        ax.set_xlim(560, 590)
        ax.set_ylim(480, 520)
        ax.set_zlim(0, 10)
        # ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        # ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        # ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.grid(False)
        ax.set_aspect('equal')

        T = self.transformation_matrix()
        for idx, p in enumerate(self._points):
            p_t = np.matmul(T, p)
            ax.plot([self.pos[0], p_t[0]], [self.pos[1], p_t[1]], [self.pos[2], p_t[2]], color="k")
            ax.scatter(p_t[0], p_t[1], p_t[2], color="#8FAADC", s=6)
            ax.text(p_t[0], p_t[1], p_t[2], s=f"#{idx+1}#")
        
        
        for idx, p in enumerate(self._pt_fov):
            p_t = np.matmul(T, p)
            ax.plot([self.pos[0], p_t[0]], [self.pos[1], p_t[1]], [self.pos[2], p_t[2]], color="k", ls="--")
            ax.text(p_t[0], p_t[1], p_t[2], s=f"{idx+1}")
        
        for i in range(4):
            p1, p2 = self._corner_fov[i], self._corner_fov[(i+1)%4]
            p1, p2 = np.matmul(T, p1), np.matmul(T, p2)
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color="r")
            # ax.text(p1[0], p1[1], p2[2], s=f"{i+1}#")

        for i in range(4):
            p1, p2 = self._ground_fov[i], self._ground_fov[(i+1)%4]
            p1, p2 = np.matmul(T, p1), np.matmul(T, p2)
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color="grey")
            # ax.text(p1[0], p1[1], p2[2], s=f"{i+1}#")
        
        p_t = np.matmul(T, self.optical_axis)
        ax.plot([self.pos[0], p_t[0]], [self.pos[1], p_t[1]], [self.pos[2], p_t[2]], color="darkorange", ls="-")
        ax.scatter(p_t[0], p_t[1], p_t[2], color="r", s=10, marker="*")

        # plot the yaw (now the angle is the globally relative angle between the GCS and UCS)
        p_t = np.matmul(T, self.head_pt)
        ax.plot([self.pos[0], p_t[0]], [self.pos[1], p_t[1]], [self.pos[2], p_t[2]], color="k")
        ax.scatter(p_t[0], p_t[1], p_t[2], color="r", s=10, marker="x")
        
        p_t = np.matmul(T, self.y_pt)
        ax.plot([self.pos[0], p_t[0]], [self.pos[1], p_t[1]], [self.pos[2], p_t[2]], color="k")
        ax.scatter(p_t[0], p_t[1], p_t[2], color="r", s=10, marker="*")
        
        p_t = np.matmul(T, self.z_pt)
        ax.plot([self.pos[0], p_t[0]], [self.pos[1], p_t[1]], [self.pos[2], p_t[2]], color="k")
        ax.scatter(p_t[0], p_t[1], p_t[2], color="r", s=10, marker="^")

        # plt.show()


    def calc_yaw(self, direction:np.ndarray):
        self.yaw = np.arctan2(direction[1], direction[0]) * 180 / np.pi
    

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
            
            
    def is_available_for_target(self):
        if self.task == None:
            return True
        elif self.task.task_id[0] != "target":
            return True
        else:
            return False


class Target(Robot):
    def __init__(self, id: int, pos: np.ndarray, SharedInf: SharedInformation) -> None:
        super().__init__(id, pos, SharedInf)
        self.discovered = False

class Building(object):
    def __init__(self, id:int, anchor:np.ndarray, width:float, length:float, height:float, floors:int) -> None:
        self.id = id
        self.uav_num = 15
        self.anchor = anchor
        self.width = width
        self.length = length
        self.height = height
        self.floors = floors
        self.discover_iter_plain:Dict[int,int] = {key:0 for key in range((self.uav_num))} #{uav_id, num}
        self.discovered = False
        self.center = self.anchor + np.array([width, length])/2

        self.uncertainty_map:List[np.ndarray] = None
        self.surface_bbox:List[np.ndarray] = None
        self.search_angle:List[float] = [90, 180, -90, -180]
        self.safe_margin = 10
        self.uncertainty_init()
        # self.generate_key_points(init_pos=np.array([0,0,0])) # for debug

    def generate_key_points(self, init_pos:np.ndarray) -> np.ndarray:
        #     b7      b6
        #      |       |
        # b8--p4------p3--b5
        #     |       |
        #     |       |
        #     |       |
        # b1--p1------p2--b4
        #     |       |
        #     b2      b3
        #
        x1, y1 = self.anchor
        x2, y2 = x1 + self.width, y1 + self.length
        safe_margin = self.safe_margin

        self.inflated_size = 2 #  inflated size for bugplanner
        self.shape = np.array([[x1-self.inflated_size, y1-self.inflated_size], [x2+self.inflated_size, y1-self.inflated_size], \
                               [x2+self.inflated_size, y2+self.inflated_size],[x1-self.inflated_size,y2+self.inflated_size]]) 

        # anti-clock
        base_points = np.array([[x1-safe_margin, y1], [x1, y1-safe_margin],
                                [x2, y1-safe_margin], [x2+safe_margin, y1],
                                [x2+safe_margin, y2], [x2, y2+safe_margin],
                                [x1, y2+safe_margin], [x1-safe_margin, y2]])
        dis = [norm(p-init_pos[:2]) for p in base_points]
        init_idx = np.argmin(np.array(dis))
        
        floor_height = self.height/self.floors
        pts = []
        # for i in range(self.floors):
        for i in range(self.floors-1, -1, -1):
            p_3d = np.hstack((base_points, (i+1)*floor_height*np.ones((8,1))))
            for j in range(9):
                pts.append(p_3d[(j+init_idx)%8])
        self.key_points = np.array(pts)
        return np.array(pts)

    def uncertainty_init(self):
        self.uncertainty_map = [np.ones([self.width+1, self.height+1])]         # p1---p2
        self.uncertainty_map.append(np.ones([self.length+1, self.height+1]))    # p2---p3
        self.uncertainty_map.append(np.ones([self.width+1, self.height+1]))     # p3---p4
        self.uncertainty_map.append(np.ones([self.length+1, self.height+1]))    # p4---p1

        x1, y1 = self.anchor
        x2, y2 = x1 + self.width, y1 + self.length
        self.surface_bbox = [np.array([[x1, x2], [0, self.height]])]
        self.surface_bbox.append(np.array([[y1, y2], [0, self.height]]))
        self.surface_bbox.append(np.array([[x1, x2], [0, self.height]]))
        self.surface_bbox.append(np.array([[y1, y2], [0, self.height]]))


# rectangle area
class Plain(object):
    def __init__(self, anchor:np.ndarray, width:float, length:float, scout_range:float) -> None:
        self.anchor = anchor  # array([0, 0])
        self.width = width  # 500
        self.length = length  # 500

        # temp
        self.size = [self.width, self.length]  # [500, 500]
        self.scout_range = scout_range  # 5.976508219484396
        self.entrance = "top_right"
        self.x1 = self.anchor[0]            # 0
        self.x2 = self.anchor[0] + width    # 500
        self.y1 = self.anchor[1]            # 0
        self.y2 = self.anchor[1] + length   # 500
        self.uav_num = 15

        self.uncertainty_map=np.ones([self.width+1, self.length+1])  # self._plain.uncertainty_map.shape == (501, 501)
        return 

    def initilize_boundpoint_list(self):
        D = floor(self.scout_range * 1.9)
        bound_point_relate, bound_point = [], []
        if self.size[0] >= self.size[1]:
            for y in range(floor(D/2), ceil(self.size[1]) + 1, D):
                bound_point_relate.append([0, y])
            if ceil(self.size[1] - floor(D/2)) % int(D) > floor(D/2):
                bound_point_relate.append([0, self.size[1] - floor(D/2)])            
            if (self.entrance == 'top_left') | (self.entrance == 'top_right'):
                bound_point_relate = bound_point_relate[::-1]
            
            for i, p in enumerate(bound_point_relate):
                if (self.entrance == 'bottom_right') | (self.entrance == 'top_right'):
                    i += 1

                if i % 2 == 0:
                    bound_point.append(np.array([self.x1, self.y1 + p[1]]))
                    bound_point.append(np.array([self.x2, self.y1 + p[1]]))
                else:
                    bound_point.append(np.array([self.x2, self.y1 + p[1]]))
                    bound_point.append(np.array([self.x1, self.y1 + p[1]]))
                    
        else:
            for x in range(floor(D/2), ceil(self.size[0]) + 1, D):
                bound_point_relate.append([x, 0])
            # if (ceil(self.size[0]) - x) > floor(D/2):
            #     bound_point_relate.append([x+floor(D/2), 0])
            if ceil(self.size[0] - floor(D/2)) % int(D) > floor(D/2):
                bound_point_relate.append([self.size[0] - floor(D/2), 0])
            if (self.entrance == 'bottom_right') | (self.entrance == 'top_right'):
                bound_point_relate = bound_point_relate[::-1]
            
            for i, p in enumerate(bound_point_relate):
                if (self.entrance == 'top_left') | (self.entrance == 'top_right'):
                    i += 1

                if i % 2 == 0:
                    bound_point.append(np.array([self.x1 + p[0], self.y1]))
                    bound_point.append(np.array([self.x1 + p[0], self.y2]))
                else:
                    bound_point.append(np.array([self.x1 + p[0], self.y2]))
                    bound_point.append(np.array([self.x1 + p[0], self.y1]))

        bound_point_rotate = []
        for p in bound_point:
            # bound_point_rotate.append(self.rotate_point(p))
            bound_point_rotate.append(p)
        
        self.bound_point_relate = bound_point_relate
        self.bound_point = bound_point
        self.bound_point_rotate = bound_point_rotate

    def workload_allocate(self):
        self.initilize_boundpoint_list()
        workload = [0]
        for i in range(len(self.bound_point_rotate) - 1):
            workload.append(dis_between_points(self.bound_point_rotate[i], self.bound_point_rotate[i+1]))
                         
        workload_cumsum = np.cumsum(workload)
        average_workload = workload_cumsum[-1] / self.uav_num

        all_traj_points = []
        traj_uavs = {0:[]}  # Trajs of all UAVs
        
        workload_idx = 0
        for i in range(self.uav_num):
            last_workload_idx = workload_idx
            workload_idx = np.where((workload_cumsum - (i + 1) * average_workload) >= -0.01)[0][0]  # 浮点数
            delta_dis = (i + 1) * average_workload - workload_cumsum[workload_idx-1]
            p_split = move_alone_line(self.bound_point_rotate[workload_idx-1], self.bound_point_rotate[workload_idx], delta_dis)
             
            for j in range(last_workload_idx, workload_idx):
                traj_uavs[i].append(self.bound_point_rotate[j])
            traj_uavs[i].append(p_split)
            all_traj_points += traj_uavs[i]
            if i < self.uav_num - 1:
                traj_uavs[i+1]=[p_split]

        search_dis = [average_workload for _ in range(self.uav_num)]
 
        self.traj_uavs = traj_uavs
        self.search_dis = search_dis

    def is_In_region(self, pos:np.ndarray):
        if self.x1 <= pos[0] <= self.x2 and self.y1 <= pos[1] <= self.y2:
            return True


class LineSeg():
    
    def __init__(self, start_pt, end_pt):
        
        self.start_pt = np.array(start_pt)  # [100, 100]
        self.end_pt = np.array(end_pt)  # [200, 100]

        self.x, self.y = start_pt
        self.x2, self.y2 = end_pt
        
        if self.x != self.x2:
            self.m = (self.y2 - self.y) / (self.x2 - self.x)  # m = 0
            self.b = self.y - self.m*self.x  # b = 100
            
            self.minv = min(self.x, self.x2)  # xmin == 100
            self.maxv = max(self.x, self.x2)  # xmax == 200
            
        else:
            self.m = None
            self.b = self.x
            self.minv = min(self.y, self.y2)
            self.maxv = max(self.y, self.y2)
            
    def length(self):
        return np.linalg.norm([self.x2-self.x, self.y2-self.y])
        
        
    # Find intersection (x, y) with line y = mx + b    
    def intersect_w_line(self, m, b):  # 返回  :  线段 self 和 线段 y = mx + b   的交点
        # Parallel lines
        if m == self.m:
            return (None, None)
        
        # Line is vertical but line segment is not
        elif m == None:
            if self.minv <= b <= self.maxv:
                return (b, self.m*b + self.b)
            else:
                return (None, None)
            
        # Line segment is vertical, but line is not
        elif self.m == None:
            y = m*self.b + b
            
            if self.minv <= y <= self.maxv:
                return (self.b, y)
            else:
                return (None, None)
            
        else:
            
            x = (b - self.b) / (self.m - m)
            y = self.m*x + self.b
            
            if self.minv <= x <= self.maxv:
                return (x, y)
            else:
                return (None, None)
            
    # Find intercept range with line y = mx + b
    def intercept_range(self, m):  # 使用 斜率m 求两个端点的 截距跨度
        
        if self.m == m:
            return (self.b, self.b)
        
        # Line is vertical, but segment is not
        elif m == None:
            return sorted([self.x, self.x2])
        
        # Line is not vertical
        else:
            b = self.y - m*self.x
            b2 = self.y2 - m*self.x2
            
            return sorted([b, b2])

    def calc_alpha(self, p):
        """
        alpha represents the ratio of the projection point C of P on the line AB and the length of AB
        """
        return np.dot((self.end_pt-self.start_pt), (p-self.start_pt))/(self.length()*self.length())

    def distance_to_point(self, p):
        alpha = self.calc_alpha(p)
        if alpha <= 0 or alpha >= 1:
            return np.min([np.linalg.norm(p-self.start_pt), np.linalg.norm(p-self.end_pt)])
        else:
            # if self.m == None:
            #     return np.abs(p[0] - self.b)
            # else:
            #     return np.abs(p[1] - self.m*p[0] - self.b)/(np.sqrt(1+self.m*self.m))
            p_alpha = self.start_pt + alpha*(self.end_pt - self.start_pt)
            return np.linalg.norm(p_alpha-p)


class PolygonRegion():
    def __init__(self, vertices, scout_range, uav_num):
        self.vertices = vertices  # len==7  #  [[100, 100], [200, 100], [300, 150], [375, 225], [400, 300], [200, 250], [100, 150]]
        self.scout_range = scout_range  #h=30 scout=5.976508219484396  # h=270 scout=53.788573975359554
        self.uav_num = uav_num  # 2
    
    def update_start_point(self, start_point):
        self.start_point = np.array(start_point)  # array([313.36482349, 474.88038825])

    def initilize_boundpoint_list(self, overlap=0.8):
        D = floor(self.scout_range * 1.9)
        points = self.vertices
        linesegs = [LineSeg(points[i], points[i+1]) if i+1 < len(points) else LineSeg(points[i], points[0]) for i in range(len(points))]
        lengths = [lineseg.length() for lineseg in linesegs]
        longest_seg = [lineseg for lineseg in linesegs if lineseg.length() == max(lengths)]
        m = longest_seg[0].m
        b = longest_seg[0].b

        intercept_ranges = [lineseg.intercept_range(m) for lineseg in linesegs]

        max_intercept = np.max(intercept_ranges)
        min_intercept = np.min(intercept_ranges)

        num_lines = 10

        # spacing = (max_intercept - min_intercept) / (num_lines+1)
        if m is None:
            spacing = D
        else:
            spacing = D/np.cos(np.arctan(m))

        intercepts = np.arange(min_intercept + spacing, max_intercept, spacing)

        line_pts = [[lineseg.intersect_w_line(m, intercept) for lineseg in linesegs if lineseg.intersect_w_line(m, intercept)[0] is not None] for intercept in intercepts]

        refined_pts = []
        # 如果intecpet刚好在顶点处，会被加入两次，需要剔除重复的
        for pts in line_pts:
            if len(pts) == 2:
                refined_pts.append(pts)
            else:
                line_p = []
                for idx, p in enumerate(pts):
                    if idx == 0:
                        line_p.append(p)
                    elif p != line_p[-1]:
                        line_p.append(p)
                assert len(line_p) == 2
                refined_pts.append(line_p)

        bound_points = []

        for idx, pts in enumerate(refined_pts):
            if idx == 0:
                bound_points.append(np.array(pts[0]))
                bound_points.append(np.array(pts[1]))
            else:
                d0 = np.linalg.norm(bound_points[-1] - np.array(pts[0]))
                d1 = np.linalg.norm(bound_points[-1] - np.array(pts[1]))
                if d0 < d1:
                    bound_points.append(np.array(pts[0]))
                    bound_points.append(np.array(pts[1]))
                else:
                    bound_points.append(np.array(pts[1]))
                    bound_points.append(np.array(pts[0]))            
            # if idx % 2 == 0:
            #     bound_points.append(np.array(pts[0]))
            #     bound_points.append(np.array(pts[1]))
            # else:
            #     bound_points.append(np.array(pts[1]))
            #     bound_points.append(np.array(pts[0]))
        
        self.line_pts = line_pts
        self.bound_points = bound_points
        self.adjust_points_order()

    def calc_nearset_seg_idx(self):
        points = self.vertices
        linesegs = [LineSeg(points[i], points[i+1]) if i+1 < len(points) else LineSeg(points[i], points[0]) for i in range(len(points))]
        
        # find the nearest seg
        dist_to_start_point = [lineseg.distance_to_point(self.start_point) for lineseg in linesegs]
        
        # 当最小值不止一个时，还应该再判断选哪个作为nearest seg
        min_d = np.min(dist_to_start_point)
        min_indexes = np.argwhere(dist_to_start_point == min_d).flatten()
        if min_indexes.size == 1:
            nearset_seg_idx = min_indexes[0]
        else:
            # 当最小值不止一个时，还应该再判断选哪个作为nearest seg
            # 此时为点p到相邻的两条线段的最小距离为公共顶点
            alpha_list = [linesegs[idx].calc_alpha(self.start_point) for idx in min_indexes]
            a_list = []
            for alpha in alpha_list:
                if alpha >= 1:
                    a_list.append(alpha-1)
                elif alpha <= 0:
                    a_list.append(-alpha)
                else:
                    a_list.append(alpha)
            nearset_seg_idx = min_indexes[np.argmin(a_list)]

        # print(f"nearset_seg_idx: {nearset_seg_idx}")
        return nearset_seg_idx

    def initilize_boundpoint_list_edge(self):
        """ select enter egde as the nearest edge 返回蛇形走位的边界点
        距离大概为15
        In [6]: 15/(2**(1/2))
        Out[6]: 10.606601717798211
        扫描宽度是 scout_range 的两倍
        """
        # self.test_find_nearest_edge()
        # zyt remove
        # print('self.scout_range  | only half')
        # print(self.scout_range)
        # D = floor(self.scout_range * 1.9)  # D==CD, 扫描宽度  # self.scout_range == 5.976508219484396  # D==11
        D =         (self.scout_range * 2)  # D==CD, 扫描宽度  # self.scout_range == 5.976508219484396  # D==11
        points = self.vertices  # [[100, 100], [200, 100], [300, 150], [375, 225], [400, 300], [200, 250], [100, 150]]
        linesegs = [LineSeg(points[i], points[i+1]) if i+1 < len(points) else LineSeg(points[i], points[0]) for i in range(len(points))]
        
        # choose the nearest seg
        nearset_seg_idx = self.calc_nearset_seg_idx()  # 4 # 返回起点到 最近的多边形区域的 边的下标 # 作为最近的入口边
        
        # choose the reference seg
        idx = [(nearset_seg_idx-1)%len(linesegs), (nearset_seg_idx+1)%len(linesegs)]  # [3,5] # 前后两线段
        #   method1: select the longest seg
        # lengths = [linesegs[idx[0]].length(), linesegs[idx[1]].length()]
        # longest_seg = linesegs[idx[np.argmax(lengths)]]
        # m = longest_seg.m

        #   method2: select the seg, that has the shortest intercept range 
        intercept_ranges0 = [lineseg.intercept_range(linesegs[idx[0]].m) for lineseg in linesegs] # len==7 # [[-500.0, -200.0], [-750.0, -500.0], [-900.0, -750.0], (-900.0, -900.0), [-900.0, -350.0], [-350.0, -150.0], [-200.0, -150.0]]
        intercept_ranges1 = [lineseg.intercept_range(linesegs[idx[1]].m) for lineseg in linesegs] # len==7 # [[-100.0, 0.0], [-150.0, -100.0], (-150.0, -150.0), [-150.0, -100.0], [-100.0, 50.0], (50.0, 50.0), [0.0, 50.0]]
        
        intercept_ranges = [np.max(intercept_ranges0)-np.min(intercept_ranges0), np.max(intercept_ranges1)-np.min(intercept_ranges1)]  #  [750.0, 200.0]
        selected_seg = linesegs[idx[np.argmin(intercept_ranges)]]  # 选择纵截距 [跨度] 最小的  作为平行边
        m = selected_seg.m  # 纵截距 [跨度] 最小的 斜率

        
        intercept_ranges = [lineseg.intercept_range(m) for lineseg in linesegs]  # 多边形区域每个边 的 使用 最近跨度小边斜率 m 计算两端点 的 跨度  # [[-100.0, 0.0], [-150.0, -100.0], (-150.0, -150.0), [-150.0, -100.0], [-100.0, 50.0], (50.0, 50.0), [0.0, 50.0]]
        max_intercept = np.max(intercept_ranges)  # 50
        min_intercept = np.min(intercept_ranges)  # -150

        # print('m')
        # print(m)
        # print('D')
        # print(D)
        # spacing = (max_intercept - min_intercept) / (num_lines+1)
        if m is None:
            spacing = D # 扫描宽度是D==11 # spacing是截距宽度
        else:
            spacing = D/np.cos(np.arctan(m))  # 截距宽度(斜边|y轴线) spacing==15.556349186104045
        # print('spacing')
        # print(spacing)
        # zyt remove
        # print('min_intercept + spacing, max_intercept, spacing')
        # print(min_intercept + spacing, max_intercept, spacing, sep='\n')
        intercepts = np.arange(min_intercept + spacing, max_intercept, spacing)  # shape==(12,) 个分割线段  # [-134.44365081389594, -118.8873016277919, -103.33095244168786, -87.77460325558381, -72.21825406947977, -56.661904883375726, -41.10555569727168, -25.54920651116764, -9.992857325063596, 5.563491861040461, 21.11984104714449, 36.67619023324852]

        line_pts = [
            [
                lineseg.intersect_w_line(m, intercept) 
                for lineseg in linesegs 
                if lineseg.intersect_w_line(m, intercept)[0] is not None
            ] for intercept in intercepts
        ]  # len==12 个 交点对 == 边界点对  # [[(268.8873016277919, 134.44365081389594), (382.77817459305203, 248.33452377915614)], [(237.7746032555838, 118.8873016277919), (390.55634918610406, 271.6690475583123)], [(206.6619048833757, 103.33095244168786), (398.3345237791561, 295.0035713374682)], [(187.77460325558383, 100.0), (383.6994710074451, 295.92486775186126)], [(172.21825406947977, 100.0), (362.9576720926397, 290.73941802315994)], [(156.6619048833757, 100.0), (342.2158731778343, 285.55396829445857)], [(141.10555569727168, 100.0), (321.4740742630289, 280.3685185657572)], [(125.54920651116764, 100.0), (300.73227534822354, 275.1830688370559)], [(109.9928573250636, 100.0), (279.9904764334181, 269.9976191083545)], [(259.24867751861274, 264.8121693796532), (100, 105.56349186104046)], [(238.50687860380734, 259.6267196509518), (100, 121.11984104714449)], [(217.76507968900196, 254.44126992225048), (100, 136.67619023324852)]]

        refined_pts = []  # len==12 提纯的 line_pts # [[(268.8873016277919, 134.44365081389594), (382.77817459305203, 248.33452377915614)], [(237.7746032555838, 118.8873016277919), (390.55634918610406, 271.6690475583123)], [(206.6619048833757, 103.33095244168786), (398.3345237791561, 295.0035713374682)], [(187.77460325558383, 100.0), (383.6994710074451, 295.92486775186126)], [(172.21825406947977, 100.0), (362.9576720926397, 290.73941802315994)], [(156.6619048833757, 100.0), (342.2158731778343, 285.55396829445857)], [(141.10555569727168, 100.0), (321.4740742630289, 280.3685185657572)], [(125.54920651116764, 100.0), (300.73227534822354, 275.1830688370559)], [(109.9928573250636, 100.0), (279.9904764334181, 269.9976191083545)], [(259.24867751861274, 264.8121693796532), (100, 105.56349186104046)], [(238.50687860380734, 259.6267196509518), (100, 121.11984104714449)], [(217.76507968900196, 254.44126992225048), (100, 136.67619023324852)]]
        # 如果intecpet刚好在顶点处，会被加入两次，需要剔除重复的
        for pts in line_pts:
            if len(pts) == 2:
                refined_pts.append(pts)
            else:
                line_p = []
                for idx, p in enumerate(pts):
                    if idx == 0:
                        line_p.append(p)
                    elif p != line_p[-1]:
                        line_p.append(p)
                assert len(line_p) == 2
                refined_pts.append(line_p)

        bound_points = []  # len=24 将边界点对 直接 按顺序变成 点 list  # [[268.8873016277919, 134.44365081389594], [382.77817459305203, 248.33452377915614], [390.55634918610406, 271.6690475583123], [237.7746032555838, 118.8873016277919], [206.6619048833757, 103.33095244168786], [398.3345237791561, 295.0035713374682], [383.6994710074451, 295.92486775186126], [187.77460325558383, 100.0], [172.21825406947977, 100.0], [362.9576720926397, 290.73941802315994], [342.2158731778343, 285.55396829445857], [156.6619048833757, 100.0], [141.10555569727168, 100.0], [321.4740742630289, 280.3685185657572], [300.73227534822354, 275.1830688370559], [125.54920651116764, 100.0], [109.9928573250636, 100.0], [279.9904764334181, 269.9976191083545], [259.24867751861274, 264.8121693796532], [100.0, 105.56349186104046], [100.0, 121.11984104714449], [238.50687860380734, 259.6267196509518], [217.76507968900196, 254.44126992225048], [100.0, 136.67619023324852]]


        for idx, pts in enumerate(refined_pts):
            # if idx == 0:
            #     bound_points.append(np.array(pts[0]))
            #     bound_points.append(np.array(pts[1]))
            # else:
            #     d0 = np.linalg.norm(bound_points[-1] - np.array(pts[0]))
            #     d1 = np.linalg.norm(bound_points[-1] - np.array(pts[1]))
            #     if d0 < d1:
            #         bound_points.append(np.array(pts[0]))
            #         bound_points.append(np.array(pts[1]))
            #     else:
            #         bound_points.append(np.array(pts[1]))
            #         bound_points.append(np.array(pts[0]))            
            if idx % 2 == 0:
                bound_points.append(np.array(pts[0]))
                bound_points.append(np.array(pts[1]))
            else:
                bound_points.append(np.array(pts[1]))
                bound_points.append(np.array(pts[0]))
        
        self.line_pts = line_pts            # 没有用 提纯的 与边界 交点对
        self.bound_points = bound_points    # 将边界点对 直接 按顺序变成 点 lis
        self.adjust_points_order()          # 改成蛇形走位 self.bound_points 

    def adjust_points_order(self):
        """
        ＿＿＿＿＿＿＿
        |1  3      -1|
        |            |
        |0  2      -2|
        ￣￣￣￣￣￣￣  
        [0 1   -1 -2]
        """
        if len(self.bound_points) < 4:
            print('len(self.bound_points)')
            print(len(self.bound_points))
            print(self.bound_points)
        key_points = [self.bound_points[0], self.bound_points[1], self.bound_points[-1], self.bound_points[-2]]  # len==4  # [[268.8873016277919, 134.44365081389594], [382.77817459305203, 248.33452377915614], [100.0, 136.67619023324852], [217.76507968900196, 254.44126992225048]]
        pos = self.start_point  # array([313.36482349, 474.88038825])
        dis_list = [np.linalg.norm(p - pos) for p in key_points]  # len==4 # [343.3299027903115, 236.9414316272741, 399.8832672953519, 240.276332388641]
        min_idx = np.argmin(np.array(dis_list))  # 下标 为 1  # 作为 入口
        if min_idx == 0:
            return
        elif min_idx == 1:
            flip = True  # 对称翻转， 因为是第二个， 顺序从第二个到第一个，对称
            reverse = False
        elif min_idx == 2:
            flip = False  # 不用翻转， 从蛇尾巴开始
            reverse = True  # 蛇尾逆序
        elif min_idx == 3:
            flip = True
            reverse = True
        
        
        if flip:  # 对称反转
            bound_points = []
            for idx in range(len(self.bound_points)):
                if idx % 2 == 0:
                    bound_points.append(self.bound_points[idx+1])
                    bound_points.append(self.bound_points[idx])
            self.bound_points = bound_points
        if reverse:  # 蛇尾开始逆序
            bound_points = self.bound_points[::-1]
        
        self.bound_points = bound_points  # len==24  # [[382.77817459305203, 248.33452377915614], [268.8873016277919, 134.44365081389594], [237.7746032555838, 118.8873016277919], [390.55634918610406, 271.6690475583123], [398.3345237791561, 295.0035713374682], [206.6619048833757, 103.33095244168786], [187.77460325558383, 100.0], [383.6994710074451, 295.92486775186126], [362.9576720926397, 290.73941802315994], [172.21825406947977, 100.0], [156.6619048833757, 100.0], [342.2158731778343, 285.55396829445857], [321.4740742630289, 280.3685185657572], [141.10555569727168, 100.0], [125.54920651116764, 100.0], [300.73227534822354, 275.1830688370559], [279.9904764334181, 269.9976191083545], [109.9928573250636, 100.0], [100.0, 105.56349186104046], [259.24867751861274, 264.8121693796532], [238.50687860380734, 259.6267196509518], [100.0, 121.11984104714449], [100.0, 136.67619023324852], [217.76507968900196, 254.44126992225048]]   
    
    def workload_allocate(self):
        # self.initilize_boundpoint_list()
        self.initilize_boundpoint_list_edge() # change the enter egde as the nearest
        workload = [0]
        for i in range(len(self.bound_points) - 1):
            workload.append(np.linalg.norm(self.bound_points[i] - self.bound_points[i+1]))
                         
        workload_cumsum = np.cumsum(workload)
        average_workload = workload_cumsum[-1] / self.uav_num

        all_traj_points = [] # FIXME necessary?
        traj_uavs = {0:[]}  # Trajs of all UAVs
        
        workload_idx = 0
        for i in range(self.uav_num):
            last_workload_idx = workload_idx
            workload_idx = np.where((workload_cumsum - (i + 1) * average_workload) >= -0.01)[0][0]  # 浮点数
            delta_dis = (i + 1) * average_workload - workload_cumsum[workload_idx-1]
            p_split = move_alone_line(self.bound_points[workload_idx-1], self.bound_points[workload_idx], delta_dis)
             
            for j in range(last_workload_idx, workload_idx):
                traj_uavs[i].append(self.bound_points[j])
            traj_uavs[i].append(p_split)
            all_traj_points += traj_uavs[i]
            if i < self.uav_num - 1:
                traj_uavs[i+1]=[p_split]

        search_dis = [average_workload for _ in range(self.uav_num)]
 
        self.traj_uavs = traj_uavs
        self.search_dis = search_dis

    def test_find_nearest_edge(self):
        """
        plot for test
        """
        import matplotlib.pyplot as plt
        from matplotlib import patches, cm
        plt.figure(figsize=(8,8))
        ax = plt.subplot(1,1,1)
        polygon = patches.Polygon(self.vertices, closed = True, fill = False)
        ax.add_artist(polygon)

        rb = cm.get_cmap('rainbow', len(self.vertices))
        p_list, c_list = [], []
        count = 0
        while count < 10000:
            p = [600*np.random.rand(), 600*np.random.rand()]
            self.start_point = np.array(p)
            idx_ = self.calc_nearset_seg_idx()
            
            c_list.append(rb(idx_))
            p_list.append(p)
            count += 1

        ax.scatter(x=np.array(p_list)[:,0], y=np.array(p_list)[:,1], marker=".", color=c_list)
        # plt.show()

    def get_workload(self):
        rec = shapely.Polygon(self.vertices)
        return rec.area

class Task():
    def __init__(self, task_id:Tuple[str, int], init_assign:int, key_points:np.ndarray) -> None:  # <1>task_id,类型元组uav_id <2>init_assign,分配的uav_id <3>key_points,扫描点+返航起点 <4>cur_index,当前要去的航点，还没有完成 <5>total_index,扫描点+返航点一共有多少 <6>finished,完成任务 <7>assign_id,未分配
        self.task_id = task_id  # ('plain', 0) # ('plain', 1)
        self.init_assign = init_assign  # 传入的是 uav_id = 0  # uav_id = 1
        self.key_points = key_points  # <3>key_points,扫描点+返航起点 # (14*3) # [[382.77817459305203, 248.33452377915614, 30.0], [268.8873016277919, 134.44365081389594, 30.0], [237.7746032555838, 118.8873016277919, 30.0], [390.55634918610406, 271.6690475583123, 30.0], [398.3345237791561, 295.0035713374682, 30.0], [206.6619048833757, 103.33095244168786, 30.0], [187.77460325558383, 100.0, 30.0], [383.6994710074451, 295.92486775186126, 30.0], [362.9576720926397, 290.73941802315994, 30.0], [172.21825406947977, 100.0, 30.0], [156.6619048833757, 100.0, 30.0], [342.2158731778343, 285.55396829445857, 30.0], [321.4740742630289, 280.3685185657572, 30.0], [327.1320643266746, 402.07519493594015, 30.0]] #  (10, 3) N*3 # [[-1.862655543808662, 36.93499701266063, 30.0], [-1.862655543808662, 36.93499701266063, 30.0], [-2.714981840942367, 53.83595830047633, 30.0], [-104.21397945264731, 161.0538831460993, 30.0], [-89.5578615685133, 161.57258212763242, 30.0], [-3.567308138076072, 70.73691958829203, 30.0], [-4.419634435209777, 87.63788087610772, 30.0], [-74.90174368437928, 162.09128110916552, 30.0], [-60.24562580024528, 162.6099800906986, 30.0], [-5.271960732343482, 104.5388421639234, 30.0]]  ########   ######### (13, 3) #  [[321.4740742630289, 280.3685185657572, 30.0], [141.10555569727168, 100.0, 30.0], [125.54920651116764, 100.0, 30.0], [300.73227534822354, 275.1830688370559, 30.0], [279.9904764334181, 269.9976191083545, 30.0], [109.9928573250636, 100.0, 30.0], [100.0, 105.56349186104046, 30.0], [259.24867751861274, 264.8121693796532, 30.0], [238.50687860380734, 259.6267196509518, 30.0], [100.0, 121.11984104714449, 30.0], [100.0, 136.67619023324852, 30.0], [217.76507968900196, 254.44126992225048, 30.0], [313.3648234926275, 474.88038825386116, 30.0]]  
        
        self.cur_index:int = 0
        self.total_index = self.key_points.shape[0]  # 14
        self.finished:bool = False
        self.assign_id:int = -1 # -1 for unassigned

        self.finished_point_ID = -1 
        # self.changed = False 

        
    @property
    def target_pos(self) -> np.ndarray:
        return self.key_points[self.cur_index]

    def update(self, uav_pos:np.ndarray, finished_point_ID=0, scout_end=False,eps=3):  # 该算法要求每个点都必经过 # 不适合zytJQ算法 # 事件触发, 一次更新多个点
        self.cur_pos = uav_pos  # 2024-3-1可以作为断点
        # print('self.cur_pos') # zyt!!!!
        # print(self.cur_pos)
        # print('self.target_pos')
        # print(self.target_pos)
        # if norm(self.cur_pos - self.target_pos) < eps:
        #     self.cur_index += 1  # 进入下一个牵引点  # 走过一个就加一  #  走过0...n-1后，变为n，总数量n，也就是完成
        #  使用 finished_point_ID
        if finished_point_ID != 0:
            self.cur_index = finished_point_ID  # self.cur_index 要飞往的下一个点  # finished_point_ID 刚好是 飞过的 key + 1 == self.cur_index # back
        while self.cur_index < self.total_index and norm(self.cur_pos - self.target_pos) < eps:
            self.cur_index += 1  # 进入下一个牵引点  # 走过一个就加一  #  走过0...n-1后，变为n，总数量n，也就是完成
        
        if self.cur_index == self.total_index or scout_end==True:  # True==1
            self.finished = True
            self.assign_id = -1

    # let the task unassigned, save the current execute state
    def unassign(self):  # 取消其当前的任务分配
        if self.cur_index == 0:  # 如果当前无人机任务执行到了第一个任务点，则清空该任务的分配信息。
            self.assign_id = -1  # uav id==-1 没有无人机 执行该任务
            self.cur_pos = None  
            return
        else:  # 否则，在当前位置插入探索过程中的点，更新总任务索引，并重置该任务的分配信息。
            self.key_points = np.insert(self.key_points, self.cur_index, self.cur_pos, axis=0) # 插入探索过程中的点
            self.total_index += 1  # 任务点 牵引点 +1
            
            self.assign_id = -1  # uav id==-1 没有无人机 执行该任务
            self.cur_pos = None
            
    def complete_own_help_others(self, finished_point_ID, scout_end, uav_data):
        # 应该记录下 已经完成的航点 和 离开的位置
        self.finished_point_ID = finished_point_ID
        # if self.total_index == finished_point_ID:
        # if len(self.key_points) == finished_point_ID: 
        # if len(self.key_points) <= finished_point_ID: 
        self.uav_data = uav_data
        if len(self.key_points) <= finished_point_ID or scout_end == 1: # if len(self.key_points)-2 <= finished_point_ID or scout_end == 1: 
            # uav起飞为 0   # 长度刚好是最后一个 id
            self.finished = True
            self.assign_id = -1
            return True
        return False
    def unassign_save_site(self, finished_point_ID, scout_end, uav_data):
        self.assign_id = -1
        # 应该记录下 已经完成的航点 和 离开的位置
        self.finished_point_ID = finished_point_ID
        # if self.total_index == finished_point_ID:
        # if len(self.key_points) == finished_point_ID: 
        # if len(self.key_points) <= finished_point_ID: 
        if len(self.key_points) <= finished_point_ID or scout_end == 1: # 只有一个key len==1 finished_id==1  # if len(self.key_points)-2 <= finished_point_ID or scout_end == 1: 
            # uav起飞为 0   # 长度刚好是最后一个 id
            self.finished = True  #  ADD  self.cur_index = finished_point_ID
        self.uav_data = uav_data
        return
        """
                {
                    "fly_status": 0,

                    "order": 26,
                    "task_status": 6,
                    "scout_end": 0,
                    
                    "finished_point_ID": 21,
                    "lon": 116.1648486999,
                    "lat": 40.0553102999,
                    "alt": 30
                }
        """

    @property
    def total_distance(self) -> float:
        return self.search_distance + np.linalg.norm(self.cur_pos-self.target_pos)  # 380.35746583790706  # (array([-31.18409377,  10.01451892,  -0.268     ]), array([-26.95456424, -24.9331317 ,  15.        ])) == array([ -4.22952952,  34.94765062, -15.268     ]) == 38.371070195818035

    @property
    def search_distance(self) -> float:
        if self.finished == True:
            return 0
        dis = 0   # 剩余需要搜索的路径长度
        for i in range(self.cur_index, self.total_index-1):
            dis += np.linalg.norm(self.key_points[i] - self.key_points[i+1])
        return dis  # 380.35746583790706

    def split(self, s_p:np.ndarray) -> int:  # array([26.50384003, 12.99502138, 14.962     ])
        if self.cur_index > 0: # 任务已在执行
            tmp_key_points = np.insert(self.key_points, self.cur_index, self.cur_pos, axis=0) # 插入探索过程中的点
            dis_list = np.array([np.linalg.norm(tmp_key_points[i] - tmp_key_points[i+1]) for i in range(self.cur_index, self.total_index)])
            dis_total = np.zeros((0,2))
            
            for i in range(self.cur_index+1, self.total_index):
                travel_dis1 = np.linalg.norm(s_p - tmp_key_points[i])
                travel_dis2 = 0
                dis_total = np.vstack((dis_total, np.array([travel_dis1+dis_list[i-self.cur_index:].sum(), travel_dis2+dis_list[:i-self.cur_index].sum()])))
        else: # 任务没有执行
            dis_list = np.array([np.linalg.norm(self.key_points[i] - self.key_points[i+1]) for i in range(self.cur_index, self.total_index-1)])  # [5.00040423168109, 70.44877131978564, 5.555928791730057, 72.80765748206602, 5.0004042316811015, 75.1665436443464, 5.964423138159396, 34.95342284670212, 105.45991015175528]
            dis_total = np.zeros((0,2))  # array([], shape=(0, 2), dtype=float64)  #  len==8  # [[438.32600380241604, 45.81706387432556], [338.3979032591961, 123.56143364378516], [329.78415304944264, 133.55208613427746], [287.2393184317063, 197.5125154373256], [280.3030679999322, 205.75297125559104], [174.5268304886082, 291.15765913496637], [162.7604105723162, 297.71009839869004], [129.08549744460066, 315.6768886406315]]
            
            for i in range(self.cur_index+1, self.total_index-1): # 从当前位置开始记录  # range(0+1==1, 10-1==9)
                travel_dis1 = np.linalg.norm(s_p - self.key_points[i])  # 62.96894219619005
                travel_dis2 = np.linalg.norm(self.cur_pos - self.key_points[i])  # 40.816659642644474
                dis_total = np.vstack((dis_total, np.array([travel_dis1+dis_list[i-self.cur_index:].sum(), travel_dis2+dis_list[:i-self.cur_index].sum()])))

        dis_ave = np.average(dis_total, axis=1)   # len==8  # array([242.07153384, 230.97966845, 231.66811959, 242.37591693,       243.02801963, 232.84224481, 230.23525449, 222.38119304])
        best_split = np.argmin(dis_ave)  # best_split==7  # best_split==0 对应 key中的 self.cur_index

        if self.cur_index > 0:
            return best_split + self.cur_index, dis_ave[best_split]
        else:
            return best_split + 1, dis_ave[best_split]  # return (8, 222.38119304261608) # (3, 5335336.849062677)
    
    def split_task(self, s_p:np.ndarray) -> "Task":
        split_indx = self.split(s_p)[0]  # split_distance = task.split_dis(uav.pos) 刚刚调用过几次（task未完成数量） ， 用的是 self.split(s_p)[1] 。 这次是用的最优的
        if self.cur_index > 0: # cur==0
            key_points_1 = np.vstack((self.cur_pos, self.key_points[self.cur_index:split_indx+1]))

        else:
            key_points_1 = self.key_points[self.cur_index:split_indx+1] # len==4 # [[-40.19614839603034, 8.483362719557249, 15.0], [-35.814135917094724, 19.08752510076701, 15.0], [-26.844807127671295, 27.700631132992203, 15.0], [-40.83309989776854, -6.150054752330448, 15.0]] # [self.cur_index:split_indx+1] (0, 4)

        key_points_2 = self.key_points[split_indx:] # len==10 # [[-40.83309989776854, -6.150054752330448, 15.0], [-40.890394331519055, -19.380742838141458, 15.0], [-17.875478338247884, 36.313737165217375, 15.0], [-12.042581261739784, 37.336898773229386, 15.0], [-36.2451176357345, -21.23153912617944, 15.0], [-31.59984093994994, -23.08233541421743, 15.0], [-6.496418898860589, 37.666182516158464, 15.0], [-0.9502565359813964, 37.995466259087536, 15.0], [-26.954564244165383, -24.933131702255412, 15.0], [-5227345.1305393465, -1067165.0000035905, 15.0]]
        self.key_points = key_points_1 # 当前任务缩小为key_points_1 # 把 key_points_2 作为新任务
        self.cur_index = 0
        self.total_index = self.key_points.shape[0] # 4 # self.key_points.shape = (4, 3)
        if self.task_id[0] == "plain":#('plain', 17)
            return Task(task_id=(self.task_id[0], -1), init_assign=-1, key_points=key_points_2) # 当前任务缩小为key_points_1 # 把 key_points_2 作为新任务
        elif self.task_id[0] == "building":
            return BuildingTask(task_id=(self.task_id[0], -1), init_assign=-1, key_points=key_points_2, building_id=self.building_id)

    def split_dis(self, s_p:np.ndarray) -> float: # uav.pos==s_p==("name 's_p' is not defined")
        return self.split(s_p)[1]  # self.split(s_p) == (8, 222.38119304261608)

    def change_seq(self, pos:np.ndarray): # uav.pos == array([26.50384003, 12.99502138, 14.962     ])
        d1 = np.linalg.norm(self.target_pos - pos) # self.target_pos == array([-40.8330999 ,  -6.15005475,  15.        ])
        d2 = np.linalg.norm(self.key_points[-1] - pos)
        if d1 < d2: # (70.00570593338385, 5335192.880953756)
            pass
        else:
            self.key_points = self.key_points[::-1]

# class DefenseTask(Task):
class DefenseTask:
    # def __init__(self, task_type, task_ID, region, size, finish=False):  # 参数包括 task_type（任务类型）、task_ID（任务ID）、region（区域坐标）、size（区域大小）和finish（是否完成）等信息。  # (task_type='defense', , task_ID=0, , region=array([ 64, 194]), , size=array([82, 34]))
    def __init__(self, task_type, task_ID, arg=[20, 20, 5, 5], uav_height=20):  # 参数包括 task_type（任务类型）、task_ID（任务ID）、region（区域坐标）、size（区域大小）和finish（是否完成）等信息。  # (task_type='defense', , task_ID=0, , region=array([ 64, 194]), , size=array([82, 34]))
        # super().__init__(task_type, task_ID)
        self.task_type=task_type        # 'defense'
        self.task_ID = task_ID          # 0
        # self.region = region            # array([ 64, 194])
        # self.virtual_region = region    # array([ 64, 194])
        # self.finish = finish            # False
        # self.bottom_center = self.region  # array([ 64, 194])
        # self.size = size                # array([ 64, 194])

        # 双C保护圈参数
        # self.R1, self.R2 = 20, 20
        # self.D1, self.D2 = 5, 5
        self.R1, self.R2 = arg[0:2]
        self.D1, self.D2 = arg[2:4]
        self.theta1, self.theta2 = 20., 20.
        # self.uav_height = 2
        # self.uav_height = 20
        self.uav_height = uav_height
        self.protection_points:List[np.ndarray] = []

        # 双C保护圈参数 —— 方向向量: 从上一次获取
        self.last_pos = None 
        
        # 周围的 保护点 
        self.protection_points = None
        self.last_protection_points = self.protection_points


        # self.calculate_region_limit()

    # def calculate_region_limit(self):  # 用于计算区域的限制范围，根据给定的 region 和 size 计算出区域的四个边界坐标。
    #     self.x1 = self.virtual_region[0]                #64
    #     self.x2 = self.virtual_region[0] + self.size[0] #146
    #     self.y1 = self.virtual_region[1]                #194
    #     self.y2 = self.virtual_region[1] + self.size[1] #228

    # def is_In_region(self, position):  # 判断给定位置是否在当前区域内，如果在区域内则返回 True。
    #     if self.x1 < position[0] <self.x2 and self.y1< position[1] < self.y2:
    #         return True
    
    
    def init_more_plan(self, cur_gcs, cur_uavs, data, first_received):
        """
            self.cur_uavs[uav_id] = {
                'x':x,
                'y':y,
                '2Dpt': np.array([x,y]),
                'uavROD': uav_read_only,
            }
        """
        cur_uavs_list         = list(cur_uavs.values())
        uav_num = len(cur_uavs_list)
        
        gcs = np.array(cur_gcs['2Dpt'])
        last = np.array(self.last_pos) 

        # protection_point_list = self.first_protection_points(cur_gcs['2Dpt'], uav_num)
        if first_received == True:
            vec=[1, 0, 0]
            # protection_point_list = self.update(cur_gcs['2Dpt'], vec=vec, uav_num=uav_num, debug_flag=False)
            protection_point_list = self.update(gcs, vec=vec, uav_num=uav_num, debug_flag=False)
        else:
            # vec = np.array(cur_gcs['2Dpt']) - np.array(self.last_pos) 
            vec = gcs - last  # array([0, 0]) # array([0, 0])
            length = np.linalg.norm(vec)
            if length <= 1e-5:
                vec=[1, 0, 0]
            else:
                vec = vec / length
            # protection_point_list = self.update(cur_gcs['2Dpt'], vec=vec, uav_num=uav_num, debug_flag=False)
            protection_point_list = self.update(gcs, vec=vec, uav_num=uav_num, debug_flag=False)

        # 代价矩阵
        N, M = len(cur_uavs_list), len(protection_point_list)  # 0, 1 # (1, 1)
        cost = np.zeros((N, M))
        for i, uav in enumerate(cur_uavs_list):
            for j, task in enumerate(protection_point_list):
                travel_dis = np.linalg.norm(uav['2Dpt'] - task[:2])
                # cost[i,j] = travel_dis + task.total_distance
                cost[i,j] = travel_dis
        
        # 求解
        from scipy.optimize import linear_sum_assignment
        if N <= M:  # # 0, 1
            uav_idx, task_idx = linear_sum_assignment(cost)
        else:
            task_idx, uav_idx = linear_sum_assignment(cost.T) # (array([], dtype=int64), array([], dtype=int64))
            
        # uav_order_id_list = []
        # 将结果 加入到传入 的 cur_uavs 的每一个元素中
        for i,j in zip(uav_idx, task_idx):
            uav, protection_point = cur_uavs_list[i], protection_point_list[j]
            # uav_order_id_list.append(cur_uavs_list[i]['uavROD']['order'])
            uav['protection_point'] = protection_point
            
        debug_flag = True
        if debug_flag:
            import matplotlib.pyplot as plt
            fig = plt.figure(dpi=128, figsize=(7,7))
            # ax = fig.add_subplot(111, projection="3d")
            # ax.plot(pos[0], pos[1], pos[2], marker='o', markersize=12)
            # ax.plot([pos[0],vec[0]], [pos[1],vec[1]], [pos[2],pos[2]], '--', linewidth=3)
            # ax.scatter(np.array(self.protection_points)[:,0], np.array(self.protection_points)[:,1], np.array(self.protection_points)[:,2])

            ax = fig.add_subplot(111)
            # 现在 新一轮的 方向

            ax.plot(gcs[0], gcs[1], marker='o', color="r", markersize=12, alpha = 0.5)
            ax.plot([gcs[0], gcs[0]+vec[0]], [gcs[1], gcs[1]+vec[1]], linewidth=3, alpha = 0.5)
            # 老一轮的 方向
            # if first_received == False:
            #     print('last')
            #     print(last)
            #     ax.plot(last[0], last[1], marker='s', color="r", markersize=12)
            #     ax.plot([last[0], last[0]+vec[0]], [last[1], last[1]+vec[1]], linewidth=3)
            #     ax.add_patch(plt.Circle(xy=(self.protectee_pos[:2]-self.D1*self.protectee_direction_vec[:2]), radius=self.R1, color='limegreen', alpha = 0.5))
            #     ax.add_patch(plt.Circle(xy=(self.protectee_pos[:2]+self.D2*self.protectee_direction_vec[:2]), radius=self.R2, color='yellow', alpha = 0.5))
            ax.add_patch(plt.Circle(xy=(self.protectee_pos[:2]-self.D1*self.protectee_direction_vec[:2]), radius=self.R1, color='limegreen', alpha = 0.5))
            ax.add_patch(plt.Circle(xy=(self.protectee_pos[:2]+self.D2*self.protectee_direction_vec[:2]), radius=self.R2, color='yellow', alpha = 0.5))
            
            # for i, order_id in enumerate(uav_order_id_list):
            #     ax.text(self.protection_points[i][0]+1, self.protection_points[i][1]+1, s=f"{order_id}")
            uav_x_list = [] # 一次传入 list 加速 画图
            uav_y_list = []
            for i, uav in enumerate(cur_uavs_list):
                ax.text(uav['protection_point'][0]+1, uav['protection_point'][1]+1, s=f"{uav['uavROD']['order']}", color='r', alpha=0.8)
                ax.text(uav['x']+1, uav['y']+1, s=f"{uav['uavROD']['order']}", color='k', alpha=0.8)
                ax.plot(
                    [uav['protection_point'][0], uav['x']], 
                    [uav['protection_point'][1], uav['y']], 
                    linestyle='dashed',
                    linewidth=1, alpha = 0.5)

                uav_x_list.append(uav['x'])
                uav_y_list.append(uav['y'])
            ax.scatter(uav_x_list, uav_y_list, marker="s", alpha = 0.5)
            ax.scatter(np.array(self.protection_points)[:,0], np.array(self.protection_points)[:,1], marker="x", alpha = 0.5)
            xlim_low = self.protectee_pos[0] - self.R1 - self.D1
            xlim_up  = self.protectee_pos[0] + self.R2 + self.D2
            ylim_low = self.protectee_pos[1] - self.R1 - self.D1
            ylim_up  = self.protectee_pos[1] + self.R2 + self.D2

            xlim_low = min(xlim_low, min(uav_x_list))
            xlim_up  = max(xlim_up, max(uav_x_list))
            ylim_low = min(ylim_low, min(uav_y_list))
            ylim_up  = max(ylim_up, max(uav_y_list))

            ax.set_xlim([xlim_low, xlim_up])
            ax.set_ylim([ylim_low, ylim_up])
            ax.set_aspect('equal') # 等比例显示刻度 # 是想要的效果

            # ax.set_xlim([20,180])
            # ax.set_ylim([25,185])

            string  = '-'.join(
                [
                    '废弃',
                    'R1+' + str(self.R1),
                    'R2+' + str(self.R2),
                    'D1+' + str(self.D1),
                    'D2+' + str(self.D2),
                    'uav_num+' + str(uav_num),
                ]
            )
            self.save_formation_picture(plt, string)
            # plt.show()
            # plt.pause(0.5)

            
        self.last_pos =  np.array(cur_gcs['2Dpt'])
        return cur_uavs  # {26: {'x': 0.011108373306534247, 'y': 0.008502610789715629, '2Dpt': array([0.01110837, 0.00850261]), 'uavROD': mappingproxy({'fly_status': 1, 'order': 26, 'task_status': 0, 'scout_end': -1, 'finished_point_ID': -1, 'lon': 116.1648486999, 'lat': 40.0553102999, 'alt': 30}), 'protection_point': array([15.,  0.,  2.])}, 27: {'x': 0.011108373306534247, 'y': 0.008502610789715629, '2Dpt': array([0.01110837, 0.00850261]), 'uavROD': mappingproxy({'fly_status': 1, 'order': 27, 'task_status': 0, 'scout_end': -1, 'finished_point_ID': -1, 'lon': 116.1648486999, 'lat': 40.0553102999, 'alt': 30}), 'protection_point': array([-1.5000000e+01,  2.4492936e-15,  2.0000000e+00])}}

    
    
    def first_protection_points(self, pos, uav_num=15):  # 更新任务信息，包括被保护对象的位置和方向向量等。 
        if self.last_pos == None: 
            vec = np.array([1,0,0])
        self.update(pos, vec, uav_num, True)
        # 加工数据  增加航点  分割空域 3航点
        return self.protection_points
    
    def not_first_protection_points(self, pos, uav_num=15):  # 更新任务信息，包括被保护对象的位置和方向向量等。 
        vec = np.array(pos) - np.array(self.last_pos) 
        self.update(pos, vec, uav_num, True)
        return self.protection_points

    def update(self, pos, vec, uav_num=15, debug_flag=False):  # 更新任务信息，包括被保护对象的位置和方向向量等。 
        length = np.linalg.norm(vec)
        # if length == 0:
        if length <= 1e-10:
            vec = [1, 0, 0] # x = 1 # MAVLINK 坐标系 NED 指向北  
        else:
            vec = vec / length
        if uav_num > 15:
            print("                error            ")
            print("                uav_num > 15:    ")
        self.last_protection_points = self.protection_points
        self.protection_points:List[np.ndarray] = []  # 更新任务信息，包括被保护对象的位置和方向向量等。  #
        self.protectee_pos = pos            # array([110. , 110. ,   0.5])
        self.protectee_direction_vec = vec  # array([1., 0., 0.])
        num1 = int(uav_num/2)  # 根据参数计算出保护圈内的 UAV 分布点，分为两组，分别计算对应的保护点坐标。
        num2 = uav_num - num1 # num1 <= num2
        if num1 <= num2:
            num1, num2 = num2, num1 
            # num1 >= num2

        # 1. 前C
        if num1 % 2 == 0: # 偶数
            theta1_list = [ self.theta1/2 + i*self.theta1 for i in range(int(num1/2))] \
                        + [ -self.theta1/2 - i*self.theta1 for i in range(int(num1/2))]
        else: # 奇数
            theta1_list = [0] + [i*self.theta1 for i in range(1, int(num1/2) + 1)] + [-i*self.theta1 for i in range(1, int(num1/2) + 1)]  #len==7# [0, 20.0, 40.0, 60.0, -20.0, -40.0, -60.0]  # self.theta1==20.0

        for theta in theta1_list:
            p = (
                self.protectee_pos[:2]
                - self.D1 * self.protectee_direction_vec[:2]
                + np.array(
                    [
                        self.R1
                        * (
                            vec[0] * np.cos(theta / 180 * np.pi)
                            - vec[1] * np.sin(theta / 180 * np.pi)
                        ),
                        self.R1
                        * (
                            vec[0] * np.sin(theta / 180 * np.pi)
                            + vec[1] * np.cos(theta / 180 * np.pi)
                        ),
                    ]
                )
            )

            self.protection_points.append(np.hstack((p, self.uav_height))) #len==7 # [[125.0, 110.0, 2.0], [123.79385241571816, 116.84040286651337, 2.0], [120.32088886237956, 122.85575219373078, 2.0], [115.0, 127.32050807568876, 2.0], [123.79385241571816, 103.15959713348663, 2.0], [120.32088886237956, 97.14424780626922, 2.0], [115.0, 92.67949192431124, 2.0]]

        # 2. 后C
        if num2 % 2 == 0: # 偶数
            theta2_list = [self.theta2/2 + i*self.theta2 for i in range(int(num2/2))] \
                        + [-self.theta2/2 - i*self.theta2 for i in range(int(num2/2))]
        else: # 奇数
            theta2_list = [0] + [i*self.theta2 for i in range(1, int(num2/2) + 1)] + [-i*self.theta2 for i in range(1, int(num2/2) + 1)] # [10.0, 30.0, 50.0, 70.0, -10.0, -30.0, -50.0, -70.0]

        for theta in theta2_list:
            p = (
                self.protectee_pos[:2]
                + self.D2 * self.protectee_direction_vec[:2]
                + np.array(
                    [
                        self.R2
                        * (
                            vec[0] * np.cos(np.pi - theta / 180 * np.pi)
                            - vec[1] * np.sin(np.pi - theta / 180 * np.pi)
                        ),
                        self.R2
                        * (
                            vec[0] * np.sin(np.pi - theta / 180 * np.pi)
                            + vec[1] * np.cos(np.pi - theta / 180 * np.pi)
                        ),
                    ]
                )
            )

            self.protection_points.append(np.hstack((p, self.uav_height)))  # len==7+8 # [-8:]==[[95.30384493975583, 113.4729635533386, 2.0], [97.67949192431122, 120.0, 2.0], [102.14424780626922, 125.32088886237956, 2.0], [108.15959713348663, 128.79385241571816, 2.0], [95.30384493975583, 106.5270364466614, 2.0], [97.67949192431122, 100.0, 2.0], [102.1442478062692, 94.67911113762044, 2.0], [108.15959713348663, 91.20614758428184, 2.0]] 

        # self.debug = False  # 若设置了 debug 标志为 True，则可视化保护圈和相关信息。
        # self.debug = False  # 若设置了 debug 标志为 True，则可视化保护圈和相关信息。
        # self.debug = True  # 若设置了 debug 标志为 True，则可视化保护圈和相关信息。
        self.debug = debug_flag  
        if self.debug:
            import matplotlib.pyplot as plt
            fig = plt.figure(dpi=128, figsize=(7,7))
            # ax = fig.add_subplot(111, projection="3d")
            # ax.plot(pos[0], pos[1], pos[2], marker='o', markersize=12)
            # ax.plot([pos[0],vec[0]], [pos[1],vec[1]], [pos[2],pos[2]], '--', linewidth=3)
            # ax.scatter(np.array(self.protection_points)[:,0], np.array(self.protection_points)[:,1], np.array(self.protection_points)[:,2])

            ax = fig.add_subplot(111)
            ax.plot(pos[0], pos[1], marker='o', color="r", markersize=12, alpha = 0.5)
            ax.plot([pos[0], pos[0]+vec[0]], [pos[1], pos[1]+vec[1]], linewidth=3, alpha = 0.5)
            ax.add_patch(plt.Circle(xy=(self.protectee_pos[:2]-self.D1*self.protectee_direction_vec[:2]), radius=self.R1, color='limegreen', alpha = 0.5))
            ax.add_patch(plt.Circle(xy=(self.protectee_pos[:2]+self.D2*self.protectee_direction_vec[:2]), radius=self.R2, color='yellow', alpha = 0.5))
            ax.scatter(np.array(self.protection_points)[:,0], np.array(self.protection_points)[:,1], marker="x", alpha = 0.5)
            xlim_low = self.protectee_pos[0] - self.R1 - self.D1
            xlim_up  = self.protectee_pos[0] + self.R2 + self.D2
            ylim_low = self.protectee_pos[1] - self.R1 - self.D1
            ylim_up  = self.protectee_pos[1] + self.R2 + self.D2

            ax.set_xlim([xlim_low, xlim_up])
            ax.set_ylim([ylim_low, ylim_up])
            # ax.set_xlim([20,180])
            # ax.set_ylim([25,185])

            string  = '-'.join(
                [
                    'R1+' + str(self.R1),
                    'R2+' + str(self.R2),
                    'D1+' + str(self.D1),
                    'D2+' + str(self.D2),
                    'uav_num+' + str(uav_num),
                ]
            )
            self.save_formation_picture(plt, string)
            # plt.show()
            # plt.pause(0.5)
        a = 1
        return self.protection_points

    def save_formation_picture(self, plt, string):
        def create_file(file_path:str) -> None:
            """创建文件或文件夹：
                - 结尾是'/'，则创建文件夹；
                - 否则，创建普通文件。
            """
            import os
            # 获取目录路径
            dir_path = os.path.dirname(file_path)
            # 如果目录不存在，则创建目录
            if not os.path.exists(dir_path):
                # print('create dir_path ', dir_path)
                os.makedirs(dir_path)

            # 如果文件不存在，则创建文件
            if not os.path.exists(file_path):
                # print('create file_path ', file_path)
                open(file_path, 'w').close()
        import os
        import datetime
        # 获取当前文件的目录
        # current_dir = os.path.dirname(os.path.abspath(__file__))
        file_name_without_extension = os.path.splitext(__file__)[0]
        current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        # fig_path = current_dir + '/' + file_name_without_extension + '/' + current_time + '.png'
        # fig_path =  file_name_without_extension + '/' + current_time + '.png'
        fig_path =  file_name_without_extension + '/' + string + '-' + current_time + '.png'
        create_file(file_path=fig_path)
        print('save_formation_picture')
        print('fig_path')
        print(fig_path)
        plt.savefig(fig_path, dpi=600, bbox_inches='tight')


class BuildingTask(Task):
    def __init__(self, task_id:Tuple[str, int], init_assign: int, key_points: np.ndarray, building_id:int) -> None:
        super().__init__(task_id, init_assign, key_points)
        self.building_id = building_id


if __name__ == '__main__':
    def test_defense():
        
        arg = [20, 20, 5, 5]
        arg = [20, 15, 5, 5]
        arg = [20, 20, 5, 5]
        d = DefenseTask('task_type', 'task_ID', arg)
        pos =  np.array([110. , 110. ,   0.5])
        vec =  np.array([1., 0., 0.])
        vec =  np.array([0., 1., 0.])
        uav_num = 15
        for i  in range(15):
            # d.update(pos, vec, uav_num, True)
            d.update(pos, vec, i+1, True)
    
    def test_one_defense():
        
        arg = [20, 20, 5, 5]
        arg = [20, 15, 5, 5]
        arg = [20, 20, 5, 5]
        d = DefenseTask('task_type', 'task_ID', arg)
        pos =  np.array([110. , 110. ,   0.5])
        vec =  np.array([1., 0., 0.])
        vec =  np.array([0., 1., 0.])
        uav_num = 15
        for i  in range(15):
            # d.update(pos, vec, uav_num, True)
            d.update(pos, vec, i+1, True)

    # test_defense()
    test_one_defense() # 测试一个 C 的保护
