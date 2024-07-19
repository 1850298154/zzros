import os
import sys
import time
import argparse
from typing import Literal, Union, Dict, List, Tuple

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from .worldR import World

class WorldEXP(World): # # 仿真动画,output_type,# 参数,CONSTANTS_RADIUS_OF_EARTH,args,logfile,# 任务UAV分配,assignment  ###  # 加载到self._config # 并构造四个对象容器 uav_dict,target_dict,building_dict,plain # ,load_world_config(),construct_world()
    def __init__(self, args:Dict, uav_id_2_xylist, scan_areas_id=0) -> None: # # 加载到self._config # 并构造四个对象容器 uav_dict,target_dict,building_dict,plain # ,load_world_config(),construct_world()
        super().__init__(args=args, uav_id_2_xylist=uav_id_2_xylist,  scan_areas_id=0)

    def simulate(self, region_polygon_xylist_list, half_scout_range):
        super().simulate()
        T_period = 0
        t_now = time.time()  # 1708914342.5501292
        if self.args["render"]:
            self.vis_init()  # # 初始化空的: 2D信息+3D显示 info_ax,ax
        
        if True: # test
            import numpy as np
            from .robotR import PolygonRegion
            scout_range = self.plain.scout_range  #  5.976508219484396
            # scout_range = 5  #  5.976508219484396
            scout_range = half_scout_range  #  5.976508219484396
            # vertices = [[100,100], [200,100], [300,150]]
            vertices = region_polygon_xylist_list
            # vertices = [[100,100], [200,100], [300,150], [375,225], [400,300], [200,250], [100, 150]]
            # vertices = [[115,100], [300,100], [300,300], [125, 200]]
            self._plain = PolygonRegion(vertices=vertices, scout_range=scout_range, uav_num=len(self.uav_dict))  # 多边形区域 vertices,scout_range,uav_num

            
            idx = np.random.choice(list(self.uav_dict.keys()))  # 1
            selected_uav = self.uav_dict[idx]  # <src.robotR.UAV object at 0x000001B2819CD690>
            self.init_planer.update_params(offset=selected_uav.offset,
                                            search_height=self.config["planner"]["search_height"]) # 初始规划器 更新: 高度+偏移（未使用，拐角处）

            # --------- Update Plain --------------
            idx = np.random.choice(list(self.uav_dict.keys()))  # 1
            selected_uav = self.uav_dict[idx]  # <src.robotR.UAV object at 0x000001B2819CD690>
            self._plain.update_start_point(selected_uav.pos[:2])  # 多边形平原 更行: 随机uav 起点 xy  # 为后面  分割平面 求该起点 到多边形区域各个边界的距离 # 获得最近的多边形线段 平行 所有分割

            # --------- Update SharedInf ----------
            self.SharedInf.update(self.uav_dict, self.target_dict, self.building_dict, self.plain)  #   # 第一次更新 前面加入的构造好的实例对象： # uav_dict,target_dict,building_dict,plain # 目的是 同步共享

        self.init_plan()  # # 航线分配 + {uav:task}  # 使用 world 构造的 规划器 进行初次规划

        uav_id_2_xylist_list = {}
        for k, uav in self.uav_dict.items():
            # print('k', k)
            # print('uav.task.key_points')
            # print(uav.task.key_points)
            # uav_id_2_xylist_list[k] = list(uav.task.key_points[:2])
            uav_id_2_xylist_list[k] = np.array(uav.task.key_points)[:, :2].tolist()


        return uav_id_2_xylist_list
        for t in range(2500):
            if t % 100 == 0:
                print("="*40)
                print("time now is -------->", t)
                if self.args["render"]:
                    self.update_world_vis()  # 更新世界可见
                    # self.plot_2d() # 不能直接 把 LQY 的代码拿过来直接用
                    import matplotlib.pyplot as plt
                    plt.pause(0.01)


                
                for uav in self.uav_dict.values():
                    # uav.scan()
                    pass

                for uav in self.uav_dict.values():
                    uav.move()

                self.online_plan()


# if __name__ == '__main__':
def cpp_solve(uav_id_2_xylist, region_polygon_xylist_list, half_scout_range) -> dict:
    """
    This function calculates the solution for cpp problem involving UAVs and regions within a given
    range.
    
    example:
    uav_id_2_xylist             =   {
                                        111:[0,0],
                                        # 222:[300,300],
                                        222:[-10,-10],
                                    }
    region_polygon_xy_list_list =   [[0,0], [100,0], [100,100], [0,100]]
    half_scout_range            =   5
    
    :param uav_id_2_xylist: The function `cpp_solve` seems to take three parameters:
    :param region_polygon_xylist_list: The `region_polygon_xylist_list` parameter is a list of polygons
    representing regions in a two-dimensional space. Each polygon is defined by a list of (x, y)
    coordinates that form the vertices of the polygon. The list `region_polygon_xylist_list` contains
    multiple such polygons, each represented
    :param half_scout_range: The `half_scout_range` parameter likely represents half the distance that a
    UAV can scout in a given region. This parameter is used to determine the range within which the UAV
    can operate effectively
    """
    
    # cutomize hyperparameters
    # parser = argparse.ArgumentParser(description='GFKDproject parameter initialization')
    # parser.add_argument('--world_config_file', default="world_config1.yaml",
    #                     help='path of world parameters')
    # parser.add_argument('--render', type=bool, default=True,
    #                     help='decide whether show animation')
    # parser.add_argument('--save_fig', type=bool, default=True,
    #                     help='decide whether save fig')
    # parser.add_argument('--attack', type=int, default=1,
    #                     help='decide whether attack observed targets')
    # args = parser.parse_args()
    args = {"world_config_file":"world_config.yaml",
            "render": False, # or True,
            "save_fig": False  or True,
            "attack": 1}
    fs_world = WorldEXP(args, uav_id_2_xylist)  #  WorldEXP(World) ###  # args # 加载到self._config # 并构造四个对象容器 uav_dict,target_dict,building_dict,plain # ,load_world_config(),construct_world()

    uav_id_2_xylist_list = fs_world.simulate(region_polygon_xylist_list, half_scout_range)
    return uav_id_2_xylist_list

if __name__ == '__main__':
    uav_id_2_xylist = {
        111:[0,0],
        # 222:[300,300],
        222:[-10,-10],
    }
    # region_polygon_xy_list_list = [[100,100], [200,100], [300,150], [375,225], [400,300], [200,250], [100, 150]]
    # region_polygon_xy_list_list = [[100,100], [200,100], [200,200], [100,200]]
    region_polygon_xy_list_list = [[0,0], [100,0], [100,100], [0,100]]
    # vertices = [[115,100], [300,100], [300,300], [125, 200]]
    uav_id_2_xylist_list = cpp_solve(
        uav_id_2_xylist=uav_id_2_xylist, 
        region_polygon_xylist_list=region_polygon_xy_list_list,
        half_scout_range=5
        )
    
    print("uav_id_2_xylist_list")
    # print(uav_id_2_xylist_list)
    for k, v in uav_id_2_xylist_list.items():
        print(k)
        print(*v, sep='\n')
