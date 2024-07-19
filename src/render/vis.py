import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from matplotlib.patches import Polygon

from src.robotR import SharedInformation
from src.robotR import SharedInformation, Plain, PolygonRegion, LineSeg

COLOR_DICT = {
    "uav": "#8FAADC",     # light blue
    "target_unknown": "#92D050",     # green
    "target_known": "#F0403C",     # red
    "typeD": "#FFC000",     # orange
    "plain": "grey",
    "building": "ghostwhite",
    "plain_searched": "k",
    "building_searched": "k"
}

class Visualizer():
    def __init__(self,  fig:Figure, SharedInf:SharedInformation, save_flag=False) -> None:
        self.t = 0
        self.path = os.path.abspath(os.path.join(os.getcwd(), "."))
        self.save_flag = save_flag
        self.fig = fig
        self.SharedInf = SharedInf
        # self.ax = fig.add_axes([0,0.05,1,0.7], projection="3d")
        # self.info_ax = fig.add_axes([0,0.75,1,0.2])
        # self.ax = fig.add_subplot(2,1,2, projection="3d")
        # self.info_ax = fig.add_subplot(2,1,1)
        axs = fig.subplots(2, 1, height_ratios=[2,9])
        axs[0].remove()
        self.info_ax = fig.add_subplot(2,1,1)
        axs[1].remove()    
        self.ax = fig.add_subplot(2,1,2, projection="3d")
        self.fig.subplots_adjust(top=0.99,
                                 bottom=0.02,
                                 left=0.01,
                                 right=0.99,
                                 hspace=0.01,
                                 wspace=0.2)


        self.info_ax.set_facecolor("none")
        self.info_ax.xaxis.set_visible(False)
        self.info_ax.yaxis.set_visible(False)
        self.info_ax.spines['right'].set_visible(False)
        self.info_ax.spines['left'].set_visible(False)
        self.info_ax.spines['top'].set_visible(False)
        self.info_ax.spines['bottom'].set_visible(False)


    def update(self):
        map_width, map_length, map_height = self.SharedInf.world_config["map"]["width"], self.SharedInf.world_config["map"]["length"], self.SharedInf.world_config["map"]["height"]
        self.ax.cla()
        self.ax.set_xlim(0, map_width)
        self.ax.set_ylim(0, map_length)
        self.ax.set_zlim(0, map_height)
        self.ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))  # 接受一个四元组 (r, g, b, alpha) 作为参数，
        self.ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))  # 分别代表了红色、绿色、蓝色和透明度的数
        self.ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))  # 背景颜色为白色，并且透明度为 0。
        # self.ax.w_xaxis._axinfo['grid']['linewidth'] = 0.
        # self.ax.w_yaxis._axinfo['grid']['linewidth'] = 0.
        # self.ax.w_zaxis._axinfo['grid']['linewidth'] = 0.
        self.ax.grid(False) # 去掉背景网格
        # self.ax.set_aspect('equal')

        self.draw_building()  # 发现楼 # 则画出来
        # self.draw_plain()
        if type(self.SharedInf.plain) == Plain:
            self.draw_plain() # rectangle
        elif type(self.SharedInf.plain) == PolygonRegion:
            self.draw_polygon() # 多边形
        self.draw_tree()  # 暂未使用
        self.draw_uav()  # 画出无人机
        self.draw_target()  # 发现楼 # 则画出来

        # self.draw_plain_task()  # LQY 2023.10.27 注释掉了
        # self.draw_building_task()  # # LQY 2023.10.27 注释掉了

        self.info_ax.cla()
        self.plot_task_info()
        if self.save_flag:
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
                    
            filename = self.path + "\\sim_figs2\\frame_{:03d}.png".format(self.t)
            create_file(filename)
            plt.savefig(filename, dpi=600, bbox_inches='tight')  # FileNotFoundError: [Errno 2] No such file or directory: 'F:\\project\\2023\\07\\GuoMeng\\ZhuoZhi\\github-collaborate\\zhuozhi\\sim_figs2\\frame_000.png'  
        else:
            plt.pause(0.01)
        self.t += 1

        
    def draw_building(self):
        debug = False
        for building in self.SharedInf.building_dict.values():
            if building.discovered == True or debug:
                x = np.linspace(building.anchor[0], building.anchor[0] + building.width, 2)
                y = np.linspace(building.anchor[1], building.anchor[1] + building.length, 2)
                z = np.linspace(0, building.height, 2)
                X, Y = np.meshgrid(x, y)
                Z1 = np.zeros((2, 2))
                Z2 = np.full((2, 2), building.height)
                self.ax.plot_surface(X, Y, Z1, alpha=0.7, color=COLOR_DICT["building"])
                self.ax.plot_surface(X, Y, Z2, alpha=0.7, color=COLOR_DICT["building"])
                X, Z = np.meshgrid(x, z)
                Y1 = np.full((2, 2), building.anchor[1])
                Y2 = np.full((2, 2), building.anchor[1] + building.length)
                self.ax.plot_surface(X, Y1, Z, alpha=0.7, color=COLOR_DICT["building"])
                self.ax.plot_surface(X, Y2, Z, alpha=0.7, color=COLOR_DICT["building"])
                Y, Z = np.meshgrid(y, z)
                X1 = np.full((2, 2), building.anchor[0])
                X2 = np.full((2, 2), building.anchor[0] + building.width)
                self.ax.plot_surface(X1, Y, Z, alpha=0.7, color=COLOR_DICT["building"])
                self.ax.plot_surface(X2, Y, Z, alpha=0.7, color=COLOR_DICT["building"])

            if debug:
                self.ax.plot(building.key_points[:,0], building.key_points[:,1], building.key_points[:,2])

    def draw_plain(self):
        p = self.SharedInf.plain
        x = np.linspace(p.anchor[0], p.anchor[0]+p.width, 2)
        y = np.linspace(p.anchor[1], p.anchor[1]+p.length, 2)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros((2,2))
        self.ax.plot_surface(X, Y, Z, alpha=0.2, color=COLOR_DICT["plain"])

    def draw_polygon(self):
        v = np.array(self.SharedInf.plain.vertices)
        v = np.vstack((v, v[0]))
        self.ax.plot(v[:,0], v[:,1], np.zeros(v.shape[0]))  # 带扫描多边形区域  

    # TODO
    def draw_tree(self):
        pass

    def draw_uav(self):
        for uav in self.SharedInf.uav_dict.values():
            if uav.active == False:
                continue
            T = uav.transformation_matrix() # (3,4) # [[1.0, 0.0, 0.0, 327.1320643266746], [0.0, 1.0, 0.0, 402.07519493594015], [-0.0, 0.0, 1.0, 30.0]]
            for p in uav._points:  # len==4 # [[7.0710678118654755, 7.0710678118654755, 0.0, 1.0], [-7.071067811865475, 7.0710678118654755, 0.0, 1.0], [-7.071067811865475, -7.0710678118654755, 0.0, 1.0], [7.0710678118654755, -7.0710678118654755, 0.0, 1.0]]
                p_t = np.matmul(T, p)  # (3,) # [334.20313213854007, 409.1462627478056, 30.0]
                self.ax.plot([uav.pos[0], p_t[0]], [uav.pos[1], p_t[1]], [uav.pos[2], p_t[2]], color="k")  # 黑色横杠支架
                self.ax.scatter(p_t[0], p_t[1], p_t[2], color=COLOR_DICT["uav"], s=6)  # 蓝色支架末端点
            self.ax.text(uav.pos[0]+5,
                         uav.pos[1]+5,
                         uav.pos[2],
                        #  s=str(uav.id)+f"_{uav.task.task_id}",
                         s=f"{uav.id}",
                         fontsize=7)  # 显示uav.id编号
            
            # plot FOV
            # for p in uav._pt_fov:
            #     p_t = np.matmul(T, p)
            #     self.ax.plot([uav.pos[0], p_t[0]], [uav.pos[1], p_t[1]], [uav.pos[2], p_t[2]], color="k")
            
            # for i in range(4):
            #     p1, p2 = uav._corner_fov[i], uav._corner_fov[(i+1)%4]
            #     p1, p2 = np.matmul(T, p1), np.matmul(T, p2)
            #     self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color="r")
            
            # for i in range(4):
            #     p1, p2 = uav._ground_fov[i], uav._ground_fov[(i+1)%4]
            #     p1, p2 = np.matmul(T, p1), np.matmul(T, p2)
            #     self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color="grey")

            
            # plot the yaw (now the angle is the globally relative angle between the GCS and UCS)
            p_t = np.matmul(T, uav.head_pt)  # [347.1320643266746, 402.07519493594015, 30.0]
            self.ax.plot([uav.pos[0], p_t[0]], [uav.pos[1], p_t[1]], [uav.pos[2], p_t[2]], color="k", linewidth=0.8, ls="--") # 机头 黑色虚线 
            self.ax.scatter(p_t[0], p_t[1], p_t[2], color="r", s=6, marker="<") # 机头 左红色三角

            # plot the optical axis
            p_t = np.matmul(T, uav.optical_axis)  # [349.8465406848261, 402.07519493594015, 2.9299411962766833]
            self.ax.plot([uav.pos[0], p_t[0]], [uav.pos[1], p_t[1]], [uav.pos[2], p_t[2]], color="orange", linewidth=0.8)  # 黄色实线 垂直向下
            self.ax.scatter(p_t[0], p_t[1], p_t[2], color="orange", s=6, marker="v")  # 黄色 下三角     

            # draw the planned traj
            max_sec_num = 3 + 1
            task = uav.task
            self.ax.plot([uav.pos[0], task.target_pos[0]], [uav.pos[1], task.target_pos[1]], [uav.pos[2], task.target_pos[2]], color="navy", ls="dashed", linewidth=1)  # 当前位置-->最近目标点 == 当前需要飞过去的点 # 蓝色虚线
            self.ax.plot(task.key_points[task.cur_index:task.cur_index+max_sec_num,0], task.key_points[task.cur_index:task.cur_index+max_sec_num,1], task.key_points[task.cur_index:task.cur_index+max_sec_num,2], color="navy", ls="dashed", linewidth=1)  # 画出接下来的 4 段 # max_sec_num == 4
    
    def draw_target(self):
        for target in self.SharedInf.target_dict.values():  # 9 个 目标
            if target.active == False:
                continue
            if target.discovered:
                self.ax.scatter(target.pos[0], target.pos[1], target.pos[2], color= COLOR_DICT["target_known"],label='detected target')  # 红色 已经发现 
            else:
                self.ax.scatter(target.pos[0], target.pos[1], target.pos[2], color= COLOR_DICT["target_unknown"], alpha=0.7, label='undetected target') # 绿色 未发现
            
    def draw_plain_task(self):
        x = np.linspace(0, self.SharedInf.plain.width, self.SharedInf.plain.width+1)  # AttributeError: 'PolygonRegion' object has no attribute 'width'
        y = np.linspace(0, self.SharedInf.plain.length, self.SharedInf.plain.length+1)
        X, Y = np.meshgrid(x, y)
        Z = self.SharedInf.plain.uncertainty_map.T

        if np.all(Z==1):
            self.ax.contourf(X,Y,Z, colors='grey', levels=20, alpha=0.5)
        else:
            self.ax.contourf(X,Y,Z, colors=['white', 'grey'], levels=20, alpha=0.5)

    def draw_building_task(self):
        for building in self.SharedInf.building_dict.values():
            if building.discovered == True:
                x = np.linspace(building.anchor[0], building.anchor[0] + building.width, building.width + 1)
                y = np.linspace(building.anchor[1], building.anchor[1] + building.length, building.length + 1)
                z = np.linspace(0, building.height, building.height + 1)
                X, Z = np.meshgrid(x, z)

                if np.all(building.uncertainty_map[0].T == 1):
                    self.ax.contourf(X, building.uncertainty_map[0].T, Z, alpha=0.7, zdir='y', offset=building.anchor[1], levels=20, colors=COLOR_DICT["building_searched"])
                else:
                    self.ax.contourf(X, building.uncertainty_map[0].T, Z, alpha=0.7, zdir='y', offset=building.anchor[1], levels=20, colors=["grey", COLOR_DICT["building_searched"]])
                
                if np.all(building.uncertainty_map[2].T == 1):
                    self.ax.contourf(X, building.uncertainty_map[2].T, Z, alpha=0.7, zdir='y', offset=building.anchor[1] + building.length, levels=20, colors=COLOR_DICT["building_searched"])
                else:
                    self.ax.contourf(X, building.uncertainty_map[2].T, Z, alpha=0.7, zdir='y', offset=building.anchor[1] + building.length, levels=20, colors=["grey", COLOR_DICT["building_searched"]])          
                
                Y, Z = np.meshgrid(y, z)

                if np.all(building.uncertainty_map[1].T == 1):
                    self.ax.contourf(building.uncertainty_map[1].T, Y, Z, alpha=0.7, zdir='x', offset=building.anchor[0] + building.width, levels=20, colors=COLOR_DICT["building_searched"])
                else:
                    self.ax.contourf(building.uncertainty_map[1].T, Y, Z, alpha=0.7, zdir='x', offset=building.anchor[0] + building.width, levels=20, colors=["grey", COLOR_DICT["building_searched"]])
                
                if np.all(building.uncertainty_map[3].T == 1):
                    self.ax.contourf(building.uncertainty_map[3].T, Y, Z, alpha=0.7, zdir='x', offset=building.anchor[0], levels=20, colors=COLOR_DICT["building_searched"])
                else:
                    self.ax.contourf(building.uncertainty_map[3].T, Y, Z, alpha=0.7, zdir='x', offset=building.anchor[0], levels=20, colors=["grey", COLOR_DICT["building_searched"]])

    def plot_path(self,path,col='--g'):
        path_x = []
        path_y = []
        path_z = []
        for path_i in path:
            # print(path_i)
            path_x.append(path_i[0])
            path_y.append(path_i[1])
            path_z.append(2)
        self.ax.plot(path_x, path_y, path_z, col, linewidth=1.5)

    def plot_task_info(self):
        rows = ["Task Type", "Task ID"]
        cols = [f"UAV {i}" for i in range(15)]

        data_1 = [uav.task.task_id[0] if uav.active else "-" for uav in self.SharedInf.uav_dict.values()]
        data_2 = [uav.task.task_id[1] if uav.active else "-" for uav in self.SharedInf.uav_dict.values()]

        data = [data_1, data_2]
        self.info_ax.table(cellText=data, rowLabels=rows, colLabels=cols, 
                           cellLoc="center", rowLoc="center", colLoc="center",
                           bbox=[0.14,0,0.86,1], fontsize=12)
