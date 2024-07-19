from math import floor

import numpy as np
import shapely
from scipy.optimize import linear_sum_assignment

from src.robotR import UAV, Target, Task, SharedInformation, PolygonRegion, BuildingTask
from .assignment import Assignment, Coalition
from ..utils import move_alone_line, norm
from typing import Literal, Union, Dict, List

class PlannerBase():
    def __init__(self, SharedInf:SharedInformation) -> None:
        self.SharedInf = SharedInf

class InitPlanner(PlannerBase):
    def __init__(self, SharedInf) -> None:
        super().__init__(SharedInf)
        self.assignment:Assignment = None

    def update_params(self, search_height, offset):
        self.search_height = search_height # 30
        self.offset = offset # 19.222708411399005

    def plan(self) -> Assignment:
        # self.uavs_traj_plan()
        self.uavs_traj_plan_edge()  # <1>入口,<2>旅行一段,<3>搜索(含入口点)  # start_point{1个xy} # travel_traj{2个xy= pos + start_point} # search_traj{12个xy, 无 pos, 包含有 start_point}

        # convert to 3d, construct task for each
        self.construct_task()  # 构造任务信息task # uav 和 SharedInf 共享

        # because there's a gap between uav_pos and camera FOV projected on the ground
        self.task_offset()  # 现在不需要, 多边形拐角做偏移

        # self.debug_display()
        # self.debug_display_polygon()
        self.new_keypoint_debug_display_polygon()

    def uavs_traj_plan_edge(self):
        """All Search Trajs start from the edge point
        ＿＿＿＿＿＿＿
        |1  2  5  6         |
        | n  _              |
        |0  3  4  7 ...     |
        ￣￣￣￣￣￣￣  
        [0, 3, 4, 7, 8, 11, 12, 15, 16, 19, 20, 23]
        截断任务 
            第一段 包含三个直线 n [0,1,2,3]
            第二段 包含一个直线 _ [3,4]
        """
        
        # 1 decide enter edge
        plain:PolygonRegion = self.SharedInf.plain # SharedInf.更新获得world中的 多边形区域
        plain.initilize_boundpoint_list_edge() # 返回蛇形走位的边界点 self.bound_points  

        # select the point on edge
        # FIXME　if the last one is needed
        edge_pts = [0]  # 分成多个任务段 #  每次的起点 # len==12 # [0, 3, 4, 7, 8, 11, 12, 15, 16, 19, 20, 23]
        i = 0
        while i < len(plain.bound_points):
            i += 3
            if i < len(plain.bound_points):
                edge_pts.append(i)
            i += 1
            if i < len(plain.bound_points):
                edge_pts.append(i)
        
        if edge_pts[-1] + 1 != len(plain.bound_points):
            edge_pts.append(len(plain.bound_points) - 1)

        # 2 divide on average
        uav_num = len(self.SharedInf.uav_dict) # 2个 无人机 分配
        pts_num = len(edge_pts)                # 12个 任务 待分配

        # 2.1 uav_num <= pts_num
        if uav_num <= pts_num:
            workload = [0]  # 每一段的代价# len==12  # [0, 411.91708861781666, 24.596747752497542, 567.3243972662001, 21.38015694776607, 547.7155595477786, 21.380156947766086, 518.3822262144453, 21.380156947766142, 477.06178222012727, 21.380156947766114, 377.9796282597071]
            for i in range(len(edge_pts) - 1):
                s_idx, e_idx = edge_pts[i], edge_pts[i+1]
                d = 0
                for j in range(s_idx, e_idx):
                    d += np.linalg.norm(plain.bound_points[j+1] - plain.bound_points[j])
                workload.append(d)
                            
            workload_cumsum = np.cumsum(workload)  # len==12 # [0.0, 411.91708861781666, 436.5138363703142, 1003.8382336365144, 1025.2183905842805, 1572.9339501320592, 1594.3141070798254, 2112.6963332942705, 2134.0764902420365, 2611.1382724621635, 2632.5184294099295, 3010.4980576696366]
            average_workload = workload_cumsum[-1] / uav_num  # 1505.2490288348183
            
            # init split pts selection 
            # can be imporved iterately
            split_idx = [0]  # len==uav+1==3 # [0, 6, 11]
            basic_seg_n = floor((len(edge_pts) - 1) / uav_num)  # (12-1)//2 == 5
            reminder = (len(edge_pts) - 1) % uav_num  # (12-1)%2 == 1
            for i in range(uav_num):
                if i < reminder:  # 第一个无人机， 除了基数(商)， 多分配一个余数
                   split_idx.append(split_idx[-1] + basic_seg_n + 1) # 分配 basic_seg_n + 1 节段任务
                else:
                    split_idx.append(split_idx[-1] + basic_seg_n) #  # 分配 basic_seg_n     节段任务


            split_dis = []  # len==uav==2 # [1594.3141070798254, 1416.1839505898113]
            for i in range(len(split_idx)-1):
                split_dis.append(workload_cumsum[split_idx[i+1]] - workload_cumsum[split_idx[i]])

            # assign trajs for uav
            cost = np.zeros((uav_num, uav_num)) # shape==(2,2)  # [[1757.8154329322974, 1538.0220723934438], [1831.2555387070995, 1610.8647852850097]]
            for i, uav in enumerate(self.SharedInf.uav_dict.values()):
                for j in range(uav_num):
                    total_dis = np.linalg.norm(uav.pos[:2] - plain.bound_points[edge_pts[split_idx[j]]]) + split_dis[j]  # i(uav到起点) 和 j(扫描区域) 的排列组合  
                    cost[i,j] = total_dis  #  数量为 i*j 矩阵  # 最后结果为 投影到xy两个轴后 横竖 都是满的  # 约束为 一行|一列 只有一个元素

            _, task_idx = linear_sum_assignment(cost)  # _, task_idx == (array([0, 1], dtype=int64), array([0, 1], dtype=int64))

            # verification

            for i, j in zip(list(self.SharedInf.uav_dict.keys()), task_idx):
                uav = self.SharedInf.uav_dict[i] # i=0->1  # i=1
                uav.start_point = plain.bound_points[edge_pts[split_idx[j]]]  # 扫描起点 # 多边形区域边界上的第 edge_pts[split_idx[2]]==edge_pts[6]==12 个点  [321.47407426, 280.36851857]
                uav.travel_traj = [np.array(uav.pos[:2]), uav.start_point]  # 当期位置 + 扫描起点  # [[313.3648234926275, 474.88038825386116], [321.4740742630289, 280.3685185657572]]
                uav.search_traj = plain.bound_points[edge_pts[split_idx[j]]: edge_pts[split_idx[j+1]]+1]  # len==12 # [edge_pts[6]: edge_pts[11]+1] == [12:23 +1] # [[321.4740742630289, 280.3685185657572], [141.10555569727168, 100.0], [125.54920651116764, 100.0], [300.73227534822354, 275.1830688370559], [279.9904764334181, 269.9976191083545], [109.9928573250636, 100.0], [100.0, 105.56349186104046], [259.24867751861274, 264.8121693796532], [238.50687860380734, 259.6267196509518], [100.0, 121.11984104714449], [100.0, 136.67619023324852], [217.76507968900196, 254.44126992225048]]
                pass

        # 2.2 uav_num > pts_num
        else:
            self.uavs_traj_plan()
            # self.debug_display_polygon()


        ###########debug##################
        # import matplotlib.pyplot as plt
        # from matplotlib import patches, cm
        # plt.figure(figsize=(8,8))
        # ax = plt.subplot(1,1,1)
        # polygon = patches.Polygon(plain.vertices, closed = True, fill = False)
        # ax.add_artist(polygon)

        # plain = self.SharedInf.plain
        # b_pts = np.array(plain.bound_points)
        # ax.plot(b_pts[:,0], b_pts[:,1], color="Salmon", marker='.')
        # # ax.plot([plain.start_point[0], b_pts[0][0]], [plain.start_point[1], b_pts[0][1]], color="Crimson")
        

        # split_real_idx = [edge_pts[i] for i in split_idx]
        # for idx, pt in enumerate(b_pts):
        #     # if idx in edge_pts:
        #     if idx in split_real_idx:
        #         c = 'r'
        #     else:
        #         c = 'k'
        #     ax.text(pt[0], pt[1], s=f"({idx})", c=c, fontsize=12)


        # # for idx, v in enumerate(plain.vertices):
        # #     ax.text(v[0], v[1], s=f"{idx}#", fontsize=12)

        # rb = cm.get_cmap('rainbow', uav_num)
        # for uav in self.SharedInf.uav_dict.values():
        #     ax.scatter(uav.pos[0], uav.pos[1], marker='o', color=rb(uav.id%uav_num))
        #     ax.text(uav.pos[0], uav.pos[1], s=f"{uav.id}", color=rb(uav.id%uav_num))

        # rb8 = cm.get_cmap('rainbow', 8)
        # # target_pos = [target.pos for target in self.target_list]
        # # ax.scatter(np.array(target_pos)[:,0], np.array(target_pos)[:,1], marker='d', color="r")

        # for i, uav in self.SharedInf.uav_dict.items():
        #     traj = np.array(uav.search_traj)
        #     ax.plot([uav.pos[0], uav.start_point[0]], [uav.pos[1], uav.start_point[1]], color=rb8(i%8))
        #     ax.scatter(uav.start_point[0],uav.start_point[1], marker='o', color=rb8(i%8))
        #     ax.plot([uav.start_point[0], traj[0][0]], [uav.start_point[1], traj[0][1]], color=rb8(i%8))
        #     ax.plot(traj[:,0], traj[:,1], marker='x', color=rb8(i%8))
        
        # plt.show()

        # print(1)


    
    def uavs_traj_plan(self):
        """original version, containing iteration
        """
        self.SharedInf.plain.workload_allocate()
        idx_2_uavid, uavid_2_idx = self.init_traj_assign()

        dis_threshold = 5
        iter_count = 0
        task = self.SharedInf.plain
        traj_uavs = task.traj_uavs
        search_dis = task.search_dis
        
        travel_dis = []
        # 1 计算每个uav的总路程
        for uav_id, uav in self.SharedInf.uav_dict.items():
            idx = uavid_2_idx[uav_id]
            p1 = traj_uavs[idx][0]
            p2 = traj_uavs[idx][-1]
            travel_dis.append(min(norm(p1 - uav.pos[:2]), norm(p2 - uav.pos[:2]))) # 动态决定uav的start point
        
        total_dis = [search_dis[i] + travel_dis[i] for i in range(len(self.SharedInf.uav_dict))]
        # print(f"[Iter Info] Iter Count: {iter_count}\n\tTravel_dis: {travel_dis}\n\tSearch_dis: {search_dis}\n\tTotal_dis: {total_dis}")
        # print(f"[Iter Info] Iter Count: {iter_count}, Total Total Dis: {sum(total_dis)}, Dis: {abs(total_dis[0] - total_dis[-1])}\n\tTotal_dis: {total_dis}")
        # self.debug_display(travel_traj=False)
        # 2 迭代调整每个uav的总路程
        while True:
            if (iter_count % 4) == 3 :
                alpha = 0.9
            else:
                alpha = 0.5 
            for i in range(len(total_dis) - 1):
                dis_delta = total_dis[i+1] - total_dis[i]
                if dis_delta > 0:
                    # split_point从i+1号uav的traj里面找
                    # 计算i+1号uav的分段路径 累积和

                    segment = [0]
                    segment += [norm(traj_uavs[i+1][j] - traj_uavs[i+1][j+1]) for j in range(len(traj_uavs[i+1]) - 1)]
                    segment_cumsum = np.cumsum(segment)

                    delta = dis_delta * alpha # 偏移量
                    if np.where(segment_cumsum >= delta)[0].shape[0] == 0:
                        segment_idx = len(segment_cumsum) - 1
                    else:
                        segment_idx = np.where(segment_cumsum >= delta)[0][0]
                    dis_delta_segment = delta - segment_cumsum[segment_idx - 1]
                    if (delta - segment_cumsum[segment_idx - 1]) > (segment_cumsum[segment_idx] - segment_cumsum[segment_idx-1]):
                        dis_delta_segment = (segment_cumsum[segment_idx] - segment_cumsum[segment_idx-1]) * 0.8 
                    p_split = move_alone_line(traj_uavs[i+1][segment_idx-1], traj_uavs[i+1][segment_idx], dis_delta_segment)

                    # 注意！这里删掉了原来的split_point
                    traj_uavs[i] = traj_uavs[i][:-1] + [traj_uavs[i+1][j] for j in range(1, segment_idx)] + [p_split]
                    traj_uavs[i+1] = [p_split] + [traj_uavs[i+1][j] for j in range(segment_idx, len(traj_uavs[i+1]))]

                else:
                    # split_point从i号uav的traj里面找
                    # 计算i+1号uav的分段路径 累积和（注意要反一下顺序）
                    segment = [0]
                    segment += [norm(traj_uavs[i][j] - traj_uavs[i][j-1]) for j in range(len(traj_uavs[i]) - 1, 0, -1)]
                    segment_cumsum = np.cumsum(segment)

                    delta = - dis_delta * alpha
                    if np.where(segment_cumsum >= delta)[0].shape[0] == 0:
                        segment_idx = len(segment_cumsum) - 1
                    else:
                        segment_idx = np.where(segment_cumsum >= delta)[0][0]
                    dis_delta_segment = delta - segment_cumsum[segment_idx - 1]
                    if (delta - segment_cumsum[segment_idx - 1]) > (segment_cumsum[segment_idx] - segment_cumsum[segment_idx-1]):
                        dis_delta_segment = (segment_cumsum[segment_idx] - segment_cumsum[segment_idx-1]) * 0.8
                                     
                    p_split = move_alone_line(traj_uavs[i][len(traj_uavs[i]) - segment_idx], traj_uavs[i][len(traj_uavs[i]) - segment_idx - 1], dis_delta_segment)

                    # 注意！这里删掉了原来的split_point
                    traj_uavs[i+1] = [p_split] + [traj_uavs[i][j] for j in range(len(traj_uavs[i]) - segment_idx, len(traj_uavs[i]) - 1)] + traj_uavs[i+1][1:]
                    traj_uavs[i] = [traj_uavs[i][j] for j in range(0, len(traj_uavs[i]) - segment_idx)] + [p_split]

                # Update dis Info 
                for idx in range(i, i+2):
                    uav_id = idx_2_uavid[idx]
                    uav = self.SharedInf.uav_dict[uav_id]
                    p1 = traj_uavs[idx][0]
                    p2 = traj_uavs[idx][-1]
                    travel_dis[idx] = (min(norm(p1 - uav.pos[:2]), norm(p2 - uav.pos[:2]))) # 动态决定uav的start point
                    search_dis[idx] = (np.sum([norm(traj_uavs[idx][j] - traj_uavs[idx][j+1]) for j in range(len(traj_uavs[idx]) - 1)]))
                    total_dis[idx] = travel_dis[idx] + search_dis[idx]

            iter_count += 1            
            # print(f"[Iter Info] Iter Count: {iter_count}, Dis: {abs(total_dis[0] - total_dis[-1])}\n\tTravel_dis: {travel_dis}\n\tSearch_dis: {search_dis}\n\tTotal_dis: {total_dis}")
            # print(f"[Iter Info] Iter Count: {iter_count}, Total Total Dis: {sum(total_dis)}, Dis: {abs(total_dis[0] - total_dis[-1])}\n\tTotal_dis: {total_dis}")
            if iter_count >= 150:
                break
            
            # 比较0#uav和-1#uav之间的距离差是否小于阈值
            # if abs(total_dis[0] - total_dis[-1]) < dis_threshold:
            #     break

            dis_list = [abs(total_dis[i%len(total_dis)] - total_dis[(i+1)%len(total_dis)]) for i in range(len(total_dis)+1)]
            if max(dis_list) < dis_threshold:
                break

        for uav_id, uav in self.SharedInf.uav_dict.items():
            idx = uavid_2_idx[uav_id]
            p1 = traj_uavs[idx][0]
            p2 = traj_uavs[idx][-1]
            if norm(p1 - uav.pos[:2]) > norm(p2 - uav.pos[:2]):
                traj_uavs_temp = traj_uavs[idx][::-1]
            else:
                traj_uavs_temp = traj_uavs[idx]
            uav.start_point = traj_uavs_temp[0]
            uav.travel_traj = [np.array(uav.pos[:2]), uav.start_point]
            uav.search_traj = traj_uavs_temp[1:]


    def init_traj_assign(self):
        self.uav_num = len(self.SharedInf.uav_dict)
        task = self.SharedInf.plain
        cost = np.zeros((self.uav_num, self.uav_num))
        for i, uav in enumerate(self.SharedInf.uav_dict.values()):
            for j, traj in enumerate(task.traj_uavs.values()):
                dis1 = np.linalg.norm(uav.pos[:2] - traj[0])
                dis2 = np.linalg.norm(uav.pos[:2] - traj[-1])
                cost[i,j] = dis1 if dis1 <= dis2 else dis2
        
        _, traj_idx = linear_sum_assignment(cost)
        idx_2_uavid, uavid_2_idx = {}, {}
        for i, j in zip(list(self.SharedInf.uav_dict.keys()), traj_idx):
            uavid_2_idx[i] = j
            idx_2_uavid[j] = i
        
        return idx_2_uavid, uavid_2_idx
        

    def debug_display(self, travel_traj=True):
        import matplotlib.pyplot as plt
        from matplotlib import patches, cm
        plt.figure(figsize=(8,8))
        ax = plt.subplot(1,1,1)

        uav_config:Dict = self.SharedInf.world_config["robots"]["UAV"]
        plain = self.SharedInf.plain

        ax.add_patch(patches.Rectangle((uav_config["x"], uav_config["y"]), 100, 100, fill=False))

        ax.add_patch(patches.Rectangle(plain.anchor, plain.width, plain.length, color="grey", alpha=0.3))
        
        rb8 = cm.get_cmap('rainbow', 8)
        # target_pos = [target.pos for target in self.target_list]
        # ax.scatter(np.array(target_pos)[:,0], np.array(target_pos)[:,1], marker='d', color="r")

        for i, uav in self.SharedInf.uav_dict.items():
            traj = np.array(uav.search_traj)
            if travel_traj:
                ax.plot([uav.pos[0], uav.start_point[0]], [uav.pos[1], uav.start_point[1]], color=rb8(i%8))
            ax.scatter(uav.start_point[0],uav.start_point[1], marker='o', color=rb8(i%8))
            ax.plot([uav.start_point[0], traj[0][0]], [uav.start_point[1], traj[0][1]], color=rb8(i%8))
            ax.plot(traj[:,0], traj[:,1], marker='x', color=rb8(i%8))

        map_config = self.SharedInf.world_config["map"]
        ax.set_xlim((0,map_config["width"]))
        ax.set_ylim((0,map_config["length"]))
        # plt.show()

    def debug_display_polygon(self):
        import matplotlib.pyplot as plt
        from matplotlib import patches, cm
        plt.figure(figsize=(8,8))
        ax = plt.subplot(1,1,1)
        plt.tight_layout()  # 放在创建ax之后
        # 设置边界范围
        plain:PolygonRegion = self.SharedInf.plain  # [(0.0, 0.0), (-101.08728450689489, -18.255704826644262), (-187.93016793916104, 73.48031196475847), (-162.542976061968, 158.9895444600783), (-8.293250285281932, 164.44864190284252)]
        ux = [
            uav.pos[0]
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        uy = [
            uav.pos[1]
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        tx = [
            uav.task.key_points[:,0].tolist() # 注意是个列表
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        ty = [
            uav.task.key_points[:,1].tolist() # 注意是个列表
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        ntx = []
        nty = []
        for i in tx:
            for j in i:
                ntx.append(j)
        for i in ty:
            for j in i:
                nty.append(j)
        xmin = min(min(np.array(plain.vertices)[:,0]),min(ux),np.min(ntx))
        ymin = min(min(np.array(plain.vertices)[:,1]),min(uy),np.min(nty))
        xmax = max(max(np.array(plain.vertices)[:,0]),max(ux),np.max(ntx))
        ymax = max(max(np.array(plain.vertices)[:,1]),max(uy),np.max(nty))
        # xmin = min(min(np.array(plain.vertices)[:,0]),min(ux))
        # ymin = min(min(np.array(plain.vertices)[:,1]),min(uy))
        # xmax = max(max(np.array(plain.vertices)[:,0]),max(ux))
        # ymax = max(max(np.array(plain.vertices)[:,1]),max(uy))
        ax.set_xlim([xmin, xmax])
        ax.set_ylim([ymin, ymax])
        # ax.axis('equal')
        # 自动调整图形的宽高比，确保 x 和 y 轴的单位刻度相同
        ax.set_aspect('equal') # 包括外边的边框也保持刻度不变，
        polygon = patches.Polygon(plain.vertices, closed = True, fill = False)
        ax.add_artist(polygon)  # 画出多边形区域
        rb8 = cm.get_cmap('rainbow', 8)  # 从 紫色 开始  # 红、橙、黄、绿、青、蓝、靛、紫
        # target_pos = [target.pos for target in self.target_list]
        # ax.scatter(np.array(target_pos)[:,0], np.array(target_pos)[:,1], marker='d', color="r")

        for i, uav in self.SharedInf.uav_dict.items():
            traj = np.array(uav.search_traj)  # 只有欧索点 # (12, 3) # [[321.4740742630289, 280.3685185657572, 30.0], [141.10555569727168, 100.0, 30.0], [125.54920651116764, 100.0, 30.0], [300.73227534822354, 275.1830688370559, 30.0], [279.9904764334181, 269.9976191083545, 30.0], [109.9928573250636, 100.0, 30.0], [100.0, 105.56349186104046, 30.0], [259.24867751861274, 264.8121693796532, 30.0], [238.50687860380734, 259.6267196509518, 30.0], [100.0, 121.11984104714449, 30.0], [100.0, 136.67619023324852, 30.0], [217.76507968900196, 254.44126992225048, 30.0]]  ############### [[-1.862655543808662, 36.93499701266063], [-2.714981840942367, 53.83595830047633], [-104.21397945264731, 161.0538831460993], [-89.5578615685133, 161.57258212763242], [-3.567308138076072, 70.73691958829203], [-4.419634435209777, 87.63788087610772], [-74.90174368437928, 162.09128110916552], [-60.24562580024528, 162.6099800906986], [-5.271960732343482, 104.5388421639234]]
            traj = np.array(uav.task.key_points) # 多一个返回点 # (13, 3) # [[321.4740742630289, 280.3685185657572, 30.0], [141.10555569727168, 100.0, 30.0], [125.54920651116764, 100.0, 30.0], [300.73227534822354, 275.1830688370559, 30.0], [279.9904764334181, 269.9976191083545, 30.0], [109.9928573250636, 100.0, 30.0], [100.0, 105.56349186104046, 30.0], [259.24867751861274, 264.8121693796532, 30.0], [238.50687860380734, 259.6267196509518, 30.0], [100.0, 121.11984104714449, 30.0], [100.0, 136.67619023324852, 30.0], [217.76507968900196, 254.44126992225048, 30.0], [313.3648234926275, 474.88038825386116, 30.0]] ############### [[-1.862655543808662, 36.93499701266063], [-2.714981840942367, 53.83595830047633], [-104.21397945264731, 161.0538831460993], [-89.5578615685133, 161.57258212763242], [-3.567308138076072, 70.73691958829203], [-4.419634435209777, 87.63788087610772], [-74.90174368437928, 162.09128110916552], [-60.24562580024528, 162.6099800906986], [-5.271960732343482, 104.5388421639234]]
            ax.plot([uav.pos[0], uav.start_point[0]], [uav.pos[1], uav.start_point[1]], color=rb8(i%8), alpha=0.4) # 无人机当前位置--> 扫描区域内第一个起点  # [313.364834926275, 474.88038825386116, 30.0] --> [321.4740742630289, 280.3685185657572, 30.0]
            ax.scatter(uav.start_point[0],uav.start_point[1], marker='s', color=rb8(i%8), alpha=0.4)
            ax.plot([uav.start_point[0], traj[0][0]], [uav.start_point[1], traj[0][1]], color=rb8(i%8), alpha=0.4)  #  扫描区域内第一个起点  # [321.4740742630289, 280.3685185657572, 30.0]
            ax.plot(traj[:,0], traj[:,1], marker='x', color=rb8(i%8), alpha=0.4)  # 航迹: start --> 返航回家
            
            y_offset = 0  # 偏移量，可以根据具体情况调整
            font_size = 24  # 字体大小，可以根据具体情况调整

            # ax.text(uav.pos[0],uav.pos[1], s=f'0', ha='center', va='center', color=rb8(i%8), alpha=0.8, fontsize=font_size)
            ax.text(uav.pos[0],uav.pos[1], s=f'0', ha='center', va='center', color='k', alpha=0.8, fontsize=font_size)
            # ax.text(uav.start_point[0],uav.start_point[1], s=f'1', ha='center', va='center')
            for k in range(len(traj)):
                # ax.plot(traj[i, 0], traj[i, 1], marker='x', color='black')
                ax.text(traj[k, 0], traj[k, 1], s=f'{k+1}', ha='center', va='center', color=rb8(i%8), alpha=0.8, fontsize=font_size)


        # ax.set_xlim((0,100))
        # ax.set_ylim((0,100))
        # plt.show()
        self.save_init_planner(plt)

    def new_keypoint_debug_display_polygon(self):
        import matplotlib.pyplot as plt
        from matplotlib import patches, cm
        plt.figure(figsize=(8,8))
        ax = plt.subplot(1,1,1)
        plt.tight_layout()  # 放在创建ax之后
        # 设置边界范围
        plain:PolygonRegion = self.SharedInf.plain  # [(0.0, 0.0), (-101.08728450689489, -18.255704826644262), (-187.93016793916104, 73.48031196475847), (-162.542976061968, 158.9895444600783), (-8.293250285281932, 164.44864190284252)]
        ux = [
            uav.pos[0]
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        uy = [
            uav.pos[1]
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        tx = [
            uav.task.key_points[:,0].tolist() # 注意是个列表
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        ty = [
            uav.task.key_points[:,1].tolist() # 注意是个列表
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        ntx = []
        nty = []
        for i in tx:
            for j in i:
                ntx.append(j)
        for i in ty:
            for j in i:
                nty.append(j)
        xmin = min(min(np.array(plain.vertices)[:,0]),min(ux),np.min(ntx))
        ymin = min(min(np.array(plain.vertices)[:,1]),min(uy),np.min(nty))
        xmax = max(max(np.array(plain.vertices)[:,0]),max(ux),np.max(ntx))
        ymax = max(max(np.array(plain.vertices)[:,1]),max(uy),np.max(nty))
        # xmin = min(min(np.array(plain.vertices)[:,0]),min(ux))
        # ymin = min(min(np.array(plain.vertices)[:,1]),min(uy))
        # xmax = max(max(np.array(plain.vertices)[:,0]),max(ux))
        # ymax = max(max(np.array(plain.vertices)[:,1]),max(uy))
        ax.set_xlim([xmin, xmax])
        ax.set_ylim([ymin, ymax])
        # ax.axis('equal')
        # 自动调整图形的宽高比，确保 x 和 y 轴的单位刻度相同
        ax.set_aspect('equal') # 包括外边的边框也保持刻度不变，
        polygon = patches.Polygon(plain.vertices, closed = True, fill = False)
        ax.add_artist(polygon)  # 画出多边形区域
        rb8 = cm.get_cmap('rainbow', 8)  # 从 紫色 开始  # 红、橙、黄、绿、青、蓝、靛、紫
        # target_pos = [target.pos for target in self.target_list]
        # ax.scatter(np.array(target_pos)[:,0], np.array(target_pos)[:,1], marker='d', color="r")

        for i, uav in self.SharedInf.uav_dict.items():
            # traj = np.array(uav.search_traj)  # 只有欧索点 # (12, 3) # [[321.4740742630289, 280.3685185657572, 30.0], [141.10555569727168, 100.0, 30.0], [125.54920651116764, 100.0, 30.0], [300.73227534822354, 275.1830688370559, 30.0], [279.9904764334181, 269.9976191083545, 30.0], [109.9928573250636, 100.0, 30.0], [100.0, 105.56349186104046, 30.0], [259.24867751861274, 264.8121693796532, 30.0], [238.50687860380734, 259.6267196509518, 30.0], [100.0, 121.11984104714449, 30.0], [100.0, 136.67619023324852, 30.0], [217.76507968900196, 254.44126992225048, 30.0]]  ############### [[-1.862655543808662, 36.93499701266063], [-2.714981840942367, 53.83595830047633], [-104.21397945264731, 161.0538831460993], [-89.5578615685133, 161.57258212763242], [-3.567308138076072, 70.73691958829203], [-4.419634435209777, 87.63788087610772], [-74.90174368437928, 162.09128110916552], [-60.24562580024528, 162.6099800906986], [-5.271960732343482, 104.5388421639234]]
            key_point_traj = np.array(uav.task.key_points) # 多一个返回点 # (13, 3) # [[321.4740742630289, 280.3685185657572, 30.0], [141.10555569727168, 100.0, 30.0], [125.54920651116764, 100.0, 30.0], [300.73227534822354, 275.1830688370559, 30.0], [279.9904764334181, 269.9976191083545, 30.0], [109.9928573250636, 100.0, 30.0], [100.0, 105.56349186104046, 30.0], [259.24867751861274, 264.8121693796532, 30.0], [238.50687860380734, 259.6267196509518, 30.0], [100.0, 121.11984104714449, 30.0], [100.0, 136.67619023324852, 30.0], [217.76507968900196, 254.44126992225048, 30.0], [313.3648234926275, 474.88038825386116, 30.0]] ############### [[-1.862655543808662, 36.93499701266063], [-2.714981840942367, 53.83595830047633], [-104.21397945264731, 161.0538831460993], [-89.5578615685133, 161.57258212763242], [-3.567308138076072, 70.73691958829203], [-4.419634435209777, 87.63788087610772], [-74.90174368437928, 162.09128110916552], [-60.24562580024528, 162.6099800906986], [-5.271960732343482, 104.5388421639234]]
            print('key_point_traj')
            print(key_point_traj)
            # ax.plot([uav.pos[0], uav.start_point[0]], [uav.pos[1], uav.start_point[1]], color=rb8(i%8), alpha=0.4) # 无人机当前位置--> 扫描区域内第一个起点  # [313.364834926275, 474.88038825386116, 30.0] --> [321.4740742630289, 280.3685185657572, 30.0]
            ax.plot([uav.pos[0], key_point_traj[0,0]], [uav.pos[1], key_point_traj[0,1]], color=rb8(i%8), alpha=0.4) # 无人机当前位置--> 扫描区域内第一个起点  # [313.364834926275, 474.88038825386116, 30.0] --> [321.4740742630289, 280.3685185657572, 30.0]
            # ax.scatter(uav.start_point[0],uav.start_point[1], marker='s', color=rb8(i%8), alpha=0.4)
            ax.scatter(key_point_traj[0,0],key_point_traj[0,1], marker='s', color=rb8(i%8), alpha=0.4)
            # ax.plot([uav.start_point[0], key_point_traj[0][0]], [uav.start_point[1], key_point_traj[0][1]], color=rb8(i%8), alpha=0.4)  #  扫描区域内第一个起点  # [321.4740742630289, 280.3685185657572, 30.0]
            ax.plot(key_point_traj[:,0], key_point_traj[:,1], marker='x', color=rb8(i%8), alpha=0.4)  # 航迹: start --> 返航回家
            
            y_offset = 0  # 偏移量，可以根据具体情况调整
            font_size = 24  # 字体大小，可以根据具体情况调整

            # ax.text(uav.pos[0],uav.pos[1], s=f'0', ha='center', va='center', color=rb8(i%8), alpha=0.8, fontsize=font_size)
            ax.text(uav.pos[0],uav.pos[1], s=f'0', ha='center', va='center', color='k', alpha=0.8, fontsize=font_size)
            # ax.text(uav.start_point[0],uav.start_point[1], s=f'1', ha='center', va='center')
            for k in range(len(key_point_traj)):
                # ax.plot(traj[i, 0], traj[i, 1], marker='x', color='black')
                ax.text(key_point_traj[k, 0], key_point_traj[k, 1], s=f'{k+1}', ha='center', va='center', color=rb8(i%8), alpha=0.8, fontsize=font_size)


        # ax.set_xlim((0,100))
        # ax.set_ylim((0,100))
        # plt.show()
        self.save_init_planner(plt)
    
    def save_init_planner(self, plt):
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
        fig_path =  file_name_without_extension + '/' + current_time + '.png'
        print('fig_path')
        print(fig_path)
        create_file(file_path=fig_path)
        plt.savefig(fig_path, dpi=60, bbox_inches='tight')
        plt.close()

    def construct_task(self):
        init_z = self.search_height  # z = height = 30
        for id, uav in self.SharedInf.uav_dict.items():
            uav.start_point = np.hstack((uav.start_point, np.array(init_z)))  # 1个xyz  # [382.77817459305203, 248.33452377915614, 30.0]
            uav.travel_traj = np.hstack((np.array(uav.travel_traj), init_z*np.ones((np.array(uav.travel_traj).shape[0],1))))  # 2个xyz  # [[327.1320643266746, 402.07519493594015, 30.0], [382.77817459305203, 248.33452377915614, 30.0]]
            uav.search_traj = np.hstack((np.array(uav.search_traj), init_z*np.ones((np.array(uav.search_traj).shape[0],1))))  # 13个xyz 第一个无人机多一段，12段 # [[382.77817459305203, 248.33452377915614, 30.0], [268.8873016277919, 134.44365081389594, 30.0], [237.7746032555838, 118.8873016277919, 30.0], [390.55634918610406, 271.6690475583123, 30.0], [398.3345237791561, 295.0035713374682, 30.0], [206.6619048833757, 103.33095244168786, 30.0], [187.77460325558383, 100.0, 30.0], [383.6994710074451, 295.92486775186126, 30.0], [362.9576720926397, 290.73941802315994, 30.0], [172.21825406947977, 100.0, 30.0], [156.6619048833757, 100.0, 30.0], [342.2158731778343, 285.55396829445857, 30.0], [321.4740742630289, 280.3685185657572, 30.0]]  ###### 12个xyz 第二个无人机没有多一段 12段 # [[321.4740742630289, 280.3685185657572, 30.0], [141.10555569727168, 100.0, 30.0], [125.54920651116764, 100.0, 30.0], [300.73227534822354, 275.1830688370559, 30.0], [279.9904764334181, 269.9976191083545, 30.0], [109.9928573250636, 100.0, 30.0], [100.0, 105.56349186104046, 30.0], [259.24867751861274, 264.8121693796532, 30.0], [238.50687860380734, 259.6267196509518, 30.0], [100.0, 121.11984104714449, 30.0], [100.0, 136.67619023324852, 30.0], [217.76507968900196, 254.44126992225048, 30.0]]
            init_pos        = np.hstack((uav.init_pos[:2], np.array(init_z))) # 1个xyz # [327.1320643266746, 402.07519493594015, 30.0] # 有时候不一样  # uav.init_pos=[327.1320643266746, 402.07519493594015, 30.0] # 
            task_id = ("plain", id)
            # task = Task(task_id=task_id, init_assign=id, key_points=np.vstack([uav.start_point, uav.search_traj, uav.init_pos])) #  # 这个高度比设置的 高度高很多, 工程化预防在出发和返航的两段碰撞
            # task = Task(task_id=task_id, init_assign=id, key_points=np.vstack([uav.start_point, uav.search_traj]))  # 不要重复 1、2 两个点。 0是当前飞机起飞位置。
            # print('uav.init_pos')
            # print(uav.init_pos)
            # task = Task(task_id=task_id, init_assign=id, key_points=np.vstack([uav.init_pos, uav.search_traj, uav.init_pos]))  # 这个高度比设置的 高度高很多, 工程化预防在出发和返航的两段碰撞
            # task = Task(task_id=task_id, init_assign=id, key_points=np.vstack([uav.search_traj, init_pos]))  # 任务类型, 无人机id, -无旅行+搜索+返航, # 这个高度是 设置的高度 # <1>task_id,类型元组uav_id <2>init_assign,分配的uav_id <3>key_points,扫描点+返航点 <4>cur_index,当前要去的航点，还没有完成 <5>total_index,扫描点+返航点一共有多少 <6>finished,完成任务 <7>assign_id,未分配 #
            task = Task(task_id=task_id, init_assign=id, key_points=np.vstack([uav.search_traj]))  # 任务类型, 无人机id, -无旅行+搜索+返航, # 这个高度是 设置的高度 # <1>task_id,类型元组uav_id <2>init_assign,分配的uav_id <3>key_points,扫描点+返航点 <4>cur_index,当前要去的航点，还没有完成 <5>total_index,扫描点+返航点一共有多少 <6>finished,完成任务 <7>assign_id,未分配 #
            # if len(uav.search_traj)<=2: # len==1
            #     task = Task(task_id=task_id, init_assign=id, key_points=np.vstack([uav.search_traj, init_pos]))  # 去掉终点线的两个线段
            # else:
            #     task = Task(task_id=task_id, init_assign=id, key_points=np.vstack([uav.search_traj[:-2], init_pos]))  # 去掉终点线的两个线段
            self.SharedInf.task_dict[task_id] = task # 记录 平原任务(多个区域) 对应的 详情信息 robotR.Task # SharedInf.task_dict == {('plain', 0): <src.robotR.Task obj...F27A008B0>} 
            uav.task = task  # uav 共享  SharedInf.task_dict[
            task.assign_id = id # uav_id 0
            pass

    def task_offset(self):
        return # 因为uav的探测范围是个圆形，他初始的搜索位置不一定要在边界线上  # 我刚看了一下，这个offset函数写的非常简单，只适用于“矩形的边与坐标轴平行的情况”，如果矩形有旋转，是用不了的
        offset = self.offset
        for task in self.SharedInf.task_dict.values():
            max_x = task.key_points.max(axis=0)[0]
            min_x = task.key_points.min(axis=0)[0]
            for idx, p in enumerate(task.key_points):
                if p[0] == max_x:
                    task.key_points[idx] = p + np.array([offset, 0 , 0])
                if p[0] == min_x:
                    task.key_points[idx] = p - np.array([offset, 0 , 0])

class OnlinePlanner(PlannerBase):
    _version = 1
    def __init__(self, SharedInf) -> None:
        super().__init__(SharedInf)
        self.attack_flag = bool(self.SharedInf.args["attack"])  # 1 --> true

    def plan(self):
        # 1 assign target for uav
        for task_id, task in self.SharedInf.task_dict.items():  # self.SharedInf.task_dict == {('plain', 26): <src.robotR.Task object at 0x000001BE5446C6A0>, ('plain', 27): <src.robotR.Task object at 0x000001BE5446F430>}
            if task_id[0] == "target" and task.assign_id == -1 and task.finished == False:
                if not self.attack_flag:
                    task.finished == True
                    continue
                dis_dict = {uav.id:norm(uav.pos - task.target_pos) for uav in self.SharedInf.uav_dict.values() if uav.active == True}
                dis_dict_sorted = sorted(dis_dict.items(), key=lambda x:x[1])
                for uav_id, _ in dis_dict_sorted:
                    uav = self.SharedInf.uav_dict[uav_id]
                    if uav.active != True:
                        continue
                    if uav.task is None:
                        uav.task = task
                        task.assign_id = uav_id
                        break
                    elif uav.task.task_id[0] != "target":
                        uav.task.unassign()
                        uav.task = task
                        task.assign_id = uav_id
                        break

        # 2 assign building to init_assign uav
        for task_id, task in self.SharedInf.task_dict.items():
            if task_id[0] == "building" and task.assign_id == -1 and task.finished == False:
                uav = self.SharedInf.uav_dict[task.init_assign]
                if uav.active != True:
                    continue
                if uav.task is None:
                    uav.task = task
                    task.assign_id = uav.id
                elif uav.task.task_id[0] != "target":
                    uav.task.unassign()
                    uav.task = task
                    task.assign_id = uav.id



        # 3 For all the unassigned uav

        # 把部分任务扔回任务池 FIXME if necessary?
        # for task_id, task in self.SharedInf.task_dict.items():
        #     if task_id[0] != "target" and task.assign_id != -1 and task.cur_index == 0 and task.finished == False:
        #         self.SharedInf.uav_dict[task.assign_id].task = None
        #         task.unassign()

        unassigned_uav_list:List[UAV] = []
        unassigned_task_list:List[Task] = []
        for uav in self.SharedInf.uav_dict.values():
            if uav.task is None and uav.active == True:  # zytTODO: 更新 uav.task 为 None
                unassigned_uav_list.append(uav)

        for task in self.SharedInf.task_dict.values():
            if task.assign_id == -1 and task.finished == False:
                unassigned_task_list.append(task)

        # 只有 打击阵亡 是 unassigned_task_list
        N, M = len(unassigned_uav_list), len(unassigned_task_list)  # 0, 1 # (1, 1) # (1, 0)
        cost = np.zeros((N, M))
        for i, uav in enumerate(unassigned_uav_list):
            for j, task in enumerate(unassigned_task_list):
                travel_dis = np.linalg.norm(uav.pos - task.target_pos)
                # cost[i,j] = travel_dis + task.total_distance
                cost[i,j] = travel_dis

        # 如下是两种 重分配的方式 ： 1.打击过后  2.分割任务
        # 1.打击过后
        # 可以分配的无人机==unassigned_uav_list  <  打击阵亡==unassigned_task_list
        if N <= M:  # # 0, 1  ## (1, 0) 2024-3-27 # (1, 0)
            uav_idx, task_idx = linear_sum_assignment(cost)
        else:
            task_idx, uav_idx = linear_sum_assignment(cost.T) # (array([], dtype=int64), array([], dtype=int64))

        for i,j in zip(uav_idx, task_idx): #直接把任务给帮忙的没任务无人机
            uav, task = unassigned_uav_list[i], unassigned_task_list[j]
            uav.task = task  # {'task_id': ('plain', 1), 'init_assign': 1, 'key_points': array([[-56.39009774,   0.68102275,   6.        ],       [-20.19836008,  68.06279052,   6.        ],       [-19.39042567,  65.3402789 ,   6.        ],       [-54.13449383,   0.65378184,   6.        ],       [-51.87888992,   0.62654093,   6.        ],       [-18.58249127,  62.61776728,   6.        ],       [-17.77455687,  59.89525566,   6.        ],       [-49.62328601,   0.59930002,   6.        ],       [-47.3676821 ,   0.57205911,   6.        ],       [-16.96662246,  57.17274404,   6.        ],       [-16.15868806,  54.45023242,   6.        ],       [-45.11207819,   0.5448182 ,   6.        ],       [-42.85647428,   0.51757729,   6.        ],       [-15.35075366,  51.7277208 ,   6.        ],       [-14.54281926,  49.00520918,   6.        ],       [-40.60087037,   0.49033638,   6.        ],       [-38.34526646,   0.46309547,   6.        ],       [-13.73488485,  46.28269756,   6.        ],       [-12.92695045,  43.56018594,   6.        ],       [-36.08966255,   0.43585456,   6.        ],       [-33.83405864,   0.40861365,   6.        ],       [-12.11901605,  40.83767431,   6.        ],       [-11.31108164,  38.11516269,   6.        ],       [-31.57845473,   0.38137274,   6.        ],       [-29.32285082,   0.35413183,   6.        ],       [-10.50314724,  35.39265107,   6.        ],       [ -9.69521284,  32.67013945,   6.        ],       [-27.06724691,   0.32689092,   6.        ]]), 'cur_index': 0, 'total_index': 28, 'finished': False, 'assign_id': 10, 'finished_point_ID': 0, 'uav_data': {'fly_status': 0, 'order': 1, 'task_status': 5, 'scout_end': 0, 'finished_point_ID': 0, 'lon': 8.5450487, 'lat': 47.3973261, 'alt': 2.603, 'break_lon': 8.5450006, 'break_lat': 47.3973184, 'break_alt': 11.0}}
            # self.SharedInf.uav_dict[task.assign_id].task = None
            task.assign_id = uav.id
            # task.update(uav.pos)  # zytTODO: 不需要更新位置

        # # ################ 分配  不好可能会引起   任务冲突
        # # 2.分割任务
        # # 还有 空闲  uav 可以分配任务 ， 去就切分任务分给 这个空闲无人机
        # if N > M: # 还有uav 可以分配任务 # 可以暂时不考虑 # (1, 0)  2024-3-27 现在 考虑 # 将 未完成的任务 继续分割
        #     second_round_uav_list:List[UAV] = []  # 第二轮无人机名单
        #     unfinished_task_list:List[Task] = []  # 未完成任务清单  可以二次拆分
        #     for uav in self.SharedInf.uav_dict.values():
        #         if uav.task is None and uav.active == True: # {'id': 10, 'pos': array([26.50384003, 12.99502138, 14.962     ]), 'SharedInf': <src.robotR.SharedInformation object at 0x000001C4DE6B78E0>, 'active': True, 'init_pos': array([ 2.08888954e+01, -6.96113502e+01, -1.10000000e-02]), 'yaw': 0, 'pitch': 0, 'roll': 0, 'velocity': 20, 'dt': 0.1, 'fly_high': 15, 'scout_range': 2.988254109742198, 'offset': 9.611354205699502, 'start_point': array([ 5.71820432,  1.76451477, 15.        ]), 'task': None}  ########### {'id': 1, 'pos': array([-31.18409377,  10.01451892,  -0.268     ]), 'SharedInf': <src.robotR.SharedInformation object at 0x000001C4DE6B78E0>, 'active': True, 'init_pos': array([-7.48610947e+01, -6.62705440e+01,  2.70000000e-02]), 'yaw': 0, 'pitch': 0, 'roll': 0, 'velocity': 20, 'dt': 0.1, 'fly_high': 15, 'scout_range': 2.988254109742198, 'offset': 9.611354205699502, 'start_point': array([-26.95456424, -24.9331317 ,  15.        ]), 'task': <src.robotR.Task object at 0x000001C4E4B4F0A0>}
        #             second_round_uav_list.append(uav)  #  没有任务的  无人机
            
        #     # FIXME update unfinished task selection conditions  # 
        #     for task in self.SharedInf.task_dict.values():  # task_dict == {('plain', 10): <src.robotR.Task object at 0x000001C4E4B4F070>, ('plain', 1): <src.robotR.Task object at 0x000001C4E4B4F0A0>, ('plain', 17): <src.robotR.Task object at 0x000001C4E4B4F040>}
        #         # if task.finished == False and (task.total_index - task.cur_index > 1) and (task.task_id[1] < 21):
        #         if task.finished == False and (task.total_index - task.cur_index > 1) : # LQY建议 >= 2 # {'task_id': ('plain', 10), 'init_assign': 10, 'key_points': array([[  5.71820432,   1.76451477,  15.        ],       [ 11.91899132,   3.67794416,  15.        ],       [ 21.93082599,  27.90588076,  15.        ],       [ 24.79433477,  21.74333129,  15.        ],       [ 18.11977833,   5.59137356,  15.        ],       [ 24.32056534,   7.50480296,  15.        ],       [ 27.65784356,  15.58078182,  15.        ],       [ 20.88889537, -69.61135018,  15.        ]]), 'cur_index': 0, 'total_index': 8, 'finished': True, 'assign_id': -1, 'finished_point_ID': 7, 'uav_data': {'fly_status': 1, 'order': 10, 'task_status': 0, 'scout_end': 1, 'finished_point_ID': 7, 'lon': 8.5460517, 'lat': 47.398229, 'alt': 14.962, 'break_lon': 0.0, 'break_lat': 0.0, 'break_alt': 0.0}, 'cur_pos': array([26.50384003, 12.99502138, 14.962     ])}                    ##########             {'task_id': ('plain', 1), 'init_assign': 1, 'key_points': array([[-26.95456424, -24.9331317 ,  15.        ],       [-22.30928755, -26.78392799,  15.        ],       [  4.59590583,  38.32475   ,  15.        ],       [ 10.14206819,  38.65403374,  15.        ],       [-17.66401085, -28.63472428,  15.        ],       [-13.01873416, -30.48552057,  15.        ],       [ 15.68823055,  38.98331749,  15.        ],       [ 19.0673172 ,  34.06843023,  15.        ],       [  5.71820432,   1.76451477,  15.        ],       [-74.86109472, -66.27054396,  15.        ]]), 'cur_index': 0, 'total_index': 10, 'finished': False, 'assign_id': 1, 'finished_point_ID': 2, 'uav_data': {'fly_status': 1, 'order': 1, 'task_status': 3, 'scout_end': 0, 'finished_point_ID': 2, 'lon': 8.5460121, 'lat': 47.3977102, 'alt': -0.268, 'break_lon': 0.0, 'break_lat': 0.0, 'break_alt': 0.0}, 'cur_pos': array([-31.18409377,  10.01451892,  -0.268     ])}               ##########               {'task_id': ('plain', 17), 'init_assign': 17, 'key_points': array([[-4.01961484e+01,  8.48336272e+00,  1.50000000e+01],       [-3.58141359e+01,  1.90875251e+01,  1.50000000e+01],       [-2.68448071e+01,  2.77006311e+01,  1.50000000e+01],       [-4.08330999e+01, -6.15005475e+00,  1.50000000e+01],       [-4.08903943e+01, -1.93807428e+01,  1.50000000e+01],       [-1.78754783e+01,  3.63137372e+01,  1.50000000e+01],       [-1.20425813e+01,  3.73368988e+01,  1.50000000e+01],       [-3.62451176e+01, -2.12315391e+01,  1.50000000e+01],       [-3.15998409e+01, -2.30823354e+01,  1.50000000e+01],       [-6.49641890e+00,  3.76661825e+01,  1.50000000e+01],       [-9.50256536e-01,  3.79954663e+01,  1.50000000e+01],       [-2.69545642e+01, -2.49331317e+01,  1.50000000e+01],       [-5.22734513e+06, -1.06716500e+06,  1.50000000e+01]]), 'cur_index': 0, 'total_index': 13, 'finished': False, 'assign_id': 17, 'finished_point_ID': 0, 'uav_data': {'fly_status': 1, 'order': 17, 'task_status': 0, 'scout_end': 0, 'finished_point_ID': 0, 'lon': 0.0, 'lat': 0.0, 'alt': 0.0, 'break_lon': 0.0, 'break_lat': 0.0, 'break_alt': 0.0}, 'cur_pos': array([-5227345.13053935, -1067165.00000359,        0.        ])}
        #             if task.cur_index == 0:
        #                 if task.total_index > 2:  # 0 1 2 三个点 可以分
        #                     unfinished_task_list.append(task)
        #             else:
        #                 unfinished_task_list.append(task) # len==4==5初始总数-1完成
            
        #     # 穷举拆分task，再用一次匈牙利算法（每个task最多两个uav来执行）
        #     K = len(second_round_uav_list)  # [<src.robotR.UAV object at 0x0000015FE7F504F0>]
        #     L = len(unfinished_task_list)   # [<src.robotR.Task object at 0x0000015FE4EB3EE0>,       <src.robotR.Task object at 0x0000015FE4EB3EB0>]
        #     cost = np.zeros((K,L)) # array([[0., 0.]])  # array([[-1.96347343e+02, -5.33527039e+06]])  # test7 array([[0., 0., 0., 0.]]) array([[0., 0., 0.]])
            
        #     for i, uav in enumerate(second_round_uav_list):
        #         for j, task in enumerate(unfinished_task_list):
        #             split_distance = task.split_dis(uav.pos) # 考虑了travel和key的分割， 让任务均衡  # 222.38119304261608  # array([26.50384003, 12.99502138, 14.962     ])
        #             cost[i,j] = -(task.total_distance - split_distance)  # 函数是min  #我们要max 就加负号
            
        #     uav_idx, task_idx = linear_sum_assignment(cost) # (array([0], dtype=int64), array([1], dtype=int64))
        #     for i,j in zip(uav_idx, task_idx): # i,j==(0, 1) # test7  (0, 3)
        #         uav:UAV = unassigned_uav_list[i]  # uav.id == 10 # <src.robotR.UAV object at 0x000002CE43EBC520>
        #         original_task:Task = unfinished_task_list[j] # <src.robotR.Task object at 0x000002CE40E1FEE0>  #  original_task.task_id == ('plain', 17) # 这个任务要被分割 成为 两个
        #         # original_task.changed = True
        #         split_task = original_task.split_task(uav.pos) # 被分割出来的新任务 task 后半截key  # uav.pos == array([26.50384003, 12.99502138, 14.962     ])
                
        #         task_type = original_task.task_id[0]
        #         task_jdx = 0
        #         max_task_idx = -1 # max_task_idx == 17
        #         for task_id in self.SharedInf.task_dict.keys():
        #             if task_id[0] == task_type and task_id[1] > max_task_idx:
        #                 max_task_idx = task_id[1]
        #         task_jdx = max_task_idx + 1  # task_jdx == 18
                
        #         original_task.finished_point_ID = 0
        #         split_task.finished_point_ID = 0
                
        #         # 更新任务字典
        #         old_key = original_task.task_id
        #         original_task.task_id = (task_type, task_jdx+1)  # ('plain', 18)  # ('plain', 40) ('plain', 41) # 先 del 在加入新的 update
        #         split_task.task_id = (task_type, task_jdx)  # ('plain', 18)  # ('plain', 39) ('plain', 40)
        #         split_task.change_seq(uav.pos) # 如果距离队尾近， 则从队尾开始进入扫描  # uav.pos == array([26.50384003, 12.99502138, 14.962     ])
        #         self.SharedInf.task_dict.update({split_task.task_id:split_task}) # {('plain', 18): <src.robotR.Task object at 0x000002CE4B154850>}  # ('plain', 39)
        #         self.SharedInf.task_dict.update({original_task.task_id:original_task}) 
        #         del self.SharedInf.task_dict[old_key]
                
        #         # 为  待分配任务的无人机  ->  分配新拆分的任务
        #         uav.task = split_task # 10号机完成任务 # 将 18 号 任务 给 10 号
        #         split_task.init_assign = uav.id  # 10
        #         split_task.assign_id = uav.id # 10
                
        #         # judge if the last 2 points are on the egde of the polygon
        #         if type(original_task) == Task:
        #             self.remove_edge_search_part(original_task)
        #             self.remove_edge_search_part(split_task)
        #             split_task.change_seq(uav.pos)
        # ################

        # 没有任务 并且 活着  返航   ("back", uav.id)
        for uav in self.SharedInf.uav_dict.values():  # 主要是在做返航   ("back", uav.id)
            if uav.task is not None or uav.active != True: # uav 有任务 或者 挂了
                continue
            # uav.task is == None
            dis_dict = {task.task_id:norm(uav.pos - task.target_pos) for task in self.SharedInf.task_dict.values() if task.assign_id == -1 and task.finished == False}  # 任务没有完成 + 没有分配
            dis_dict_sorted = sorted(dis_dict.items(), key=lambda x:x[1])
            if dis_dict_sorted == []: # 没有任务给uav执行 or uav的任务完成
                self.add_back_task(uav)  # 先拉高，再飞回起飞点，再降落
                task_id=("back", uav.id)
                task = self.SharedInf.back_task_dict[task_id]
            else:
                task:Task = self.SharedInf.task_dict[dis_dict_sorted[0][0]]
            uav.task = task
            task.assign_id = uav.id
        
        # for debug
        unfinished_task = []   # task_id == ('plain', 26)  # init_assign: 26 # assign_id:27  # ('plain', 1)
        for task_id, task in self.SharedInf.task_dict.items():
            if task.finished == False:
                unfinished_task.append(task) # [<src.robotR.Task object at 0x000002CE40E1FF10>, <src.robotR.Task object at 0x000002CE40E1FEE0>, <src.robotR.Task object at 0x000002CE4B154850>]

        self.check_repeat_task_assign()
    
    def add_back_task(self, uav:UAV):
        
        # 先拉高，再飞回起飞点，再降落
        #  >>>>>>>>>>>
        # ^          v
        # ^          v
        # ^          v
        # ^          v

        task_id=("back", uav.id)
        if task_id not in self.SharedInf.back_task_dict.keys():
            basic_height, interval = 30, 5
            height = basic_height + len(self.SharedInf.back_task_dict) * interval
            k_pts = np.vstack(
                            (
                                np.hstack((uav.pos[:2],height)),
                                np.hstack((uav.init_pos[:2],height)),
                                # np.hstack((uav.init_pos[:2],1)),
                            )
                            ) # len==3 # [[-11.82890729878763, 30.388709321073218, 30.0], [23.05294805699005, -31.684806602670466, 30.0], [23.05294805699005, -31.684806602670466, 1.0]]
        else:
            k_pts = uav.pos.reshape(1,3)
        
        task = Task(task_id=task_id, init_assign=uav.id, key_points=k_pts) # add back task
        self.SharedInf.back_task_dict[task_id] = task  # ('back', 10)


    def remove_edge_search_part(self, task:Task):
        """remove the last point of the search part if the last 2(or first 2) points are on the same edge of the polygon
        """
        if len(task.key_points) < 3:
            return
        
        if type(task) == BuildingTask:
            return

        if self.if_on_same_edge(task.key_points[0], task.key_points[1]):
            task.key_points = task.key_points[1:]
            task.total_index = task.key_points.shape[0]  # 删除最后一个点 所以 总数改变
        if len(task.key_points) < 3:
            return
        if self.if_on_same_edge(task.key_points[-2], task.key_points[-1]):
            task.key_points = task.key_points[:-1]
            task.total_index = task.key_points.shape[0]  # 删除最后一个点 所以 总数改变
        

    def if_on_same_edge(self, p1, p2):
        on_which_edge = []
        for i in range(len(self.SharedInf.plain.vertices)):
            j = (i+1) % len(self.SharedInf.plain.vertices)
            line = shapely.LineString([self.SharedInf.plain.vertices[i], self.SharedInf.plain.vertices[j]])
            if line.distance(shapely.Point(p1)) < 0.001:
                on_which_edge.append(i)
            if line.distance(shapely.Point(p2)) < 0.001:
                on_which_edge.append(i)
        

        if len(on_which_edge) < 2:
            return False
        elif len(on_which_edge) >2 :
            print(f"on_which_edge: {on_which_edge}")
        
        if on_which_edge[0] == on_which_edge[1]:
            return True
        else:
            return False

    def debug_display_polygon(self):
        import matplotlib.pyplot as plt
        from matplotlib import patches, cm
        plt.figure(figsize=(8,8))
        ax = plt.subplot(1,1,1)
        plt.tight_layout()  # 放在创建ax之后
        # 设置边界范围
        plain:PolygonRegion = self.SharedInf.plain  # [(0.0, 0.0), (-101.08728450689489, -18.255704826644262), (-187.93016793916104, 73.48031196475847), (-162.542976061968, 158.9895444600783), (-8.293250285281932, 164.44864190284252)]
        ux = [
            uav.pos[0]
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        uy = [
            uav.pos[1]
            for i, uav in self.SharedInf.uav_dict.items()
        ]
        tx = [
            uav.task.key_points[:,0].tolist() # 注意是个列表
            for i, uav in self.SharedInf.uav_dict.items()
            if uav.task is not None
        ]
        ty = [
            uav.task.key_points[:,1].tolist() # 注意是个列表
            for i, uav in self.SharedInf.uav_dict.items()
            if uav.task is not None
        ]
        ntx = []
        nty = []
        for i in tx:
            for j in i:
                ntx.append(j)
        for i in ty:
            for j in i:
                nty.append(j)
        xmin = min(min(np.array(plain.vertices)[:,0]),min(ux),np.min(ntx))
        ymin = min(min(np.array(plain.vertices)[:,1]),min(uy),np.min(nty))
        xmax = max(max(np.array(plain.vertices)[:,0]),max(ux),np.max(ntx))
        ymax = max(max(np.array(plain.vertices)[:,1]),max(uy),np.max(nty))
        # xmin = min(min(np.array(plain.vertices)[:,0]),min(ux))
        # ymin = min(min(np.array(plain.vertices)[:,1]),min(uy))
        # xmax = max(max(np.array(plain.vertices)[:,0]),max(ux))
        # ymax = max(max(np.array(plain.vertices)[:,1]),max(uy))
        ax.set_xlim([xmin, xmax])
        ax.set_ylim([ymin, ymax])
        # ax.axis('equal')
        # 自动调整图形的宽高比，确保 x 和 y 轴的单位刻度相同
        ax.set_aspect('equal') # 包括外边的边框也保持刻度不变，
        polygon = patches.Polygon(plain.vertices, closed = True, fill = False)
        ax.add_artist(polygon)  # 画出多边形区域
        rb8 = cm.get_cmap('rainbow', 8)  # 从 紫色 开始  # 红、橙、黄、绿、青、蓝、靛、紫
        # target_pos = [target.pos for target in self.target_list]
        # ax.scatter(np.array(target_pos)[:,0], np.array(target_pos)[:,1], marker='d', color="r")

        for i, uav in self.SharedInf.uav_dict.items():
            if uav.task is None:
                print(i , ' uav.task is None ')
                continue
            else:
                print(i , ' uav.task is not None ')
            traj = np.array(uav.search_traj)  # 只有欧索点 # (12, 3) # [[321.4740742630289, 280.3685185657572, 30.0], [141.10555569727168, 100.0, 30.0], [125.54920651116764, 100.0, 30.0], [300.73227534822354, 275.1830688370559, 30.0], [279.9904764334181, 269.9976191083545, 30.0], [109.9928573250636, 100.0, 30.0], [100.0, 105.56349186104046, 30.0], [259.24867751861274, 264.8121693796532, 30.0], [238.50687860380734, 259.6267196509518, 30.0], [100.0, 121.11984104714449, 30.0], [100.0, 136.67619023324852, 30.0], [217.76507968900196, 254.44126992225048, 30.0]]  ############### [[-1.862655543808662, 36.93499701266063], [-2.714981840942367, 53.83595830047633], [-104.21397945264731, 161.0538831460993], [-89.5578615685133, 161.57258212763242], [-3.567308138076072, 70.73691958829203], [-4.419634435209777, 87.63788087610772], [-74.90174368437928, 162.09128110916552], [-60.24562580024528, 162.6099800906986], [-5.271960732343482, 104.5388421639234]]
            traj = np.array(uav.task.key_points) # 多一个返回点 # (13, 3) # [[321.4740742630289, 280.3685185657572, 30.0], [141.10555569727168, 100.0, 30.0], [125.54920651116764, 100.0, 30.0], [300.73227534822354, 275.1830688370559, 30.0], [279.9904764334181, 269.9976191083545, 30.0], [109.9928573250636, 100.0, 30.0], [100.0, 105.56349186104046, 30.0], [259.24867751861274, 264.8121693796532, 30.0], [238.50687860380734, 259.6267196509518, 30.0], [100.0, 121.11984104714449, 30.0], [100.0, 136.67619023324852, 30.0], [217.76507968900196, 254.44126992225048, 30.0], [313.3648234926275, 474.88038825386116, 30.0]] ############### [[-1.862655543808662, 36.93499701266063], [-2.714981840942367, 53.83595830047633], [-104.21397945264731, 161.0538831460993], [-89.5578615685133, 161.57258212763242], [-3.567308138076072, 70.73691958829203], [-4.419634435209777, 87.63788087610772], [-74.90174368437928, 162.09128110916552], [-60.24562580024528, 162.6099800906986], [-5.271960732343482, 104.5388421639234]]
            ax.plot([uav.pos[0], uav.start_point[0]], [uav.pos[1], uav.start_point[1]], color=rb8(i%8), alpha=0.4) # 无人机当前位置--> 扫描区域内第一个起点  # [313.364834926275, 474.88038825386116, 30.0] --> [321.4740742630289, 280.3685185657572, 30.0]
            ax.scatter(uav.start_point[0],uav.start_point[1], marker='s', color=rb8(i%8), alpha=0.4)
            ax.plot([uav.start_point[0], traj[0][0]], [uav.start_point[1], traj[0][1]], color=rb8(i%8), alpha=0.4)  #  扫描区域内第一个起点  # [321.4740742630289, 280.3685185657572, 30.0]
            ax.plot(traj[:,0], traj[:,1], marker='x', color=rb8(i%8), alpha=0.4)  # 航迹: start --> 返航回家
            
            y_offset = 0  # 偏移量，可以根据具体情况调整
            font_size = 24  # 字体大小，可以根据具体情况调整

            # ax.text(uav.pos[0],uav.pos[1], s=f'0', ha='center', va='center', color=rb8(i%8), alpha=0.8, fontsize=font_size)
            ax.text(uav.pos[0],uav.pos[1], s=f'0', ha='center', va='center', color='k', alpha=0.8, fontsize=font_size)
            # ax.text(uav.start_point[0],uav.start_point[1], s=f'1', ha='center', va='center')
            for k in range(len(traj)):
                # ax.plot(traj[i, 0], traj[i, 1], marker='x', color='black')
                ax.text(traj[k, 0], traj[k, 1], s=f'{k+1}', ha='center', va='center', color=rb8(i%8), alpha=0.8, fontsize=font_size)


        # ax.set_xlim((0,100))
        # ax.set_ylim((0,100))
        # plt.show()
        self.save_online_planner(plt)
    
    def save_online_planner(self, plt):
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
        fig_path =  file_name_without_extension + '/' + current_time + '.png'
        print('fig_path')
        print(fig_path)
        create_file(file_path=fig_path)
        plt.savefig(fig_path, dpi=600, bbox_inches='tight')
        plt.close()


    def get_max_distance(self):
        max_d = 0
        for task in self.SharedInf.task_dict.values():
            if task.total_distance > max_d:
                max_d = task.total_distance

    def check_repeat_task_assign(self):
        unique_task_id = [] # test7 [('plain', 21), ('plain', 38), ('plain', 39), ('plain', 1), ('plain', 40)]  
        for uav in self.SharedInf.uav_dict.values():
            if uav.active != True:
                if uav.task != None:
                    print('     check_repeat_task_assign     ')
                    print('     uav  故障损坏  但是  有任务     ')
                    return False # uav 故障 但是有任务
            else:
                if uav.task.task_id not in unique_task_id:
                    unique_task_id.append(uav.task.task_id)
                else:# [('plain', 21), ('plain', 41), ('plain', 39), ('plain', 40)]    # ('plain', 40)
                    print('     check_repeat_task_assign    ')
                    print('     uav  正常工作  但是  uav.task.task_id 重复出现    ')
                    return False # 一个uav 多个任务
        return True  # 逻辑 写反了  是否有重复  应该返回 false