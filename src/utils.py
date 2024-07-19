import numpy as np
from scipy.spatial import ConvexHull
from matplotlib import path
from typing import Literal, Union, Dict, List


def norm(x):
    return np.linalg.norm(x)

def dis_between_points(p1:np.ndarray, p2:np.ndarray):
    return norm(p1 - p2)

# start from the p1, go to p2 with distance
def move_alone_line(p1:List, p2:List, dis):
    direction = (p2-p1)/max(norm(p2-p1), 0.01)
    return p1 + direction * dis

def points_on_the_same_line(p1:np.ndarray, p2:np.ndarray, p3:np.ndarray):
    if abs(p3[0] - p2[0]) < 0.001 and abs((p2[0] - p1[0])) < 0.0001:
        return True
    elif abs(p3[0] - p2[0]) < 0.001 or abs((p2[0] - p1[0])) < 0.0001:
        return False

    if abs((p3[1] - p2[1])/(p3[0] - p2[0]) - (p2[1] - p1[1])/(p2[0] - p1[0])) < 0.0001 :
        return True
    else:
        return False

def point_in_rectangle(center_x, center_y, width, height, point):
    # 计算矩形的左上角和右下角的坐标
    left = center_x - width/2
    right = center_x + width/2
    top = center_y - height/2
    bottom = center_y + height/2
    
    # 判断点是否在矩形中
    if point[0] >= left and point[0] <= right and point[1] >= top and point[1] <= bottom:
        return True
    else:
        return False
    
    
def isCollisionFreeVertex(obstacles, xy):

    collFree = True
    
    for obstacle in obstacles:
        hull = path.Path(obstacle)
        collFree = not hull.contains_points([xy])
        if hull.contains_points([xy]):
            # print 'collision'
            return collFree

    return collFree