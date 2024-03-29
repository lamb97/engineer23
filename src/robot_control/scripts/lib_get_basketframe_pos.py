#!/usr/bin/env python3
from matplotlib import pyplot as plt
import numpy as np

#通过现在的函数，得到在以框的某一个点为坐标系下的各点坐标，同时返回各个点在新坐标系下的坐标，以及新坐标系原点在旧坐标系下的坐标

# 定义一个top_point的类，对实例属性进行一个命名并进行赋值
class top_point():
    def __init__(self,set_point,set_pos,origin):
        self.point = set_point
        self.pos = set_pos
        self.origin = origin
        self.distance_square_to_origin = 0
        self.vector_origin_to_here_in_world = [axis_point-axis_origin for axis_point,axis_origin in zip(self.point,self.origin)] # 进行一个打包处理
        self.point_after= []


def get_basketframe_pos(points):
    # 1.获取原点，x轴点，y轴点
    class_points = [top_point(point, pos, points[0]) for pos, point in enumerate(points)]   # 给points以一个索引
    class_origin = class_points[0]# 让第一个点作为原点
    class_other_points = class_points[1:]
    class_x_point = None
    class_y_point = None
    class_xy_point = None #另外一个点
    #求各点到与原点距离
    for class_point in class_other_points:
        distance_square = 0
        # 三维坐标距离计算
        for i in range(3):
            distance_square+=(class_origin.point[i]-class_point.point[i])**2
        class_point.distance_square_to_origin=distance_square
    #根据与原点距离排序
    class_other_points.sort(key=lambda x:x.distance_square_to_origin)
    class_xy_point=class_other_points[2]
    #取最近两点作为xy轴上点,并区分xy的次序 ！！！好像没必要
    cross_ox_oy = np.cross(class_other_points[0].vector_origin_to_here_in_world,class_other_points[1].vector_origin_to_here_in_world)
    dot_cross_oy = np.dot([1,0,0],cross_ox_oy)
    if dot_cross_oy>0:# 1在0的逆时针方向时
        class_x_point=class_other_points[0]
        class_y_point=class_other_points[1]
    else:
        class_x_point = class_other_points[1]
        class_y_point = class_other_points[0]
    #2.计算新轴下的各点坐标
    new_points=[[],[],[],[]]
    new_points[class_origin.pos] = [0,0,0]
    new_points[class_x_point.pos] = [np.sqrt(class_x_point.distance_square_to_origin),0,0]
    new_points[class_y_point.pos] = [0,np.sqrt(class_y_poin坐标t.distance_square_to_origin),0]
    new_points[class_xy_point.pos] = [np.sqrt(class_x_point.distance_square_to_origin),np.sqrt(class_y_point.distance_square_to_origin),0]
    #
    # print("转换后坐标")
    # for i in range(4):
    #     print(new_points[i])
    return new_points,class_origin.point
def pointcloud_show(points):
    points =np.array(points)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    ax = plt.subplot(projection='3d')
    ax.scatter(x, y, z)

    ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
    ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
    ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
    plt.show()
if __name__ == "__main__":
    points = [[2, 3, 1], [3, 2, 1], [5, 4, 2], [6, 3, 2]]
    pointcloud_show(points)
    new_point=get_basketframe_pos(points)
    pointcloud_show(new_point)
