#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation
# import open3d as o3d
 
# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector
#!!!A是新坐标点，B是原坐标点：R，t代表A在B中的位姿 
#或者
#!!!A是原坐标点，B是新坐标点：R，t代表B到A的变换
def rigid_transform_3D(A, B):
    A=np.array(A)
    B=np.array(B)
    assert len(A) == len(B)
 
    N = A.shape[0]  # total points
    centroid_A = np.mean(A, axis=0)# 求A中x、y、z的坐标平均值
    centroid_B = np.mean(B, axis=0)# 求B中x、y、z的坐标平均值
 
    # centre the points，两组点平移到一个原点，只看旋转
    AA = A - np.tile(centroid_A, (N, 1))    #tile函数把array向下复制N倍
    BB = B - np.tile(centroid_B, (N, 1))    # A减去复制后每个点的平均值得到AA
 
    H = np.matmul(np.transpose(AA),BB) #transpose according to need that we get 3x3matrix and add them
    U, S, Vt = np.linalg.svd(H)
    R = np.matmul(Vt.T, U.T)    #vt是默认得到的转置后的结果？
 
    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = np.matmul(Vt.T,U.T) 
 
    t = -np.matmul(R, centroid_A) + centroid_B
    # err = B - np.matmul(A,R.T) - t.reshape([1, 3])

    r=Rotation.from_matrix(R)
    Q=Rotation.as_quat(r)

    euler=r.as_euler('zxy',degrees=True)
    # print("框位姿四元数",Q)
    # print("框欧拉角",euler)
    # print("框位移",t)
    return Q
 
 
if __name__=='__main__':
    a = np.array([[0,0,0],
                  [0,0,1],
                  [0,1,1],
                  [0,1,0]])
    b = np.array([[-1,0,0], 
                  [-1,1,0],
                  [-1,1,-1],
                  [-1,0,-1]])
 
 
    # c = np.reshape(a[-2:], (2, 3))
    # test_a1 = np.reshape(c[0],(1,3))
    # test_a2 = np.reshape(c[1],(1,3))
 
    # c=np.reshape(b[-2:], (2, 3))
    # test_b1 = np.reshape(c[0],(1,3))
    # test_b2 = np.reshape(c[1],(1,3))

    r= rigid_transform_3D(a, b)
 
    # bb = np.matmul(a, r.T) + t.reshape([1, 3])
    # print('b-bb:', b - bb)
 
    # c = np.matmul(test_a1, r.T) + t.reshape([1, 3])
    # print('c-test_b1:', c - test_b1)
 
    # c = np.matmul(test_a2, r.T) + t.reshape([1, 3])
    # print('c-test_b2:', c - test_b2)
 
    # pcd  = o3d.io.read_point_cloud(pcdnamec)
    # point_arr =np.asarray(pcd.points)* 1000
    # color_arr =np.asarray(pcd.colors)
    # point_arrb = np.matmul(point_arr, r.T) + t.reshape([1, 3])
    # pcd_new = o3d.geometry.PointCloud()
    # pcd_new.points = o3d.utility.Vector3dVector(point_arrb)
    # pcd_new.colors = o3d.utility.Vector3dVector(color_arr)
    # o3d.io.write_point_cloud(savenameb, pcd_new)
 
 