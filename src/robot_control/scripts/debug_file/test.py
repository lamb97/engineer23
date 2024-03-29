import rospy
from scipy.spatial.transform import Rotation
from tf2_geometry_msgs import PoseStamped as tf2_PoseStamped
from geometry_msgs.msg import PoseStamped,Vector3Stamped
import numpy as np

R1=Rotation.from_euler("zyx",[48,0,0],True)
R2=Rotation.from_euler("zyx",[0,0,90],True)
R1_matrix=R1.as_matrix()
R2_matrix=R2.as_matrix()
R_matrix=np.dot(R2_matrix,R1_matrix)
R=Rotation.from_matrix(R_matrix)
R_euler=R.as_euler("zyx",True)

a=[[1],[0],[0]]
a2=np.dot(R_matrix,a)


R3=Rotation.from_quat([0.70710678,0,0,0.70710678])
R3_euler=R3.as_euler("zyx",True)

print(R1_matrix)
print(R2_matrix)
print(R_matrix)
print(R_euler)
print(a2)
print(R3_euler)
print(R2.as_quat())
