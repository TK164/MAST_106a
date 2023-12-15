from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
    
# def transform_to_kdl(t):
#     return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
#                                                     t.transform.rotation.z, t.transform.rotation.w),
#                         PyKDL.Vector(t.transform.translation.x, 
#                                         t.transform.translation.y, 
#                                         t.transform.translation.z))

def do_transform_pose(pose, transform):
   
    orientation = transform.transform.rotation
    transform_g = np.array(quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w]))
    transform_g[0,3] = transform.transform.translation.x
    transform_g[1,3] = transform.transform.translation.y
    transform_g[2,3] = transform.transform.translation.z
    print("transform:\n", transform_g)

    orientation = pose.pose.orientation
    pose_g = np.array(quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w]))
    pose_g[0,3] = pose.pose.position.x
    pose_g[1,3] = pose.pose.position.y
    pose_g[2,3] = pose.pose.position.z
    print("pose:\n", pose_g)

    f = transform_g @ pose_g
    print("result:\n", f)
    res = PoseStamped()
    res.pose.position.x = f[0, 3]
    res.pose.position.y = f[1, 3]
    res.pose.position.z = f[2, 3]
    (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = quaternion_from_matrix(f)
    
    res.header = transform.header
    return res

def main():
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 1
    pose.pose.position.z = 0
    rot_mat = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w) = quaternion_from_matrix(rot_mat)

    transform = TransformStamped()
    transform.transform.translation.x = 0
    transform.transform.translation.y = 0
    transform.transform.translation.z = 0
    rot_mat = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w) = quaternion_from_matrix(rot_mat)
    
    pose = do_transform_pose(pose, transform)
    orientation = pose.pose.orientation
    pose_g = np.array(quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w]))
    pose_g[0,3] = pose.pose.position.x
    pose_g[1,3] = pose.pose.position.y
    pose_g[2,3] = pose.pose.position.z

if __name__ == '__main__':
    main()
