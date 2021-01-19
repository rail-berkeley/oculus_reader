from pyquaternion import Quaternion
from visual_mpc.envs.robot_envs.util.vr_control.oculus_reader.reader import OculusReader
import numpy as np

import visual_mpc.agent.utils.transformation_utils as tr

import rospy
import tf2_ros
import geometry_msgs.msg


def oculus_system_to_world_system(current_vr_transform):
    current_vr_transform = tr.RpToTrans(Quaternion(axis=[0, 0, 1], angle=-np.pi / 2).rotation_matrix,
                                        np.zeros(3)).dot(
        tr.RpToTrans(Quaternion(axis=[1, 0, 0], angle=np.pi / 2).rotation_matrix, np.zeros(3))).dot(
        current_vr_transform)
    return current_vr_transform

def publish_transform(transform, name):

    # world_transform = oculus_system_to_world_system(transform)
    world_transform = transform

    # print('transform', transform)
    # import pdb; pdb.set_trace()
    quat = Quaternion(matrix=world_transform[:3, :3])
    translation = world_transform[:3, 3]

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id = name
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    t.transform.rotation.x = quat.vector[0]
    t.transform.rotation.y = quat.vector[1]
    t.transform.rotation.z = quat.vector[2]
    t.transform.rotation.w = quat.real

    br.sendTransform(t)

def main():
    oculus_reader = OculusReader()
    rospy.init_node('oculus_reader')

    while True:
        rospy.sleep(0.4)
        poses, buttons = oculus_reader.get_arays_and_button()
        if poses == []:
            continue

        right_controller_pose = poses[0]
        publish_transform(right_controller_pose, 'oculus')

if __name__ == '__main__':
    main()