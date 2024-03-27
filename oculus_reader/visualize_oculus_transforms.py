#!/usr/bin/env python
from reader import OculusReader
from tf.transformations import quaternion_from_matrix
import rospy
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import Bool, Float32, Float32MultiArray

import copy
import numpy as np


frame_tree = {
    # child_frame: parent_frame
    'ForearmStub': 'WristRoot',
    'Thumb0': 'WristRoot',
    'Thumb1': 'Thumb0',
    'Thumb2': 'Thumb1',
    'Thumb3': 'Thumb2',
    'Index1': 'WristRoot',
    'Index2': 'Index1',
    'Index3': 'Index2',
    'Middle1': 'WristRoot',
    'Middle2': 'Middle1',
    'Middle3': 'Middle2',
    'Ring1': 'WristRoot',
    'Ring2': 'Ring1',
    'Ring3': 'Ring2',
    'Pinky0': 'WristRoot',
    'Pinky1': 'Pinky0',
    'Pinky2': 'Pinky1',
    'Pinky3': 'Pinky2',
    'ThumbTip': 'Thumb3',
    'IndexTip': 'Index3',
    'MiddleTip': 'Middle3',
    'RingTip': 'Ring3',
    'PinkyTip': 'Pinky3',
}

def publish_transform(transform, name, parent_frame):
    # when switch tracked device from hand to controller, the first transform is likely to be invalid
    if np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-3):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        translation = transform[:3, 3]
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = name
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        quat = quaternion_from_matrix(transform)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        br.sendTransform(t)

def publish_joint_transform(transform, name, prefix=''):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    translation = transform[:3, 3]
    t.header.stamp = rospy.Time.now()
    if 'WristRoot' in name:
        t.header.frame_id = prefix + 'palm'
    else:
        t.header.frame_id = prefix + frame_tree[name]
    t.child_frame_id = prefix + name
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    quat = quaternion_from_matrix(transform)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)

def process_transformations(transformations):
    tmp_transformations = copy.deepcopy(transformations)
    for name, transform in transformations.items():
        if name in frame_tree:
            parent_frame = frame_tree[name]
            parent_transform = transformations[parent_frame]
            if np.linalg.cond(parent_transform) < 1/np.finfo(parent_transform.dtype).eps:
                tmp_transformations[name] = np.matmul(np.linalg.inv(parent_transform), transform)
            else:
                tmp_transformations[name] = np.eye(4)
    return tmp_transformations

def init_button_publisher():
    namelist = ['A', 'B', 'RThU', 'RJ', 'RG', 'RTr', 'rightJS', 'rightGrip', 'rightTrig',
                'X', 'Y', 'LThU', 'LJ', 'LG', 'LTr',  'leftJS',  'leftGrip',  'leftTrig']
    typelist = [Bool, Bool, Bool, Bool, Bool, Bool, Float32MultiArray, Float32, Float32,
                Bool, Bool, Bool, Bool, Bool, Bool, Float32MultiArray, Float32, Float32]
    prefix = '/oculus_reader/'
    topiclist = [prefix + name for name in namelist]
    publishers = {}
    for idx, (topic, msg_type) in enumerate(zip(topiclist, typelist)):
        publishers[namelist[idx]] = rospy.Publisher(topic, msg_type, queue_size=1)
    return publishers

def publish_buttons(buttons, publishers):
    for key, value in buttons.items():
        if key in publishers:
            if isinstance(value, bool):
                msg = Bool()
            elif isinstance(value, tuple):
                if len(value) == 1:
                    msg = Float32()
                    value = value[0]
                else:
                    msg = Float32MultiArray()
                    value = list(value)
            msg.data = value
            publishers[key].publish(msg)

def main():
    oculus_reader = OculusReader()
    rospy.init_node('oculus_reader')
    rate = rospy.get_param('~rate', 1.0)
    button_publishers = init_button_publisher()
    while not rospy.is_shutdown():
        rospy.sleep(1.0 / rate)
        transformations, buttons = oculus_reader.get_transformations_and_buttons()
        if 'l' not in transformations or 'r' not in transformations or 'h' not in transformations:
            continue

        head_pose = transformations['h']
        publish_transform(head_pose, 'oculus_head', 'init_head')

        left_controller_pose = transformations['l']
        publish_transform(left_controller_pose, 'l_palm', 'oculus_head')
        right_controller_pose = transformations['r']
        publish_transform(right_controller_pose, 'r_palm', 'oculus_head')

        left_joint_transformations, right_joint_transformations = oculus_reader.get_joint_transformations()
        left_joint_transformations = process_transformations(left_joint_transformations)
        right_joint_transformations = process_transformations(right_joint_transformations)
        for joint_name, transform in left_joint_transformations.items():
            publish_joint_transform(transform, joint_name, prefix='l_')
        for joint_name, transform in right_joint_transformations.items():
            publish_joint_transform(transform, joint_name, prefix='r_')

        if len(left_joint_transformations) == 0:
            publish_buttons(buttons, button_publishers)

if __name__ == '__main__':
    main()
