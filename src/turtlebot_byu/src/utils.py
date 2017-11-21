#!/usr/bin/env python
import tf
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion

def create_transform(x, timestamp):
    transform = TransformStamped()

    transform.header.stamp = timestamp
    transform.header.frame_id = "map"
    transform.child_frame_id = "robot"

    transform.transform.translation.x = x[0]
    transform.transform.translation.y = x[1]
    transform.transform.translation.z = 0.0

    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, x[2])
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]

    return transform
