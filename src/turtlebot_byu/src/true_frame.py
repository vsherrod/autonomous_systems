#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf
import math
import numpy as np


class BotFrameTranslator(object):

    def __init__(self):

        self.pose_nwu = PoseStamped()

        # define rotation from Motion capture frame (m) to NWU frame (nwu)
        self.rot_m_t = np.array([[0.0, 0.0, 1.0, 0.0],
                                  [1.0, 0.0, 0.0, 0.0],
                                  [0.0, 1.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 1.0]])

        # initialize pose subscriber
        self.pose_sub = rospy.Subscriber("/vrpn_client_node/turtle_hokuyo/pose", PoseStamped, self.pose_callback)

        # initialize pose publisher
        # self.pose_pub = rospy.Publisher("/nwu_turtlebot_pose", PoseStamped, queue_size=1)

    def pose_callback(self, msg):


        # publish the message
        # self.pose_pub.publish(self.pose_nwu)

        # send transform through tf broadcaster
        self.br = tf.TransformBroadcaster()
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),(msg.pose.orientation.x,msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),msg.header.stamp,'markerset','motive')

        # euler = tf.transformations.euler_from_quaternion(quaternion_nwu)

        # print "Yaw: " + str(math.degrees(euler[2]))


def main():
    # initialize a node
    rospy.init_node('bot_frame_translator')

    # create instance of ExposureController class
    translator = BotFrameTranslator()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
