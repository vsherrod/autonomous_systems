#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OG
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
import tf.transformations as tft
from std_msgs.msg import String
import laser_scan_subscriber
import numpy as np
import occ_grid_mapping as ogc
import os
import scipy.io as sio
import tf2_ros
import tf

def load_data(filename):

    data = sio.loadmat(filename)

    X = data['X'].T
    z = data['z'].T
    thk = data['thk'].T

    return X, z, thk

def initialize_occ_grid(frame_id = "odom", res = 1.0, width = 100, height = 100, origin = Pose(Point(0.0,0.0,0.0),Quaternion(0.0,0.0,0.0,1.0))):
    occ_grid = OG()
    occ_grid.header.frame_id = "world"
    occ_grid.info.resolution = res
    occ_grid.info.width = width
    occ_grid.info.height = height
    occ_grid.info.origin = origin
    occ_grid.data = occ_grid.info.width*occ_grid.info.height*[50]

    return occ_grid

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


def occ_grid_publisher(laser_sub):

    #true pos and true neg values
    true_pos = 0.7
    true_neg = 0.4

    #sensor model parameters
    alpha = 0.2
    beta = 0.1*np.pi/180.0
    z_max = 4.5

    #occupancy grid parameters
    res = 0.1
    width = 100
    height = 100
    origin_x = -width/2.0*res
    origin_y = -height/2.0*res


    #initialize ros node and publisher
    pub = rospy.Publisher('map', OG, queue_size=1)

    rate = rospy.Rate(5) #hz

    #initialize the occ_grid msg
    occ_grid = initialize_occ_grid("odom", res, width, height,Pose(Point(origin_x,origin_y,0.0),Quaternion(0.0,0.0,0.0,1.0)))

    #initialize the tf listener
    listener = tf.TransformListener()

    while not rospy.is_shutdown():

        timestamp, z, thk = laser_sub.getData()

        if timestamp!=None:

            #tf listener to get transform from odom to baselink at timestamp
            listener.waitForTransform('world', 'base_link_truth', timestamp, rospy.Duration(0.02))
            (trans, rot) = listener.lookupTransform('world', 'base_link_truth', timestamp)

            # convert quaternion to euler angles
            rot = tft.euler_from_quaternion(rot)

            # get rotation about the z-axis
            theta = rot[2]

            # assign the states to the correct variable
            X = trans[:2]
            X.append(theta)

            occ_grid.header.stamp = timestamp

            trans = [trans[0],trans[1],trans[2]]

            (trans2, rot2) = listener.lookupTransform('base_link_truth', 'world', timestamp)

            rot2 = tft.euler_from_quaternion(rot2)

            # pass in rot so that the points can be expressed in the bot frame
            occ_grid = ogc.occupancy_grid_mapping(occ_grid, X, z, thk, true_pos, true_neg, rot2, trans2, alpha, beta, z_max)

            pub.publish(occ_grid)

        rate.sleep()

if __name__ == '__main__':

    rospy.init_node('mapper', anonymous=True)

    laser_sub = laser_scan_subscriber.LaserScanSubscriber("/scan")


    try:
        occ_grid_publisher(laser_sub)

    except rospy.ROSInterruptException:
        pass
