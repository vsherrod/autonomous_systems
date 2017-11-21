#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OG
from geometry_msgs.msg import TransformStamped
import occ_grid_mapping as ogc
import os
import scipy.io as sio
import tf2_ros
import tf
import utils

def load_data(filename):

    data = sio.loadmat(filename)

    X = data['X'].T
    z = data['z'].T
    thk = data['thk'].T

    return X, z, thk

def occ_grid_publisher():

    #load data for mapping
    X, z, thk = load_data(os.path.expanduser('~') + '/git/autonomous_systems/src/turtlebot_byu/data/state_meas_data.mat')
    #iterator for data
    i = 0

    #true pos and true neg values
    true_pos = 0.7
    true_neg = 0.4

    #initialize ros node and publisher
    pub = rospy.Publisher('map', OG, queue_size=1)
    rospy.init_node('mapper', anonymous=True)
    rate = rospy.Rate(50) #50 hz

    #initialize the occ_grid msg
    occ_grid = ogc.initialize_occ_grid()

    #initialize the tf broadcaster
    br = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        occ_grid.header.stamp = rospy.Time.now()
        transform = utils.create_transform(X[i], occ_grid.header.stamp)
        br.sendTransform(transform)

        occ_grid = ogc.occupancy_grid_mapping(occ_grid, X[i], z[i], thk, true_pos, true_neg)

        pub.publish(occ_grid)

        i += 1
        rate.sleep()


if __name__ == '__main__':

    try:
        occ_grid_publisher()

    except rospy.ROSInterruptException:
        pass
