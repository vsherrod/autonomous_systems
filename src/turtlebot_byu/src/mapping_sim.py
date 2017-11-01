#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OG
from geometry_msgs.msg import TransformStamped
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

def initialize_occ_grid():
    occ_grid = OG()
    occ_grid.header.frame_id = "map"
    occ_grid.info.resolution = 1.0
    occ_grid.info.width = 100
    occ_grid.info.height = 100
    occ_grid.info.origin.position.x = 0.0
    occ_grid.info.origin.position.y = 0.0
    occ_grid.info.origin.position.z = 0.0
    occ_grid.info.origin.orientation.x = 0.0
    occ_grid.info.origin.orientation.y = 0.0
    occ_grid.info.origin.orientation.z = 0.0
    occ_grid.info.origin.orientation.w = 1.0
    occ_grid.data = 100*100*[50]

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
    occ_grid = initialize_occ_grid()

    #initialize the tf broadcaster
    br = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        occ_grid.header.stamp = rospy.Time.now()
        transform = create_transform(X[i], occ_grid.header.stamp)
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
