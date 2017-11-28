#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid as OG
import fast_slam
import fast_slam_particle
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from fast_slam_sim import find_highest_weighted_particle as highest_weight
from fast_slam_sim import find_mean_of_particles as particle_mean
import tf.transformations as tft
from std_msgs.msg import String
import laser_scan_subscriber
import twist_subscriber
import numpy as np
import occ_grid_mapping as ogc
from pdb import set_trace as pause
import os
import scipy.io as sio
import tf2_ros
import tf


def occ_grid_fast_slam(laser_sub, laser_res, velocity_sub):

    #initial position
    x0 = np.array([[0.0],[0.0],[0.0]])

    #true pos and true neg values
    true_pos = 0.7
    true_neg = 0.4

    #sensor model parameters
    alpha = 0.2
    beta = 60.0/(640.0/laser_res)*np.pi/180.0
    z_max = 10

    #occupancy grid parameters
    res = 0.1
    width = 100
    height = 100
    origin_x = -width/2.0*res
    origin_y = -height/2.0*res

    #motion model noise parameters
    alpha1 = 0.1
    alpha4 = 0.1
    alpha2 = 0.01
    alpha3 = 0.01

    alpha_vec = [alpha1, alpha2, alpha3, alpha4]

    #number of particles
    num_particles = 10


    #initialize ros node and publisher
    pub = rospy.Publisher('map', OG, queue_size=1)

    rate = rospy.Rate(20) #hz

    #initialize the occ_grid msg
    occ_grid = ogc.initialize_occ_grid("world", res, width, height,Pose(Point(origin_x,origin_y,0.0),Quaternion(0.0,0.0,0.0,1.0)))

    #generate particles
    particles = fast_slam_particle.generate_particles(x0, num_particles, occ_grid)

    #initialize the tf listener
    listener = tf.TransformListener()

    dt_init = True

    while not rospy.is_shutdown():

        # pass in the fraction of laser data you want to use
        timestamp, z, thk = laser_sub.getData()
        while timestamp==None:
            timestamp, z, thk = laser_sub.getData()

        if dt_init:

            t_final = timestamp.to_sec()-0.2
            dt_init = False
       
        if timestamp!=None:

            dt = timestamp.to_sec() - t_final
            
            # grab the linear and angular velocity command data
            linear, angular = velocity_sub.getData()
            
            u = [linear.x, angular.z]
            
            # get the next set of particles from occupancy_grid_fast_slam
            particles = fast_slam.occupancy_grid_fast_slam(particles, u, z, thk, alpha_vec, dt, true_pos, true_neg, alpha, beta, z_max)

            # find the particle with the max weight and the average of all the particles.
            particle_max_w = highest_weight(particles)
            particle_ave = particle_mean(particles)

            # broadcast the estimated pose (x, y, theta) using the highest weighted particle
            pose_broadcaster = tf.TransformBroadcaster()
            x = particle_max_w.x
            y = particle_max_w.y
            th = particle_max_w.theta
            
            pose_broadcaster.sendTransform((x,y,0), tf.transformations.quaternion_from_euler(0,0,th), timestamp, 'world', 'robot_estimation')
            
            
            particle_max_w.occ_grid.header.stamp = timestamp

            pub.publish(particle_max_w.occ_grid)
            
            # assign the final time 
            t_final = timestamp

        rate.sleep()
        

if __name__ == '__main__':

    rospy.init_node('slammer', anonymous=True)
    laser_res = 40
    laser_sub = laser_scan_subscriber.LaserScanSubscriber("/scan",laser_res)
    velocity_sub = twist_subscriber.TwistSubscriber("/mobile_base/commands/velocity")


    try:
        occ_grid_fast_slam(laser_sub, laser_res, velocity_sub)

    except rospy.ROSInterruptException:
        pass
