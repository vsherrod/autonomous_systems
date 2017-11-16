import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Time
import threading
import pdb
import copy

class LaserScanSubscriber:
    def __init__(self, topic_name = "/scan", num_laser = 1):

        rospy.Subscriber(topic_name, LaserScan, self.callback)
        self.z_ = []
        self.theta_k_ = []
        self.timestamp_ = None
        self.timestamp = None
        self.lock = threading.Lock()
        self.num_laser = num_laser
        if 640%num_laser==0:
            self.theta_k = np.zeros((640/num_laser+1,1))
            self.z = np.zeros((640/num_laser+1,1))
        else:
            self.theta_k =  np.zeros((640/num_laser+2,1))
            self.z = np.zeros((640/num_laser+2,1))
            

    def callback(self, msg):

        self.lock.acquire()
        try:
            self.timestamp_ = msg.header.stamp
            self.theta_k_ = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.z_ = np.reshape(np.array(msg.ranges),(-1,1))
        finally:
            self.lock.release()


    def getData(self):
        self.lock.acquire()
        try:
            self.timestamp = copy.deepcopy(self.timestamp_)
            # self.theta_k = copy.deepcopy(self.theta_k_)
            # self.z = copy.deepcopy(self.z_) 
            


            if len(self.theta_k_)!=0:
                for i in range(len(self.theta_k)):                   

                    if i!=len(self.theta_k)-1:
                        self.theta_k[i] = self.theta_k_[self.num_laser*i]
                        self.z[i] = self.z_[self.num_laser*i]
                    
                    else:
                        # always add on the last laser
                        self.theta_k[i] = self.theta_k_[-1]
                        self.z[i] = self.z_[-1]
 
        finally:
            self.lock.release()

        # reshape z so that it is a column vector of shape [n,1]
        self.z = np.reshape(self.z,(-1,1))
            
        return self.timestamp, self.z, self.theta_k






