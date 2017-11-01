import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Time
import threading
import copy

class LaserScanSubscriber:
    def __init__(self, topic_name = "/scan"):

        rospy.Subscriber(topic_name, LaserScan, self.callback)
        self.z_ = []
        self.theta_k_ = []
        self.timestamp_ = None
        self.z = []
        self.theta_k = []
        self.timestamp = None
        self.lock = threading.Lock()

    def callback(self, msg):

        self.lock.acquire()
        try:
            self.timestamp_ = msg.header.stamp
            self.theta_k_ = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.z_ = np.reshape(np.array(msg.ranges),(-1,1))
        finally:
            self.lock.release()

        # print self.z
        # print self.timestamp

    def getData(self):
        self.lock.acquire()
        try:
            self.timestamp = copy.deepcopy(self.timestamp_)
            self.theta_k = copy.deepcopy(self.theta_k_)
            self.z = copy.deepcopy(self.z_)
        finally:
            self.lock.release()

        return self.timestamp, self.z, self.theta_k






