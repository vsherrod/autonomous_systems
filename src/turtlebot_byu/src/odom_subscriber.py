import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import threading
import copy

class OdomSubscriber:
    def __init__(self, topic_name = "/odom"):

        rospy.Subscriber(topic_name, Odometry, self.callback)
        self.linear_ = Vector3()
        self.angular_ = Vector3()
        self.linear = Vector3()
        self.angular = Vector3()
        self.lock = threading.Lock()

    def callback(self, msg):

        self.lock.acquire()
        try:
            self.linear_ = msg.twist.twist.linear
            self.angular_ = msg.twist.twist.angular
        finally:
            self.lock.release()


    def getData(self):
        self.lock.acquire()
        try:
            self.linear = copy.deepcopy(self.linear_)
            self.angular= copy.deepcopy(self.angular_)

        finally:
            self.lock.release()

        return self.linear, self.angular






