import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import threading
import pdb
import copy

class TwistSubscriber:
    def __init__(self, topic_name = "/vel_cmd"):

        rospy.Subscriber(topic_name, Twist, self.callback)
        self.linear_ = Vector3()
        self.angular_ = Vector3()
        self.linear = Vector3()
        self.angular = Vector3()
        self.lock = threading.Lock()

    def callback(self, msg):

        self.lock.acquire()
        try:
            self.linear_ = msg.linear
            self.angular_ = msg.angular
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






