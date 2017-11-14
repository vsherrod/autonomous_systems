import rospy
import numpy as np

class Landmark:
    def __init__(self,correspondence, x=0.0, y=0.0, covariance = 1000.0):
        self.x = x
        self.y = y
        self.sigma = covariance*np.eye((2,2))
        self.correspondence = correspondence
