import rospy

class FastSlamParticle:
    def __init__(self,occ_grid, x=0.0, y=0.0, theta=0.0, weight=0.0):

        self.x = x
        self.y = y
        self.theta = theta
        self.occ_grid = occ_grid
        self.weight = weight
