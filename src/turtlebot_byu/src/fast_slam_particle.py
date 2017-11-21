import rospy
import occ_grid_mapping as ogc
import copy

class FastSlamParticle:
    def __init__(self,occ_grid, x=0.0, y=0.0, theta=0.0, weight=0.0):

        self.x = x
        self.y = y
        self.theta = theta
        self.occ_grid = occ_grid
        self.weight = weight

class FastSlamParticleFeatures:
    def __init__(self,x=0.0, y=0.0, theta=0.0, weight=0.0):

        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

        self.landmarks = []
        self.seen = []

def normalize_weights(particles):
    weight_sum = 0.0
    for i in range(len(particles)):
        weight_sum += particles[i].weight

    for i in range(len(particles)):
        particles[i].weight = particles[i].weight/weight_sum


    return particles

def generate_particles(x0, num_particles, occ_grid):
    particles = []
    for i in range(num_particles):
        particle = FastSlamParticle(copy.deepcopy(occ_grid), x0[0][0], x0[1][0], x0[2][0])
        particles.append(particle)

    return particles
