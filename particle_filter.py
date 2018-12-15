# ------------------------------------------------------------------------
# coding=utf-8
# ------------------------------------------------------------------------
#
#  Created by Martin J. Laubach on 2011-11-15
#
# ------------------------------------------------------------------------

from __future__ import absolute_import

import random
import math
import bisect
import sys
import time
from util import *

from draw import Maze


PARTICLE_COUNT = 1000    # Total number of particles

ROBOT_HAS_COMPASS = True  # Does the robot know where north is? If so, it
# makes orientation a lot easier since it knows which direction it is facing.
# If not -- and that is really fascinating -- the particle filter can work
# out its heading too, it just takes more particles and more time. Try this
# with 3000+ particles, it obviously needs lots more hypotheses as a particle
# now has to correctly match not only the position but also the heading.

# ------------------------------------------------------------------------
# Some utility functions
ranges = []
headings = []
distances = []

start = []

resolution = 4
headingCount = 0

trajectoryName = "trajectories_2.txt"
mapName = "map_2.txt"


def getHeading():
    headings = []
    with open(trajectoryName) as f:
        for line in f:
            if line[:7] == "Heading":
                headings.append(line[15:-2])
    return headings

def getDistances():
    distances = []
    with open(trajectoryName) as f:
        for line in f:
            if line[:8] == "Distance":
                distances.append(line[16:-2])
    return distances


def getStart():
    pair = []
    with open(trajectoryName) as f:
        for line in f:
            if line[:3] == "  x":
                pair.append(float(line[5:-1]))
            if line[:3] == "  y":
                pair.append(float(line[5:-1]))
    return pair


def getRanges():
    ranges = []
    rangesStrArr = []
    
    with open(trajectoryName) as f:
        for line in f:
            if line[:8] == "  ranges":
                rangesStrArr.append(line[11:-2])

    for range in rangesStrArr:
        ranges.append(range.split(", "))
    
    return ranges


def getRFID():
    linesFromFile = []
    
    filled = 1
    empty = 0
    
    with open(mapName) as f:
        for line in f:
            line = line.replace("(", "")
            line = line.replace(")", "")
            line = line.replace("\n", "")
            linesFromFile.append(line)
        
        matrixCornerFullString = linesFromFile[0]
        matrixCornersArr = matrixCornerFullString.split(' ')
    
    obstacleList = []

    x = 2
    while x < len(linesFromFile):
        obstacleStrings = linesFromFile[x].split(' ')
        
        polygon = []
        
        for string in obstacleStrings:
            polygon.append(string.split(','))
        obstacleList.append(polygon)

        x = x + 1

    a = matrixCornersArr[0].split(',')
    b = matrixCornersArr[1].split(',')
    c = matrixCornersArr[2].split(',')
    d = matrixCornersArr[3].split(',')

    offset_x = 0 - float(d[0])
    offset_y = 0 - float(b[1])

    for obstacle in obstacleList:
        for point in obstacle:
            if point[0]:
                point[0] = float(point[0]) + offset_x
                point[1] = float(point[1]) + offset_y
                
                point[0] = point[0] * resolution
                point[1] = point[1] * resolution
                    
                temp = point[0]
                point[0] = point[1]
                point[1] = temp

    start[0] = float(start[0]) + offset_x
    start[1] = float(start[1]) + offset_y
    start[0] = start[0] * resolution
    start[1] = start[1] * resolution

    matrixHeight = abs(float(a[1]) - float(b[1]))
    matrixWidth = abs(float(a[0]) - float(d[0]))

    matrix = []

    for row in range(0, int(matrixHeight) * resolution):
        rowArr = []
        for col in range(0, int(matrixWidth) * resolution):
            rowArr.append(empty)
        matrix.append(rowArr)

    for x in range(0, int(matrixHeight) * resolution):
        print(x)
        matrix[x][0] = filled
        matrix[int(matrixHeight) * resolution - 1][x] = filled
        matrix[x][int(matrixHeight) * resolution - 1] = filled
        matrix[0][x] = filled
            
        for y in range(0, int(matrixHeight) * resolution):
            for x in range(0, int(matrixWidth) * resolution):
                if y + 1 >= matrixHeight * resolution or x + 1 >= matrixWidth * resolution:
                    continue
                # topleft
                
                for obstacle in obstacleList:
                    if inside_polygon(x, y, obstacle) or on_polygon(x, y, obstacle):
                        matrix[x][y] = filled
            
                # topright
                for obstacle in obstacleList:
                    if inside_polygon(x + 1, y, obstacle) or on_polygon(x + 1, y, obstacle):
                        matrix[x][y] = filled

                # bottomleft
                for obstacle in obstacleList:
                    if inside_polygon(x, y + 1, obstacle) or on_polygon(x, y + 1, obstacle):
                        matrix[x][y] = filled
                    
                # bottomright
                for obstacle in obstacleList:
                    if inside_polygon(x + 1, y + 1, obstacle) or on_polygon(x + 1, y + 1, obstacle):
                        matrix[x][y] = filled
    print(matrix)
    return matrix



def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]


def add_little_noise(*coords):
    return add_noise(0.02, *coords)


def add_some_noise(*coords):
    return add_noise(0.1, *coords)


# This is just a gaussian kernel I pulled out of my hat, to transform
# values near to robbie's measurement => 1, further away => 0
sigma2 = 0.95 ** 2


def w_gauss(a, b):
    size = min(len(a), len(b))
    sum_error = 0
    g = 0
    # print(size)

    for count in range(size):
        if (a[count] == "nan") and (b[count] == "nan"):
            sum_error = 0
        elif a[count] == "nan":
            sum_error = abs(b[count])
        elif b[count] == "nan":
            sum_error = abs(float(a[count]))
        else:
            sum_error = abs(float(a[count]) - b[count])

        g += math.e ** -(sum_error ** 2 / (2 * sigma2))
        
        
    avg = g / 54
    return avg

# ------------------------------------------------------------------------


def compute_mean_point(particles):
    """
    Compute the mean for all particles that have a reasonably good weight.
    This is not part of the particle filter algorithm but rather an
    addition to show the "best belief" for current position.
    """

    m_x, m_y, m_count = 0, 0, 0
    for p in particles:
        m_count += p.w
        m_x += p.x * p.w
        m_y += p.y * p.w

    if m_count == 0:
        return -1, -1, False

    m_x /= m_count
    m_y /= m_count

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0
    for p in particles:
        if world.distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_count > PARTICLE_COUNT * 0.95

# ------------------------------------------------------------------------


class WeightedDistribution(object):
    def __init__(self, state):
        accum = 0.0
        self.state = [p for p in state if p.w > 0]
        self.distribution = []
        for x in self.state:
            accum += x.w
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.state[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None

# ------------------------------------------------------------------------


class Particle(object):
    def __init__(self, x, y, heading=None, w=1, noisy=False):
        if heading is None:
            heading = random.uniform(0, 360)
        if noisy:
            x, y, heading = add_some_noise(x, y, heading)

        self.x = x
        self.y = y
        self.h = heading
        self.w = w

    def __repr__(self):
        return "(%f, %f, w=%f)" % (self.x, self.y, self.w)

    @property
    def xy(self):
        return self.x, self.y

    @property
    def xyh(self):
        return self.x, self.y, self.h

    @classmethod
    def create_random(cls, count, maze):
        return [cls(*maze.random_free_place()) for _ in range(0, count)]

    def drawline(self, coords_a, slope, maze):

        if slope == 0:
            x = coords_a[0] + 10
            y = coords_a[1]

        elif slope == sys.maxsize:
            x = coords_a[0]
            y = coords_a[1] + 10

        else:
            dx = (10 / math.sqrt(1 + (slope * slope)))
            dy = slope * dx
            x = coords_a[0] + dx
            y = coords_a[1] + dy

        x0, y0 = coords_a
        x1 = x
        y1 = y
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if x0 == x1 and y0 == y1:
                break
            if maze.is_free(x0, y0) is False:
                break
            e2 = err * 2
            if e2 > -dy:
                err = err - dy
                x0 = x0 + sx
            if e2 < dx:
                err = err + dx
                y0 = y0 + sy

        return (x0, y0)

    def read_sensor(self, maze, head_data):
        """
        Return array of ranges
        """
        ranges = []
        start = math.cos(math.radians(head_data)) - 30
        self.h = start
        for head in range(54):
            slope = math.tan(self.h)
            endpoint = self.drawline((self.x, self.y), slope, maze)

            if maze.is_free(endpoint[0], endpoint[1]) is False:
                ranges.append(math.hypot(endpoint[0] - self.x, endpoint[1] - self.y))

            else:
                ranges.append("nan")

            self.h += 1.125

        return ranges

    def advance_by(self, speed, checker=None, noisy=False):
        h = self.h
        if noisy:
            speed, h = add_little_noise(speed, h)
            h += random.uniform(-3, 3)  # needs more noise to disperse better
        r = math.radians(h)
        dx = math.sin(r) * speed
        dy = math.cos(r) * speed
        if checker is None or checker(self, dx, dy):
            self.move_by(dx, dy)
            return True
        return False

    def move_by(self, x, y):
        self.x += x
        self.y += y

# ------------------------------------------------------------------------


class Robot(Particle):
    speed = 0.2
    distanceCount = 0

    def __init__(self, maze):
        super(Robot, self).__init__(*start, heading=90)
        self.chose_random_direction()
        self.step_count = 0

    def chose_random_direction(self):
        heading = random.uniform(0, 360)
        self.h = heading

    def read_sensor(self, maze):
        """
        Poor robot, it's sensors are noisy and pretty strange,
        it only can measure the distance to the nearest beacon(!)
        and is not very accurate at that too!
        """
        return add_little_noise(super(Robot, self).read_sensor(maze))[0]

    def move(self, maze):
        """
        Move the robot. Note that the movement is stochastic too.
        """
        while True:
            self.step_count += 1
            if self.advance_by(float(distances[self.distanceCount])*resolution, noisy=True,
                               checker=lambda r, dx, dy: maze.is_free(r.x + dx, r.y + dy)):
                break
        
            # Bumped into something or too long in same direction,
            # chose random new direction
            self.chose_random_direction()

        self.distanceCount += 1

# ------------------------------------------------------------------------

start = getStart()
distances = getDistances()
maze_data = getRFID()

headings = getHeading()

ranges = getRanges()
# print(ranges)

world = Maze(maze_data)
world.draw()

# initial distribution assigns each particle an equal probability
particles = Particle.create_random(PARTICLE_COUNT, world)
robbie = Robot(world)
count = 0

while count < len(headings):
    # Read robbie's sensor
    heading_data = headings[count]

    # Update particle weight according to how good every particle matches
    # robbie's sensor reading
    for p in particles:
        if world.is_free(*p.xy):
            # print(heading_data)
            point_range = p.read_sensor(world, float(heading_data))
            p.w = w_gauss(ranges[count], point_range)
        else:
            p.w = 0
        

    # ---------- Try to find current best estimate for display ----------
    m_x, m_y, m_confident = compute_mean_point(particles)

    # ---------- Show current state ----------
    world.show_particles(particles)
    world.show_mean(m_x, m_y, m_confident)
    world.show_robot(robbie)

    # ---------- Shuffle particles ----------
    new_particles = []

    # Normalise weights
    nu = sum(p.w for p in particles)
    if nu:
        for p in particles:
            p.w = p.w / nu

    # create a weighted distribution, for fast picking
    dist = WeightedDistribution(particles)

    for _ in particles:
        p = dist.pick()
        if p is None:  # No pick b/c all totally improbable
            new_particle = Particle.create_random(1, world)[0]
        else:
            new_particle = Particle(p.x, p.y,
                                    heading=robbie.h if ROBOT_HAS_COMPASS else p.h,
                                    noisy=True)
        new_particles.append(new_particle)

    particles = new_particles

    # ---------- Move things ----------
    old_heading = robbie.h
    robbie.move(world)
    d_h = robbie.h - old_heading
    count += 1

    # Move particles according to my belief of movement (this may
    # be different than the real movement, but it's all I got)
    for p in particles:
        p.h += d_h  # in case robot changed heading, swirl particle heading too
        p.advance_by(robbie.speed)
    time.sleep(0.5)

time.sleep(20)

