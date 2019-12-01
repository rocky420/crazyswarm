from pycrazyswarm import *
import numpy as np
import math

Z = 0
Z_HOME = 1.0
FREQ_PUB = 20
THR = 0.02

# Cube around the movement of the crazyflie
MAX_X = 1.0
MAX_Y = 1.0
MAX_Z = 1.6

# Position in Z for each planar movements
Z_PLAN = 1.4


def id_to_idx(id):
    return id-1


def idx_to_id(idx):
    return idx+1


# Define a uniform helix trajectory. Each point returned should be send at the frequency freq
def uniform_helix(freq, duration):
    x = list()
    y = list()
    z = list()
    t = list()
    nb_point = freq * duration
    for i in range(nb_point):
        t0 = i * (1.0 / freq)
        x.append(MAX_X * math.cos(1.0 * t0))
        y.append(MAX_Y * math.sin(1.0 * t0))
        z.append((t0 / duration) * MAX_Z)
        t.append(1.0 / freq)
    return t, x, y, z


def reached_target_position(cfs, target_positions):
    for cf in cfs:
        if np.linalg.norm((cf.position() - target_positions[id_to_idx(cf.id)])) > THR:
            return False
    return True


if __name__ == '__main__':
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    ids = allcfs.getSortedIds()
    cfs = [allcfs.crazyfliesById[i] for i in ids]

    nb_agents = len(ids)

    # get home positions
    home_positions = []
    for cf in cfs:
        home_positions.append(np.array(cf.initialPosition) + np.array([0, 0, Z_HOME]))

    # takeoff
    while Z < Z_HOME:
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
            cf.cmdPosition(pos)
        timeHelper.sleep(0.1)
        Z += 0.05

    # Compute helix trajectory
    t, x, y, z = uniform_helix(FREQ_PUB, 35)

    # Start trajectories
    for i in range(len(x)):
        for cf in allcfs.crazyflies:
            target_position = np.asarray([x[i], y[i], z[i]]) + home_positions[id_to_idx(cf.id)] - [x[0], y[0], z[0]]
            cf.cmdPosition(target_position)
        timeHelper.sleep(1/float(FREQ_PUB))

    # Go back to home positions
    Z = z[-1]

    # land
    while Z > 0.0:
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
            cf.cmdPosition(pos)
        timeHelper.sleep(0.1)
        Z -= 0.05

    # turn-off motors
    for cf in allcfs.crazyflies:
        cf.cmdStop()
