#!/usr/bin/env python

import numpy as np
import pandas as pd
from pycrazyswarm import *
import uav_trajectory

Z = 1.0
NB_AGENTS = 6
TIMESCALE = 3.0

def id_to_idx(id):
    return id-1


def idx_to_id(idx):
    return idx+1


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    ids = np.asarray(range(NB_AGENTS)) + 1
    cfs = [allcfs.crazyfliesById[i] for i in ids]

    # Takeoff
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    # Get circle target points from csv
    df = pd.read_csv("demo/target1.csv", skiprows=0)
    target_positions = df.values[:, 1::]

    # Go to circle positions
    for cf in allcfs.crazyflies:
        target_position = target_positions[id_to_idx(cf.id)]
        cf.goTo(target_position, 0, 2.0 * TIMESCALE)
    timeHelper.sleep(2.5)

    # Move in circle
    root = 'demo/output_circle'
    fnames = ['{0}/circle{1}.csv'.format(root, i) for i in ids]
    T = 0
    for cf, fname in zip(cfs, fnames):
        traj = uav_trajectory.Trajectory()
        traj.loadcsv(fname)
        cf.uploadTrajectory(0, 0, traj)
        T = max(T, traj.duration)
    allcfs.startTrajectory(0, relative=True, timescale=TIMESCALE)
    timeHelper.sleep(T + 3.0)
    # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
    # timeHelper.sleep(T + 3.0)


    # Split in two groups
    group1 = [1, 3, 5]
    group2 = [2, 4, 6]
    # allcfs.crazyfliesById[group1].setGroupMask(1)
    # allcfs.crazyfliesById[group2].setGroupMask(2)

    for cf in cfs:
        pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
        cf.goTo(pos, 0, 5.0)
    timeHelper.sleep(5.5)

    # Landing
    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
