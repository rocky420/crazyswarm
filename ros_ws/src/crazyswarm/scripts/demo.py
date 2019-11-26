#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import pandas as pd

Z = 1.0
NB_AGENTS = 6


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
        cf.goTo(target_position, 0, 2.0)
    timeHelper.sleep(2.5)

    # Landing
    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
