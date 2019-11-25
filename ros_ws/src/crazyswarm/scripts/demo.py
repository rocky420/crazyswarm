#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 1.0
NB_AGENTS = 6;

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Takeoff
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    # Create circle
    ids = range(NB_AGENTS) -1;
    cfs = [allcfs.crazyfliesById[i] for i in ids]
    for cf in allcfs.crazyflies:
        target_position = 0;
        pos = target_position + np.array([0, 0, Z])
        cf.goTo(pos, 0, 1.0)

    # print("press button to continue...")
    # swarm.input.waitUntilButtonPressed()

    # Landing
    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
