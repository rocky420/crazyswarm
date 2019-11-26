import numpy as np
import os

NB_AGENTS = 6

if __name__ == "__main__":

    # Parameters
    root = 'demo/input_circle'
    ids = np.arange(1,NB_AGENTS+1)

    # Convert waypoints to circular trajectories
    fnames = ['{0}/circle{1}_wpts.csv'.format(root, i) for i in ids]
    for id, fname in zip(ids, fnames):
        output_fname = 'demo/output_circle/circle{0}.csv'.format(id)
        command = 'python3 ../../../../../uav_trajectories/scripts/generate_trajectory.py {0} {1} --pieces 5'.format(fname, output_fname)
        os.system(command)
