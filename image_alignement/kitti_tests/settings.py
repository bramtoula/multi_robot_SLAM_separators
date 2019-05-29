"""File that loads experiment specific parameters.
"""


def init():
    global sequence_path, matches_kept, min_points_pnp
    sequence_path = '/Users/benjaminramtoula/Documents/Cours/POLYMTL/MISTLAB/SLAM/datasets/kitti/00_color/'
    matches_kept = 100
    min_points_pnp = 8
