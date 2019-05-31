"""File that loads experiment specific parameters.
"""


def init():
    global sequence_path, matches_kept, min_inliers, iterations, epipolar_geometry_var, pnp_reproj_err, pnp_flags
    sequence_path = '/Users/benjaminramtoula/Documents/Cours/POLYMTL/MISTLAB/SLAM/datasets/kitti/00_color/'
    matches_kept = 10
    min_inliers = 5
    iterations = 300
    epipolar_geometry_var = 0.02
    pnp_reproj_err = 2.0
    pnp_flags = 0