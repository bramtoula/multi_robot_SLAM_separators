from get_groundtruth_transform import getGtTransform
from get_transform_est import getEstTransform
import numpy as np


# Given two frames, compares the estimated translation and rotation to the groundtruth
def compare_est_gt(image_1_idx,image_2_idx,print_res=False):
    image_1_name = str(image_1_idx).zfill(6)+'.png'
    image_2_name = str(image_2_idx).zfill(6)+'.png'

    R_12_est,t_12_est = getEstTransform(image_1_name,image_2_name)
    if R_12_est.all() == 0:
        return np.inf, np.inf
    R_12_gt,t_12_gt = getGtTransform(int(image_1_idx),int(image_2_idx))
    R_01_gt, t_01_gt = getGtTransform(0, int(image_1_idx))

    t_12_est_ref_global = -np.dot(R_01_gt, t_12_est).squeeze()
    t_err = np.sqrt(np.sum((t_12_gt-t_12_est_ref_global)**2))

    R_12_est_t = np.transpose(R_12_est)
    R_est_gt = np.dot(R_12_est_t,R_12_gt)
    Rot_err = np.rad2deg(np.arccos((np.trace(R_est_gt) - 1) / 2))

    if print_res:
        print('My estimate (left camera coordinate frame)')
        print(t_12_est.squeeze())
        print('My estimate (same frame as gt)')
        print(t_12_est_ref_global)
        print('Groundtruth')
        print(t_12_gt)
        print('\n')

        print('Translation error:'+str(t_err)+'\n')
        print(Rot_err)
    return Rot_err, t_err
