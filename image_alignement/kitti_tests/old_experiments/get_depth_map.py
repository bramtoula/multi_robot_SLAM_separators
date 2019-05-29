import numpy as np
import cv2
from matplotlib import pyplot as plt


def read_calib_file(filepath):
    """Read in a calibration file and parse into a dictionary."""
    data = {}

    with open(filepath, 'r') as f:
        for line in f.readlines():
            key, value = line.split(':', 1)
            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass

    return data


# img1_name = "002460.png"
img1_name = "000000.png"
print("Reading reference image : ", img1_name)
imgL = cv2.imread(str('image_2/'+img1_name), 0)
imgR = cv2.imread(str('image_3/'+img1_name), 0)

stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
disparity = stereo.compute(imgL, imgR)
# plt.imshow(disparity,'gray')
# plt.show()

calib = read_calib_file("calib.txt")
proj_mat_l = calib['P2'].reshape(3, 4)
proj_mat_r = calib['P3'].reshape(3, 4)

cam_mat_l, rotMatrix_l, transVect_l, rotMatrixX_l, rotMatrixY_l, rotMatrixZ_l, eulerAngles_l = cv2.decomposeProjectionMatrix(
    proj_mat_l)
cam_mat_r, rotMatrix_r, transVect_r, rotMatrixX_r, rotMatrixY_r, rotMatrixZ_r, eulerAngles_r = cv2.decomposeProjectionMatrix(
    proj_mat_r)

distortions = np.zeros(4)

transVect_l = cv2.convertPointsFromHomogeneous(np.transpose(transVect_l))
transVect_r = cv2.convertPointsFromHomogeneous(np.transpose(transVect_r))
R_stereo = np.dot(rotMatrix_r,np.linalg.inv(rotMatrix_l))
t_stereo = (transVect_r - transVect_l).squeeze()

R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(cam_mat_l,distortions,cam_mat_r,distortions,imgL.shape,R_stereo,t_stereo)

depth_map = cv2.reprojectImageTo3D(disparity,Q,handleMissingValues=True)
print(depth_map.shape)
plt.imshow(depth_map[:,:,-1],'gray')
plt.show()
