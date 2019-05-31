import cv2
import settings
import numpy as np
def readCalibFile():
    """Read in a calibration file and parse into a dictionary."""
    data = {}
    filepath = settings.sequence_path+'calib.txt'
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

settings.init()
calib = readCalibFile()
proj_mat_l = calib['P2'].reshape(3, 4)
proj_mat_r = calib['P3'].reshape(3, 4)

cam_mat_l, rotMatrix_l, transVect_l, rotMatrixX_l, rotMatrixY_l, rotMatrixZ_l, eulerAngles_l = cv2.decomposeProjectionMatrix(proj_mat_l)
cam_mat_r, rotMatrix_r, transVect_r, rotMatrixX_r, rotMatrixY_r, rotMatrixZ_r, eulerAngles_r = cv2.decomposeProjectionMatrix(proj_mat_r)

print(cam_mat_l)
print(rotMatrix_l)
print(cv2.convertPointsFromHomogeneous(transVect_l.transpose()))
print('\n')

print(cam_mat_r)
print(rotMatrix_r)
print(cv2.convertPointsFromHomogeneous(transVect_r.transpose()))
