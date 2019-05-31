import cv2
import numpy as np
import settings
from matplotlib import pyplot as plt

def getMatchesImg(img1, kp1, img2, kp2, matches, mask=[]):

	if len(mask) > 0:
		mask = mask.ravel().tolist()

	out_img = np.empty((max(img1.shape[0], img2.shape[0]),
                     img1.shape[1]+img2.shape[1], 3), dtype=np.uint8)

	# Draw matches.
	img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches,
	                       out_img, matchesMask=mask, flags=2)

	plt.imshow(img3), plt.show()
	return img3


def get3dPoints(matches, kp_l, kp_r, proj_mat_l, proj_mat_r):
    px_left = np.array([kp_l[m.queryIdx].pt for m in matches])
    px_right = np.array([kp_r[m.trainIdx].pt for m in matches])

    points = cv2.triangulatePoints(
        proj_mat_l, proj_mat_r,
        px_left.transpose(),
        px_right.transpose()
    ).transpose()  # shape: (N, 4)
    points = cv2.convertPointsFromHomogeneous(points).squeeze()

    cam_mat_l, rotMatrix_l, transVect_l, rotMatrixX_l, rotMatrixY_l, rotMatrixZ_l, eulerAngles_l = cv2.decomposeProjectionMatrix(proj_mat_l)
    reproj_points, jacobian = cv2.projectPoints(points,cv2.Rodrigues(rotMatrix_l)[0],cv2.convertPointsFromHomogeneous(transVect_l.transpose()),cam_mat_l,None)

    reproj_points = reproj_points.squeeze()
    reproj_dist = np.sqrt(np.sum((px_left - reproj_points)**2, axis=1))
    valid = reproj_dist < 4


    return points, valid

def getProjMatrices():
    calib = readCalibFile()
    proj_mat_l = calib['P2'].reshape(3, 4)
    proj_mat_r = calib['P3'].reshape(3, 4)
    return proj_mat_l, proj_mat_r

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

def getKpDesc(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create()
    kp, desc = orb.detectAndCompute(img, None)
    return kp,desc


def getMatches(bf,kp1, desc1, kp2, desc2, matching_distance=40,
               max_row_distance=2.5,
               max_disparity=100):
    """Returns top matches between two sets of keypoints and their descriptors

    Arguments:
        kp1 {keypoint} -- Keypoints from the first frame
        desc1 {descriptor} -- Descriptors associated to the keypoints kp1
        kp2 {keypoint} -- Keypoints from the second frame
        desc2 {descriptor} -- Descriptors associated to the keypoints kp2
    """


    # Match descriptors.
    matches = bf.match(desc1, desc2)

    # Filter if row too far, from  https://github.com/uoip/stereo_ptam
    kept = []
    for m in matches:
        pt1 = kp1[m.queryIdx].pt
        pt2 = kp2[m.trainIdx].pt
        if (m.distance < matching_distance and abs(pt1[1] - pt2[1]) < max_row_distance and abs(pt1[0]-pt2[0] < max_disparity)):
            kept.append(m)
    # Sort them in the order of their distance.
    matches = sorted(kept, key=lambda x: x.distance)

    return matches#[:settings.matches_kept]

# From  https://github.com/uoip/stereo_ptam
def circular_stereo_match(
        matcher,
        desps1, desps2, matches12,
        desps3, desps4, matches34,
        matching_distance=50,
        min_matches=10, ratio=0.8):

    dict_m13 = dict()
    dict_m24 = dict()
    dict_m34 = dict((m.queryIdx, m) for m in matches34)

    ms13 = matcher.match(desps1, desps3)
    for m in ms13:
        if m.distance < matching_distance:
            dict_m13[m.queryIdx] = m

    # to avoid unnecessary computation
    if len(dict_m13) < min_matches:
        return []

    ms24 = matcher.match(desps2, desps4)
    for m in ms24:
        if m.distance < matching_distance:
            dict_m24[m.queryIdx] = m

    matches = []
    for m in matches12:
        shared13 = dict_m13.get(m.queryIdx, None)
        shared24 = dict_m24.get(m.trainIdx, None)

        if shared13 is not None and shared24 is not None:
            shared34 = dict_m34.get(shared13.trainIdx, None)
            if (shared34 is not None and
                    shared34.trainIdx == shared24.trainIdx):
                matches.append((shared13, shared24))
    return matches

def getEstTransform(img1_name,img2_name):
    
    settings.init()
    # Read reference image
    # print("Reading reference image : ", img1_name)
    img1_l = cv2.imread(
	    str(settings.sequence_path+'image_2/'+img1_name), cv2.IMREAD_COLOR)
    img1_r = cv2.imread(
        str(settings.sequence_path+'image_3/'+img1_name), cv2.IMREAD_COLOR)

    # Read image to be aligned
    # print("Reading reference image : ", img2_name)
    img2_l = cv2.imread(
        str(settings.sequence_path+'image_2/'+img2_name), cv2.IMREAD_COLOR)
    img2_r = cv2.imread(
        str(settings.sequence_path+'image_3/'+img2_name), cv2.IMREAD_COLOR)


    # Get keypoints and descriptors
    kp1_l,desc1_l = getKpDesc(img1_l)
    kp1_r,desc1_r = getKpDesc(img1_r)
    kp2_l,desc2_l = getKpDesc(img2_l)
    kp2_r,desc2_r = getKpDesc(img2_r)

    # Get matches
    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches1_lr = getMatches(bf, kp1_l,desc1_l,kp1_r,desc1_r)
    matches2_lr = getMatches(bf, kp2_l, desc2_l, kp2_r, desc2_r)
    matches12_l = getMatches(bf, kp1_l, desc1_l, kp2_l, desc2_l)
    matches12_r = getMatches(bf, kp1_r, desc1_r, kp2_r, desc2_r)

    # getMatchesImg(img1_l, kp1_l, img1_r, kp1_r, matches1_lr, mask=[])



    # Triangulate to get 3D points of both pairs
    # Get projection matrices
    proj_mat_l, proj_mat_r = getProjMatrices()
    points_3d_1, valid_1 = get3dPoints(
        matches1_lr, kp1_l, kp1_r, proj_mat_l, proj_mat_r)
    points_3d_2, valid_2 = get3dPoints(
        matches2_lr, kp2_l, kp2_r, proj_mat_l, proj_mat_r)


    # Get matches between the pairs of stereo images
    matches_stereo = circular_stereo_match(
        bf, desc1_l, desc1_r, matches1_lr, desc2_l, desc2_r,matches2_lr)

    # Extract 3d points from first pair and matching 2d points from second pair

    points_3d_1_kept = []
    points_3d_2_kept = []
    points_2d_1_kept = []
    points_2d_2_kept = []

    for m_l,m_r in matches_stereo:
        i_l,j_l = m_l.queryIdx, m_l.trainIdx
        i_r,j_r = m_r.queryIdx, m_r.trainIdx
        for  i, match in enumerate(matches1_lr):
            if match.queryIdx == i_l:
                # print(points_3d_1[i])
                # print(kp1_l[i].pt)
                # print('\n') 
                if valid_1[i]:
                    points_3d_1_kept.append(points_3d_1[i])
                    points_2d_2_kept.append(kp2_l[j_l].pt)
                    continue

    if len(points_3d_1_kept) < settings.min_inliers:
        return np.zeros((3,3)),0
    # Get the intrinsics matrix
    cam_mat_l, rotMatrix_l, transVect_l, rotMatrixX_l, rotMatrixY_l, rotMatrixZ_l, eulerAngles_l = cv2.decomposeProjectionMatrix(
    proj_mat_l)
    # print(cam_mat_l)
    cam_mat_r, rotMatrix_r, transVect_r, rotMatrixX_r, rotMatrixY_r, rotMatrixZ_r, eulerAngles_r = cv2.decomposeProjectionMatrix(
    proj_mat_r)
    # Points in the reference frame based on the left camera
    points_3d_1_kept_left_cor = points_3d_1_kept - cv2.convertPointsFromHomogeneous(transVect_l.transpose())
    # print(points_3d_1_kept_left_cor)
    # print(points_2d_2_kept)
    # Get the rotation and translation
    # print(points_3d_1_kept)
    val, rvec, tvec, inliers = cv2.solvePnPRansac(
        np.array(points_3d_1_kept_left_cor), np.array(points_2d_2_kept),
        cam_mat_l, None, None, None,
        False, settings.iterations, settings.pnp_reproj_err, 0.99, settings.min_inliers, settings.pnp_flags)
    rot_mat, _ = cv2.Rodrigues(rvec)
    return rot_mat, tvec
