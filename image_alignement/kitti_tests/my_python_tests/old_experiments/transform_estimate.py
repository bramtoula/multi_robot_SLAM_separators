import cv2
import numpy as np

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

def keepMatchedPoints(kp1,desc1,kp2,desc2,matches):
	points1 = np.zeros((len(matches), 2), dtype=np.float32)
	points2 = np.zeros((len(matches), 2), dtype=np.float32)
	for i, match in enumerate(matches):
	   points1[i, :] = kp1[match.queryIdx].pt
	   points2[i, :] = kp2[match.trainIdx].pt
	return points1, points2


def getMatches(img1,img2):
	# Convert to grayscale
	img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
	img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

	print("Aligning images ...")

	# Get keypoints and decriptors of each image
	orb = cv2.ORB_create()
	kp1, desc1 = orb.detectAndCompute(img1, None)
	kp2, desc2 = orb.detectAndCompute(img2, None)

	matches = getBestMatches(kp1, desc1, kp2, desc2)
	return matches, kp1, desc1, kp2, desc2

def getMatchesImg(img1, kp1, img2, kp2, matches, mask = []):

	if len(mask) > 0:
		mask = mask.ravel().tolist()

	out_img = np.empty((max(img1.shape[0], img2.shape[0]),
                     img1.shape[1]+img2.shape[1], 3), dtype=np.uint8)

	# Draw matches.
	img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches, out_img, matchesMask = mask, flags=2)

	# plt.imshow(img3), plt.show()
	return img3


def getBestMatches(kp1, desc1, kp2, desc2):
    """Returns top matches between two sets of keypoints and their descriptors

    Arguments:
        kp1 {keypoint} -- Keypoints from the first frame
        desc1 {descriptor} -- Descriptors associated to the keypoints kp1
        kp2 {keypoint} -- Keypoints from the second frame
        desc2 {descriptor} -- Descriptors associated to the keypoints kp2
    """

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # Match descriptors.
    matches = bf.match(desc1, desc2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key=lambda x: x.distance)

    return matches[:MATCHES_KEPT]

def get_transform(img1_name, img2_name):
	# Read reference image
	print("Reading reference image : ", img1_name)
	img1_l = cv2.imread(str(settings.sequence_path+'image_2/'+img1_name), cv2.IMREAD_COLOR)
	img1_r = cv2.imread(str(settings.sequence_path+'image_3/'+img1_name), cv2.IMREAD_COLOR)

	# Read image to be aligned
	print("Reading reference image : ", img2_name)
	img2_l = cv2.imread(str('image_2/'+img2_name), cv2.IMREAD_COLOR)
	img2_r = cv2.imread(str('image_3/'+img2_name), cv2.IMREAD_COLOR)

	matches_1_lr, kp_l_1, desc_l_1, kp_r_1, desc_r_1 = getMatches(img1_l,img1_r)
	matches_12_ll,kp_1_l,desc_1,kp_2,desc_2 = getMatches(img1_l,img2_l)

	points_l_1, points_r_1 = keepMatchedPoints(kp_l_1,desc_l_1,kp_r_1,desc_r_1,matches_1)

	calib = read_calib_file(settings.filepath+"calib.txt")
	proj_mat_l = calib['P2'].reshape(3,4)
	proj_mat_r = calib['P3'].reshape(3,4)

	cam_mat_l, rotMatrix_l, transVect_l, rotMatrixX_l, rotMatrixY_l, rotMatrixZ_l, eulerAngles_l = cv2.decomposeProjectionMatrix(
	    proj_mat_l)
	cam_mat_r, rotMatrix_r, transVect_r, rotMatrixX_r, rotMatrixY_r, rotMatrixZ_r, eulerAngles_r = cv2.decomposeProjectionMatrix(
	    proj_mat_r)

	points_l_1 = np.transpose(points_l_1)
	points_r_1 = np.transpose(points_r_1)

	points_3d = cv2.triangulatePoints(proj_mat_l,proj_mat_r,points_l_1,points_r_1)
	points_3d = cv2.convertPointsFromHomogeneous(np.transpose(points_3d))

	points_3d_kept = []

	im_points = []
	for match12 in matches_12:
		for i,match1 in enumerate(matches_1):
			if match1.queryIdx == match12.queryIdx:
				points_3d_kept.append(points_3d[i])
				im_points.append(kp_2[match12.trainIdx].pt)

	points_3d_kept = np.array(points_3d_kept).squeeze()
	im_points = np.array(im_points).squeeze()

	distortions = np.zeros(4)

	retval, rvec, tvec, inliers = cv2.solvePnPRansac(
	    points_3d_kept, im_points, cam_mat_l, distortions)
	rot_mat,_ = cv2.Rodrigues(rvec)
    if retval = 1:
        return rot_mat,tvec
    else:
        return [],[]