import cv2
from matplotlib import pyplot as plt
import numpy as np
MATCHES_KEPT = 500

def drawComparedMatches(img1,img2):
	f, axarr = plt.subplots(2, 1)
	axarr[0].imshow(img1)
	axarr[1].imshow(img2)
	plt.show()

def getHomography(kp1,desc1,kp2,desc2,matches):
	points1 = np.zeros((len(matches), 2), dtype=np.float32)
	points2 = np.zeros((len(matches), 2), dtype=np.float32)
	for i, match in enumerate(matches):
	   points1[i, :] = kp1[match.queryIdx].pt
	   points2[i, :] = kp2[match.trainIdx].pt
	
	H, mask = cv2.findHomography(points1,points2,cv2.RANSAC)

	return H, mask

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

if __name__ == '__main__':
	# Read reference image
	img1_name = "000000.png"
	print("Reading reference image : ", img1_name)
	img1 = cv2.imread(img1_name, cv2.IMREAD_COLOR)

	# Read image to be aligned
	img2_name = "000003.png"
	print("Reading image to align : ", img2_name)
	img2 = cv2.imread(img2_name, cv2.IMREAD_COLOR)

	matches, kp1, desc1, kp2, desc2 = getMatches(img1,img2)

	img_all_matches = getMatchesImg(img1, kp1, img2, kp2, matches)

	# Compute homography
	H, mask = getHomography(kp1, desc1, kp2, desc2, matches)
	img_matches_filtered = getMatchesImg(img1, kp1, img2, kp2, matches,mask)

	# drawComparedMatches(img_all_matches, img_matches_filtered)
