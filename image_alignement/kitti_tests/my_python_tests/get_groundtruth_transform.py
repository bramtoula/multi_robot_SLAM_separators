import numpy as np



def getGtTransform(img_idx1, img_idx2):
    # In the reference frame of the left camera
    f = open('../kitti_data/00.txt')
    lines = f.readlines()

    T1 = np.fromstring(lines[img_idx1],sep=' ').reshape(3,4)
    R1 = T1[:,:3]
    t1 = T1[:,-1]

    T2 = np.fromstring(lines[img_idx2], sep=' ').reshape(3, 4)
    R2 = T2[:, :3]
    t2 = T2[:, -1]

    return np.dot(R1,np.linalg.inv(R2)), t2-t1
