import numpy as np
import matplotlib.pyplot as plt
from compare_est_gt import compare_est_gt
import settings

closure_ids = np.genfromtxt('../kitti_data/kitti_closures.txt',dtype=int)
t_errors = []
Rot_errors = []
i = 0
rejected = 0
t_lim = 10.0
Rot_lim = 45
for id_pair in closure_ids:

    # if i < 1300:
    #     i += 1
    #     continue
    print('Working on closure '+str(i)+' of '+str(len(closure_ids)))
    id1 = id_pair[0]
    id2 = id_pair[1]
    Rot_err, t_err = compare_est_gt(id1,id2,print_res=False)
    if t_err < t_lim and Rot_err < Rot_lim:
        Rot_errors.append(Rot_err)
        t_errors.append(t_err)
    else:
        rejected += 1
        print("Rejected!")
        print(id1)
        print(id2)
    i += 1


print("Number of rejected closures: "+str(rejected)+"/"+str(i))
print("Average translation error: "+str(np.mean(t_errors)))
print("Average rotation error: "+str(np.mean(Rot_errors)))
plt.hist(t_errors,bins=100)
plt.show()
plt.hist(Rot_errors,bins = 100)
plt.show()
