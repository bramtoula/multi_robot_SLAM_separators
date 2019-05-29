import numpy as np

f = open('00.txt')
lines = f.readlines()
closures = []
max_t_dist = 1
neighbors_skipped = 300

for i in range(len(lines)):
    T_ref = np.fromstring(lines[i], sep=' ').reshape(3, 4)
    R_ref = T_ref[:, :3]
    t_ref = T_ref[:, -1]
    print(i)

    for j in range(i+neighbors_skipped, len(lines)):
        T_cmp = np.fromstring(lines[j], sep=' ').reshape(3, 4)
        R_cmp = T_cmp[:, :3]
        t_cmp = T_cmp[:, -1]
        if np.abs(j-1) > neighbors_skipped:
            if (np.sum((t_ref-t_cmp)*(t_ref-t_cmp))) < max_t_dist:
                closures.append((i,j))

with open('kitti_closures.txt', 'w') as fp:
    fp.write('\n'.join('%s %s' % x for x in closures))


