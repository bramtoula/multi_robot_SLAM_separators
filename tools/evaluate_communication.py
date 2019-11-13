import numpy as np

path = '/Users/benjaminramtoula/Documents/Cours/POLYMTL/MISTLAB/SLAM/decentralized_slam_project/logs/'
dir = '2019-08-22_11-43-20_0.13_20/'

timestamps_combined = []
nb_kf_used = []
data_exchanged_f_m_q = []
data_exchanged_f_m_a = []
data_exchanged_r_s_q = []

with open(path+dir+'find_matches_query.txt', 'r') as infile:
    lines = infile.readlines()
    f_m_q_timestamps = filter(lambda x: x.startswith('15'), lines)
    f_m_q_timestamps = [float(t[:-2]) for t in f_m_q_timestamps]
    f_m_q_number_of_values_in_descriptors = filter(
        lambda x: x.startswith('number_of_values'), lines)
    f_m_q_number_of_values_in_descriptors = [
        int(v.split()[1]) for v in f_m_q_number_of_values_in_descriptors]


with open(path+dir+'find_matches_answer.txt', 'r') as infile:
    lines = infile.readlines()
    f_m_a_timestamps = filter(lambda x: x.startswith('15'), lines)
    f_m_a_timestamps = [float(t[:-2]) for t in f_m_a_timestamps]
    f_m_a_number_of_kf_ids_computing_robot = filter(
        lambda x: x.startswith('number_of_kf_ids_computing_robot'), lines)
    f_m_a_number_of_kf_ids_computing_robot = [
        int(v.split()[1]) for v in f_m_a_number_of_kf_ids_computing_robot]


with open(path+dir+'find_matches_answer.txt', 'r') as infile:
    lines = infile.readlines()
    f_m_a_timestamps = filter(lambda x: x.startswith('15'), lines)
    f_m_a_timestamps = [float(t[:-2]) for t in f_m_a_timestamps]
    f_m_a_number_of_kf_ids_computing_robot = filter(
        lambda x: x.startswith('number_of_kf_ids_computing_robot'), lines)
    f_m_a_number_of_kf_ids_computing_robot = [
        int(v.split()[1]) for v in f_m_a_number_of_kf_ids_computing_robot]

    f_m_a_sizes_of_descriptors = filter(
        lambda x: x.startswith('sizes_of_descriptors'), lines)
    f_m_a_sizes_of_descriptors = [
        [int(v2) for v2 in v.split()[1:]] for v in f_m_a_sizes_of_descriptors]

    f_m_a_sizes_of_kpts3D = filter(
        lambda x: x.startswith('sizes_of_kpts3D'), lines)
    f_m_a_sizes_of_kpts3D = [
        [int(v2) for v2 in v.split()[1:]] for v in f_m_a_sizes_of_kpts3D]

with open(path+dir+'receive_separators_query.txt', 'r') as infile:
    lines = infile.readlines()
    r_s_q_timestamps = filter(lambda x: x.startswith('15'), lines)
    r_s_q_timestamps = [float(t[:-2]) for t in r_s_q_timestamps]
    r_s_q_number_of_kf_ids_from = filter(
        lambda x: x.startswith('number_of_kf_ids_from'), lines)
    r_s_q_number_of_kf_ids_from = [
        int(v.split()[1]) for v in r_s_q_number_of_kf_ids_from]


while (not f_m_q_timestamps[0] == np.inf) or (not r_s_q_timestamps[0] == np.inf) or (not f_m_a_timestamps[0] == np.inf):
    if (f_m_q_timestamps[0] < r_s_q_timestamps[0]) and (f_m_q_timestamps[0] < f_m_a_timestamps[0]):
        if not f_m_q_number_of_values_in_descriptors[0] or f_m_q_number_of_values_in_descriptors[0] == 0:
            f_m_q_timestamps.pop(0)
            f_m_q_number_of_values_in_descriptors.pop(0)
            if not f_m_q_timestamps:
                f_m_q_timestamps.append(np.inf)
            continue
        timestamps_combined.append(f_m_q_timestamps[0])
        nb_kf_used.append(int(f_m_q_number_of_values_in_descriptors[0]/128))
        data_exchanged_f_m_q.append(f_m_q_number_of_values_in_descriptors[0]*8)
        data_exchanged_f_m_a.append(0)
        data_exchanged_r_s_q.append(0)
        f_m_q_timestamps.pop(0)
        f_m_q_number_of_values_in_descriptors.pop(0)
        if not f_m_q_timestamps:
            f_m_q_timestamps.append(np.inf)

    elif (r_s_q_timestamps[0] < f_m_a_timestamps[0]):
        if not r_s_q_number_of_kf_ids_from[0]:
            r_s_q_timestamps.pop(0)
            r_s_q_number_of_kf_ids_from.pop(0)
            if not r_s_q_timestamps:
                r_s_q_timestamps.append(np.inf)
            continue
        timestamps_combined.append(r_s_q_timestamps[0])
        nb_kf_used.append(nb_kf_used[-1])
        data_exchanged_r_s_q.append(2+(8+344*3)*r_s_q_number_of_kf_ids_from[0])
        data_exchanged_f_m_a.append(0)
        data_exchanged_f_m_q.append(0)
        r_s_q_timestamps.pop(0)
        r_s_q_number_of_kf_ids_from.pop(0)
        if not r_s_q_timestamps:
            r_s_q_timestamps.append(np.inf)

    elif f_m_a_timestamps[0] < np.inf:
        if f_m_a_number_of_kf_ids_computing_robot[0] == 0 or not f_m_a_number_of_kf_ids_computing_robot[0] or not f_m_a_sizes_of_descriptors[0]:
            f_m_a_timestamps.pop(0)
            f_m_a_sizes_of_kpts3D.pop(0)
            f_m_a_sizes_of_descriptors.pop(0)
            f_m_a_number_of_kf_ids_computing_robot.pop(0)
            if not f_m_a_timestamps:
                f_m_a_timestamps.append(np.inf)
            continue
        timestamps_combined.append(f_m_a_timestamps[0])
        nb_kf_used.append(nb_kf_used[-1])

        # print(f_m_a_sizes_of_kpts3D[0])
        # print(f_m_a_number_of_kf_ids_computing_robot[0])
        data_exchanged_f_m_a.append(
            f_m_a_number_of_kf_ids_computing_robot[0]*(344+44*np.mean(f_m_a_sizes_of_kpts3D[0])+np.mean(f_m_a_sizes_of_descriptors[0])))
        data_exchanged_f_m_q.append(0)
        data_exchanged_r_s_q.append(0)

        f_m_a_timestamps.pop(0)
        f_m_a_sizes_of_kpts3D.pop(0)
        f_m_a_sizes_of_descriptors.pop(0)
        if not f_m_a_timestamps:
            f_m_a_timestamps.append(np.inf)

    else:
        break

# all_data_exchanged = data_exchanged_f_m_a + \
#     data_exchanged_f_m_q+data_exchanged_r_s_q
all_data_exchanged_f_m = [sum(x) for x in zip(
    data_exchanged_f_m_a, data_exchanged_f_m_q)]
all_data_exchanged = [sum(x) for x in zip(
    all_data_exchanged_f_m, data_exchanged_r_s_q)]

total_data_exchanged = np.cumsum(all_data_exchanged)

# import numpy as np
import matplotlib.pyplot as plt
import csv

# results = np.loadtxt(open(
#     "collected_data/parameter_effects/netvlad_data.csv", "rt"), delimiter=",", skiprows=0)

fig, ax = plt.subplots()

ax.fill_between(np.cumsum(nb_kf_used), 0, np.cumsum(data_exchanged_f_m_q)/(2**20),
                facecolor='#46237A', label='Sharing Netvlad descriptors')

ax.fill_between(np.cumsum(nb_kf_used), np.cumsum(data_exchanged_f_m_q)/(2**20), np.cumsum(all_data_exchanged_f_m)/(2**20),
                facecolor='#E84354', label='Answers to find matches')

ax.fill_between(np.cumsum(nb_kf_used), np.cumsum(all_data_exchanged_f_m)/(2**20), total_data_exchanged/(2**20),
                facecolor='#256EFF', label='Separators sent back')
# ax.fill_between(results[:, 0], results[:, 1] - results[:, 3], results[:, 1], where=results[:, 1] >= results[:, 2],
#                 facecolor='#E84354', label='Rejected')

ax.set_ylabel('Amount of data exchanged [MB]')
ax.set_xlabel('Number of keyframes seen by first robot')

ax.legend(loc='upper left')
ax.grid(True)

# ax.set_ylim(0, 120)
# ax.set_xlim(0.10, 0.15)

plt.show()
print()
