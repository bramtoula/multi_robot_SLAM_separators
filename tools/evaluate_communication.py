import numpy as np

path = '/Users/benjaminramtoula/Documents/Cours/POLYMTL/MISTLAB/SLAM/decentralized_slam_project/logs/'
dir = '2019-08-22_11-43-20_0.13_20/'

timestamp_combined = []
nb_kf_used = []
data_exchanged = []

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
        timestamp_combined.append(f_m_q_timestamps[0])
        nb_kf_used.append(int(f_m_q_number_of_values_in_descriptors[0]/128))
        data_exchanged.append(f_m_q_number_of_values_in_descriptors[0]*8)
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
        timestamp_combined.append(r_s_q_timestamps[0])
        nb_kf_used.append(nb_kf_used[-1])
        data_exchanged.append(2+(8+344*3)*r_s_q_number_of_kf_ids_from[0])
        r_s_q_timestamps.pop(0)
        r_s_q_number_of_kf_ids_from.pop(0)
        if not r_s_q_timestamps:
            r_s_q_timestamps.append(np.inf)

    elif f_m_a_timestamps[0] < np.inf:
        if f_m_a_number_of_kf_ids_computing_robot[0] == 0:
            f_m_a_timestamps.pop(0)
            f_m_a_sizes_of_kpts3D.pop(0)
            f_m_a_sizes_of_descriptors.pop(0)
            if not f_m_a_timestamps:
                f_m_a_timestamps.append(np.inf)
            continue
        timestamp_combined.append(f_m_a_timestamps[0])
        nb_kf_used.append(nb_kf_used[-1])

        # print(f_m_a_sizes_of_kpts3D[0])
        data_exchanged.append(
            f_m_a_number_of_kf_ids_computing_robot[0]*(344+44*np.mean(f_m_a_sizes_of_kpts3D[0])+np.mean(f_m_a_sizes_of_descriptors[0])))

        f_m_a_timestamps.pop(0)
        f_m_a_sizes_of_kpts3D.pop(0)
        f_m_a_sizes_of_descriptors.pop(0)
        if not f_m_a_timestamps:
            f_m_a_timestamps.append(np.inf)

    else:
        break


print(data_exchanged)
print(np.cumsum(data_exchanged))