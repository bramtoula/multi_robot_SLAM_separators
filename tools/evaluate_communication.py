
path = '/Users/benjaminramtoula/Documents/Cours/POLYMTL/MISTLAB/SLAM/decentralized_slam_project/logs/'
dir = '2019-08-22_11-43-20_0.13_20/'

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
    r_s_a_timestamps = filter(lambda x: x.startswith('15'), lines)
    r_s_a_timestamps = [float(t[:-2]) for t in f_m_a_timestamps]
    r_s_a_number_of_kf_ids_from = filter(
        lambda x: x.startswith('number_of_kf_ids_from'), lines)
    r_s_a_number_of_kf_ids_from = [
        int(v.split()[1]) for v in f_m_a_number_of_kf_ids_from]