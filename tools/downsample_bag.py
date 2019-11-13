import rosbag


downsample_factor = 3
outbag_path = '/datasets/kitti/dataset/kitti_data_odometry_color_sequence_00_part1_filtered.bag'
inbag_path = '/datasets/kitti/dataset/kitti_data_odometry_color_sequence_00_part1.bag'


topics_list = []
counter_per_topic = []


with rosbag.Bag(outbag_path, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(inbag_path).read_messages():
        if topic not in topics_list:
            topics_list.append(topic)
            counter_per_topic.append(downsample_factor)

        if counter_per_topic[topics_list.index(topic)] == downsample_factor:
            counter_per_topic[topics_list.index(topic)] = 0
            outbag.write(topic, msg, t) 

        counter_per_topic[topics_list.index(topic)] += 1


