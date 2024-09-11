import rosbag
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    bag_path = "/home/oskar/smooth_sailing/data/ws_left3/raw.bag"

    bag = rosbag.Bag(bag_path)
    bag_info = bag.get_type_and_topic_info()

    imu_topic = None
    msg_cnt = 0
    for topic, topic_tuple in bag_info.topics.items():
        if topic_tuple.msg_type == "sensor_msgs/Imu":
            imu_topic = topic
            msg_cnt = topic_tuple.message_count
            break

    
    if imu_topic == None:
        print("No imu topic in bag, exiting")
        exit()
    
    acc = np.zeros((msg_cnt, 3))
    rate = np.zeros((msg_cnt, 3))
    ts = np.zeros(msg_cnt)
    
    cnt = 0
    for _, msg, _ in bag.read_messages(topics=[imu_topic]):
        ts[cnt] = msg.header.stamp.to_sec()
        
        acc[cnt, 0] = msg.linear_acceleration.x
        acc[cnt, 1] = msg.linear_acceleration.y
        acc[cnt, 2] = msg.linear_acceleration.z

        rate[cnt, 0] = msg.angular_velocity.x
        rate[cnt, 1] = msg.angular_velocity.y
        rate[cnt, 2] = msg.angular_velocity.z


        cnt += 1

    ts = ts[:cnt]
    acc = acc[:cnt]
    rate = rate[:cnt]
    
    print("Average rate: ", rate.mean(axis=0), "+-", rate.std(axis=0))
    print("Average acc:", acc.mean(axis=0), "+-", acc.std(axis=0))

    plt.figure("Gyro")
    plt.plot(ts, rate)

    plt.figure("Acceleration")
    plt.plot(ts, acc)

    
    plt.show()
