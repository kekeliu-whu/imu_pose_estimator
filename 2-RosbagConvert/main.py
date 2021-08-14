import sys

import pandas
import rosbag
from sensor_msgs.msg import Imu


def convert_text_to_bag(txt_fn, bag_fn):
    df = pandas.read_csv(txt_fn)

    bag = rosbag.Bag(bag_fn, mode='w', compression=rosbag.Compression.LZ4)
    try:
        seq = 0
        for _, e in df.iterrows():
            imu = Imu()
            imu.header.seq = seq
            a = e['timestamp']
            imu.header.stamp.set(int(e['timestamp']) // 1000_000_000, int(e['timestamp']) % 1000_000_000)
            imu.header.frame_id = "/base_link"
            imu.linear_acceleration.x = e['ax']
            imu.linear_acceleration.y = e['ay']
            imu.linear_acceleration.z = e['az']
            imu.angular_velocity.x = e['gx']
            imu.angular_velocity.y = e['gy']
            imu.angular_velocity.z = e['gz']

            bag.write('/imu', imu, imu.header.stamp)

            seq = seq + 1
    finally:
        bag.close()


pass

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: python3 main.py /path/to/imu.txt")
        exit(1)
    convert_text_to_bag(sys.argv[1], sys.argv[1] + ".bag")
