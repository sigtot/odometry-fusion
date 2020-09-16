from typing import List

import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt


def get_last_path() -> Path:
    with rosbag.Bag('../src/odometry_optimizer/rosbag/paths.bag') as bag:
        last_msg = None
        for topic, msg, t in bag.read_messages(topics=['/optimized_path']):
            last_msg = msg
        if last_msg is not None:
            return last_msg
        else:
            raise Exception("Got no messages from rosbag")


def plot_2d(xs, ys, x_label, y_label, fig_path):
    plt.figure()
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.plot(ys, xs)
    plt.savefig(fig_path)


def plot():
    path = get_last_path()
    stamped_poses: List[PoseStamped] = path.poses
    xs = [stamped_pose.pose.position.x for stamped_pose in stamped_poses]
    ys = [stamped_pose.pose.position.y for stamped_pose in stamped_poses]
    zs = [stamped_pose.pose.position.z for stamped_pose in stamped_poses]
    plot_2d(xs, ys, 'x', 'y', 'xy_path.pdf')
    plot_2d(xs, zs, 'x', 'z', 'xz_path.pdf')
    plot_2d(zs, ys, 'z', 'y', 'zy_path.pdf')


if __name__ == "__main__":
    plot()
