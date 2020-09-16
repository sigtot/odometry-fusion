from typing import List

import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
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


def get_odo_path(topic: str) -> List[PoseWithCovariance]:
    with rosbag.Bag('../src/odometry_optimizer/rosbag/odometries.bag') as bag:
        poses = []
        for topic, msg, t in bag.read_messages(topics=[topic]):
            poses.append(msg)
        return poses


def get_lidar_odo_path() -> List[PoseWithCovariance]:
    return get_odo_path('/aft_mapped_to_init_CORRECTED')


def get_imu_odo_path() -> List[PoseWithCovariance]:
    return get_odo_path('/rovio/odometry')


def plot_2d(xs, ys, x_label, y_label, label):
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.plot(ys, xs, label=label)
    plt.legend()


def plot():
    path = get_last_path()
    stamped_poses: List[PoseStamped] = path.poses
    xs = [stamped_pose.pose.position.x for stamped_pose in stamped_poses]
    ys = [stamped_pose.pose.position.y for stamped_pose in stamped_poses]
    zs = [stamped_pose.pose.position.z for stamped_pose in stamped_poses]

    lidar_poses = get_lidar_odo_path()
    xs_lidar = [pose_with_cov.pose.pose.position.x for pose_with_cov in lidar_poses]
    ys_lidar = [pose_with_cov.pose.pose.position.y for pose_with_cov in lidar_poses]
    zs_lidar = [pose_with_cov.pose.pose.position.z for pose_with_cov in lidar_poses]

    imu_poses = get_imu_odo_path()
    xs_imu = [pose_with_cov.pose.pose.position.x for pose_with_cov in imu_poses]
    ys_imu = [pose_with_cov.pose.pose.position.y for pose_with_cov in imu_poses]
    zs_imu = [pose_with_cov.pose.pose.position.z for pose_with_cov in imu_poses]

    plt.figure()
    plot_2d(xs, ys, 'x', 'y', 'fused')
    plot_2d(xs_lidar, ys_lidar, 'x', 'y', 'lidar')
    plot_2d(xs_imu, ys_imu, 'x', 'y', 'imu')
    plt.savefig('xy_path.pdf')
    plt.figure()
    plot_2d(xs, zs, 'x', 'z', 'fused')
    plot_2d(xs_lidar, zs_lidar, 'x', 'z', 'lidar')
    plot_2d(xs_imu, zs_imu, 'x', 'z', 'imu')
    plt.savefig('xz_path.pdf')
    plt.figure()
    plot_2d(zs, ys, 'z', 'y', 'fused')
    plot_2d(zs_lidar, ys_lidar, 'z', 'y', 'lidar')
    plot_2d(zs_imu, ys_imu, 'z', 'y', 'imu')
    plt.savefig('zy_path.pdf')


if __name__ == "__main__":
    plot()
