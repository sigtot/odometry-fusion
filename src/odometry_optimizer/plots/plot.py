from typing import List

import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
import matplotlib.pyplot as plt
import math
from mpl_toolkits.axes_grid1 import make_axes_locatable

import sys

bag_path = '../rosbag/result_odometries.bag'


def get_last_path() -> Path:
    with rosbag.Bag(bag_path) as bag:
        last_fused_path = None
        for topic, msg, t in bag.read_messages(topics=['/optimized_path']):
            last_fused_path = msg
        if last_fused_path is not None:
            return last_fused_path
        else:
            raise Exception("Got no messages from rosbag")


def get_odo_path(topic: str) -> List[PoseWithCovariance]:
    with rosbag.Bag(bag_path) as bag:
        poses = []
        for topic, msg, t in bag.read_messages(topics=[topic]):
            poses.append(msg)
        return poses


def get_lidar_odo_path() -> List[PoseWithCovariance]:
    return get_odo_path('/aft_mapped_to_init_CORRECTED')


def get_rovio_odo_path() -> List[PoseWithCovariance]:
    return get_odo_path('/rovio/odometry')


def plot_2d(xs, ys, x_label, y_label, label):
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.plot(ys, xs, label=label)
    plt.legend()


def plot_2d_handle(handle, xs, ys, x_label, y_label, label, legend=True):
    handle.set_xlabel(x_label)
    handle.set_ylabel(y_label)
    handle.plot(ys, xs, label=label)
    if legend:
        handle.legend()

def dist(point):
    return math.sqrt(point.x**2 + point.y**2 + point.z**2)

def plot_rovio_v_loam():
    lidar_poses = get_lidar_odo_path()
    xs_lidar = [pose_with_cov.pose.pose.position.x for pose_with_cov in lidar_poses]
    ys_lidar = [pose_with_cov.pose.pose.position.y for pose_with_cov in lidar_poses]
    zs_lidar = [pose_with_cov.pose.pose.position.z for pose_with_cov in lidar_poses]

    first_lidar_stamp = lidar_poses[0].header.stamp.to_sec()
    ts_lidar = [pose_with_cov.header.stamp.to_sec() - first_lidar_stamp for pose_with_cov in lidar_poses]

    dist_lidar = [dist(pose_with_cov.pose.pose.position) for pose_with_cov in lidar_poses]

    rovio_poses = get_rovio_odo_path()
    xs_rovio = [pose_with_cov.pose.pose.position.x for pose_with_cov in rovio_poses]
    ys_rovio = [pose_with_cov.pose.pose.position.y for pose_with_cov in rovio_poses]
    zs_rovio = [pose_with_cov.pose.pose.position.z for pose_with_cov in rovio_poses]

    first_rovio_stamp = rovio_poses[0].header.stamp.to_sec()
    ts_rovio = [pose_with_cov.header.stamp.to_sec() - first_rovio_stamp for pose_with_cov in rovio_poses]

    dist_rovio = [dist(pose_with_cov.pose.pose.position) for pose_with_cov in rovio_poses]

    f, a0 = plt.subplots(figsize=(6, 2))
    plot_2d_handle(a0, ys_rovio, xs_rovio, 'y [m]', 'x [m]', 'ROVIO')
    plot_2d_handle(a0, ys_lidar, xs_lidar, 'y [m]', 'x [m]', 'LOAM')
    yticks = [0, 5, 10, 15]
    a0.set_yticks(yticks)

    divider = make_axes_locatable(a0)
    a1 = divider.append_axes("right", size=1.8, pad=0.7)

    plot_2d_handle(a1, dist_rovio, ts_rovio, 't [s]', 'distance [m]', 'ROVIO', False)
    plot_2d_handle(a1, dist_lidar, ts_lidar, 't [s]', 'distance [m]', 'LOAM', False)

    yticks = [0, 20, 40, 60, 80]
    xticks = [0, 20, 40, 60, 80, 100, 120]

    a1.set_xticks(xticks)
    a1.set_xticklabels(xticks, rotation=40)

    a1.set_yticks(yticks)

    #plt.gca().set_aspect('equal', adjustable='box')

    f.tight_layout()

    f.savefig('rovio_v_loam_dist_and_xy.pdf')


    plt.figure()
    plot_2d(zs_rovio, ts_rovio, 't [s]', 'z [m]', 'ROVIO')
    plot_2d(zs_lidar, ts_lidar, 't [s]', 'z [m]', 'LOAM')
    plt.savefig('rovio_v_loam_z_time.pdf')


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

    rovio_poses = get_rovio_odo_path()
    xs_rovio = [pose_with_cov.pose.pose.position.x for pose_with_cov in rovio_poses]
    ys_rovio = [pose_with_cov.pose.pose.position.y for pose_with_cov in rovio_poses]
    zs_rovio = [pose_with_cov.pose.pose.position.z for pose_with_cov in rovio_poses]

    plt.figure()
    plot_2d(xs, ys, 'x', 'y', 'fused')
    plot_2d(xs_lidar, ys_lidar, 'x', 'y', 'LOAM')
    plot_2d(xs_rovio, ys_rovio, 'x', 'y', 'ROVIO')
    plt.savefig('xy_path.pdf')
    plt.figure()
    plot_2d(xs, zs, 'x', 'z', 'fused')
    plot_2d(xs_lidar, zs_lidar, 'x', 'z', 'LOAM')
    plot_2d(xs_rovio, zs_rovio, 'x', 'z', 'ROVIO')
    plt.savefig('xz_path.pdf')
    plt.figure()
    plot_2d(zs, ys, 'z', 'y', 'fused')
    plot_2d(zs_lidar, ys_lidar, 'z', 'y', 'LOAM')
    plot_2d(zs_rovio, ys_rovio, 'z', 'y', 'ROVIO')
    plt.savefig('zy_path.pdf')


if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "rovio_v_lidar":
            print("Plotting rovio v lidar")
            plot_rovio_v_loam()
            exit(0)
    print("Tell me what to do")
    exit(1)

