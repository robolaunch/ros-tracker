from threading import Lock
import os

ROS_VERSION = None
UPDATE_FREQUENCY = 30

class CannotParseError(Exception):
    pass
class NoROScoreError(Exception):
    pass


__ros_distro = os.environ['ROS_DISTRO']
if __ros_distro == 'kinetic' or __ros_distro == 'melodic' or __ros_distro == 'noetic':
    ROS_VERSION = 1
elif __ros_distro == 'foxy' or __ros_distro == 'dashing' or __ros_distro == 'galactic':
    ROS_VERSION = 2



general_lock = Lock()
topics = None
services = None
actions = None
nodes = None
hostname = None
port = None
memory_usage = None
cpu_usage = None
cpu_core_usage = None
network_usage = None
process_list = None
uptime = None
gpu_info = None

rosbag_lock = Lock()
rosbag_start_list = []
rosbag_close_list = []
rosbagged_dict = {}

