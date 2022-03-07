from subprocess import Popen, PIPE
from time import sleep
from globals import NoROScoreError
import globals
import signal

class ROSBag():
    @staticmethod
    def _startROSBag(bag_name, topic_name):
        #print("types:", type(bag_name), type(topic_name))
        if globals.ROS_VERSION == 1:
            p = Popen(["rosbag", "record", "-O", bag_name, topic_name], stdout=PIPE, stderr=PIPE)
        elif globals.ROS_VERSION == 2:
            print("started ros2 bag")
            p = Popen(["ros2", "bag", "record", "-o", bag_name, topic_name], stdout=PIPE, stderr=PIPE)
        # TODO need to check if that bag name is exists,
        globals.rosbagged_dict[bag_name] = {"topic_name": topic_name, "process": p}

    @staticmethod
    def _closeROSBag(bag_name):
        p = globals.rosbagged_dict[bag_name]["process"]
        print("closing rosbag:", bag_name)
        p.send_signal(signal.SIGINT)
        p.wait()
        print("killed rosbag:", bag_name)
        del globals.rosbagged_dict[bag_name]

    @staticmethod
    def rosBagHandler():
        while True:
            if globals.rosbag_start_list:
                print("there is sth on rosbag start list")
                #------------------------------
                globals.rosbag_lock.acquire()
                for rosbag_start in globals.rosbag_start_list:
                    print("started ros bag")
                    ROSBag._startROSBag(rosbag_start["bag_name"], rosbag_start["topic_name"])
                globals.rosbag_start_list = []
                globals.rosbag_lock.release()
                #------------------------------
            if globals.rosbag_close_list:
                #------------------------------
                globals.rosbag_lock.acquire()
                for rosbag_close in globals.rosbag_close_list:
                    ROSBag._closeROSBag(rosbag_close["bag_name"])
                globals.rosbag_close_list = []
                globals.rosbag_lock.release()
                #------------------------------
            sleep(globals.UPDATE_FREQUENCY)

