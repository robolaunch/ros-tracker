from subprocess import Popen, PIPE
from time import sleep
from globals import NoROScoreError
import globals
import signal

def _startROSBag(bag_name, topic_name):
    #print("types:", type(bag_name), type(topic_name))
    p = Popen(["rosbag", "record", "-O", bag_name, topic_name], stdout=PIPE, stderr=PIPE)
    # TODO need to check if that bag name is exists,
    globals.rosbagged_dict[bag_name] = {"topic_name": topic_name, "process": p}

def _closeROSBag(bag_name):
    p = globals.rosbagged_dict[bag_name]["process"]
    p.send_signal(signal.SIGINT)
    p.wait()
    del globals.rosbagged_dict[bag_name]


def rosbagHandler():
    while True:
        if globals.rosbag_start_list:
            #------------------------------
            globals.rosbag_lock.acquire()
            for rosbag_start in globals.rosbag_start_list:
                _startROSBag(rosbag_start["bag_name"], rosbag_start["topic_name"])
            globals.rosbag_start_list = []
            globals.rosbag_lock.release()
            #------------------------------
        if globals.rosbag_close_list:
            #------------------------------
            globals.rosbag_lock.acquire()
            for rosbag_close in globals.rosbag_close_list:
                _closeROSBag(rosbag_close["bag_name"])
            globals.rosbag_close_list = []
            globals.rosbag_lock.release()
            #------------------------------
        sleep(globals.UPDATE_FREQUENCY)

