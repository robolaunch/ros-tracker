from fastapi import FastAPI
from enum import Enum
from threading import Thread, Lock
from ros2_info import ROS2
from ros1_info import ROS1
import time
import system_info
from gpu_info import getGPUInfo
from rosbag import ROSBag
import globals
import api_returns
from typing import List


def ros2Loop():
    while True:
        #current_time = time.time()
        temp_topics = ROS2.getTopics()
        globals.general_lock.acquire()
        globals.topics = temp_topics
        globals.general_lock.release()
        #print("ROS2 topics update time: " + str(time.time() - current_time))
        #current_time = time.time()
        
        temp_nodes = ROS2.getNodes()
        globals.general_lock.acquire()
        globals.nodes = temp_nodes
        globals.general_lock.release()
        #print("ROS2 nodes update time: " + str(time.time() - current_time))
        #current_time = time.time()

        temp_services = ROS2.getServices()
        globals.general_lock.acquire()
        globals.services = temp_services
        globals.general_lock.release()
        #print("ROS2 services update time: " + str(time.time() - current_time))
        #current_time = time.time()

        temp_actions = ROS2.getActions()
        globals.general_lock.acquire()
        globals.actions = temp_actions
        globals.general_lock.release()
        #print("ROS2 actions update time: " + str(time.time() - current_time))
        #current_time = time.time()

        """
        temp_hostname, temp_port = ROS2.getHostnamePort()
        globals.general_lock.acquire()
        globals.hostname = temp_hostname
        globals.port = temp_port
        globals.general_lock.release()
        """

        #print("-------------------------------------------------------")
        print("sleeping 10 sec")
        time.sleep(globals.UPDATE_FREQUENCY)
        print("slept 10 sec")

def ros1Loop():
    while True:
        #current_time = time.time()
        temp_topics, temp_actions = ROS1.getTopicsActions()
        globals.general_lock.acquire()
        globals.topics = temp_topics
        globals.actions = temp_actions
        globals.general_lock.release()
        #print("ROS1 topics update time: " + str(time.time() - current_time))
        #current_time = time.time()

        temp_services = ROS1.getServices()
        globals.general_lock.acquire()
        globals.services = temp_services
        globals.general_lock.release()
        #print("ROS1 services update time: " + str(time.time() - current_time))
        #current_time = time.time()

        temp_nodes = ROS1.getNodes()
        globals.general_lock.acquire()
        globals.nodes = temp_nodes
        globals.general_lock.release()
        #print("ROS1 nodes update time: " + str(time.time() - current_time))
        #current_time = time.time()

        temp_hostname, temp_port = ROS1.getHostnamePort()
        globals.general_lock.acquire()
        globals.hostname = temp_hostname
        globals.port = temp_port
        globals.general_lock.release()
        #print("ROS1 hostname and port update time: " + str(time.time() - current_time))
        #current_time = time.time()

        time.sleep(globals.UPDATE_FREQUENCY)

def systemLoop():
    while True:
        temp_memory_usage = system_info.getMemoryUsage()
        globals.general_lock.acquire()
        globals.memory_usage = temp_memory_usage
        globals.general_lock.release()
        #print("ROS2 memory usage update time: " + str(time.time() - current_time))
        #current_time = time.time()

        temp_cpu_usage, temp_core_usage = system_info.getTotalCpuUsage()
        globals.general_lock.acquire()
        globals.cpu_usage = temp_cpu_usage
        globals.cpu_core_sage = temp_core_usage
        globals.general_lock.release()
        #print("ROS2 cpu usage update time: " + str(time.time() - current_time))

        temp_network_usage = system_info.getNetworkUsageDict()
        globals.general_lock.acquire()
        globals.network_usage = temp_network_usage
        globals.general_lock.release()
        #print("ROS2 network usage update time: " + str(time.time() - current_time))
        #current_time = time.time()

        temp_uptime = system_info.getUptime()
        globals.general_lock.acquire()
        globals.uptime = temp_uptime
        globals.general_lock.release()

        """
        temp_process_list = get_process_cpu_usage()
        globals.general_lock.acquire()
        globals.process_list = temp_process_list
        globals.general_lock.release()
        #print("ROS2 process list update time: " + str(time.time() - current_time))
        #current_time = time.time()
        """

        temp_gpu_info = getGPUInfo()
        globals.general_lock.acquire()
        globals.gpu_info = temp_gpu_info
        globals.general_lock.release()

        time.sleep(globals.UPDATE_FREQUENCY)     
            
def openThreads():
    if globals.ROS_VERSION == 1:
        __thread = Thread(target = ros1Loop)
        __thread.start()
        # rosnode cannot be started in another thread. It will be checked later. TODO
        #__thread2 = Thread(target = ROS1ServiceThread.rosNode)
        #__thread2.start()

    else:
        __thread = Thread(target = ros2Loop)
        __thread.start()
    
    __thread_system = Thread(target = systemLoop)
    __thread_system.start()
    __thread_bag = Thread(target = ROSBag.rosBagHandler)
    __thread_bag.start()

openThreads()


app = FastAPI()

@app.get("/")
async def root():
    return {"message": "Hello World"}




@app.get("/system", response_model = api_returns.SystemOut)
async def system():
    globals.general_lock.acquire()
    output = {"memory_usage": globals.memory_usage, "cpu_usage": globals.cpu_usage, "cpu_core_usage": globals.cpu_core_usage ,"network_usage": globals.network_usage, "uptime": globals.uptime}
    globals.general_lock.release()
    return output

@app.get("/gpu", response_model = List[api_returns.GPUOut])
async def gpu():
    globals.general_lock.acquire()
    output = globals.gpu_info
    globals.general_lock.release()
    print(output)
    return output