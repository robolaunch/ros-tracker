from threading import Thread, Lock
from std_msgs.msg import String
from ros1_info import ROS1
from ros2_info import ROS2
from system_info import *
import time

UPDATE_FREQUENCY = 0.1
ROS_VERSION = 2

general_lock = Lock()
topics = None
services = None
nodes = None
hostname = None
port = None
memory_usage = None
cpu_usage = None
network_usage = None
process_list = None


if ROS_VERSION == 1:
    import rospy
    class ServiceThread():

        def loop():
            print("update thread started")
            global topics, services, nodes, hostname, port, memory_usage, cpu_usage, network_usage, process_list
            while True:
                temp_topics = ROS1.getTopics()
                general_lock.acquire()
                topics = temp_topics
                general_lock.release()

                temp_services = ROS1.getServices()
                general_lock.acquire()
                services = temp_services
                general_lock.release()

                temp_nodes = ROS1.getNodes()
                general_lock.acquire()
                nodes = temp_nodes
                general_lock.release()

                temp_hostname, temp_port = ROS1.getHostnamePort()
                general_lock.acquire()
                hostname = temp_hostname
                port = temp_port
                general_lock.release()

                temp_memory_usage = get_memory_usage()
                general_lock.acquire()
                memory_usage = temp_memory_usage
                general_lock.release()

                temp_cpu_usage = get_total_cpu_usage()
                general_lock.acquire()
                cpu_usage = temp_cpu_usage
                general_lock.release()

                temp_network_usage = get_network_usage_dict()
                general_lock.acquire()
                network_usage = temp_network_usage
                general_lock.release()

                temp_process_list = get_process_cpu_usage()
                general_lock.acquire()
                process_list = temp_process_list
                general_lock.release()

                time.sleep(UPDATE_FREQUENCY)

        def rosNode():
            global topics, services, nodes, hostname, port, memory_usage, cpu_usage, network_usage, process_list
            
            pub = rospy.Publisher('/statistics', String, queue_size=10)
            # rate according to the update frequency
            rate = rospy.Rate(UPDATE_FREQUENCY)
            i = 0
            while not rospy.is_shutdown():
                i += 1
                #print("Publishing to topic /statistics")
                general_lock.acquire()
                all_str = str({'topics': topics, 'services': services, 'nodes': nodes, 'hostname': hostname, 'port': port, 'memory_usage': memory_usage, 'cpu_usage': cpu_usage, 'network_usage': network_usage, 'process_list': process_list})
                general_lock.release()

                #print("published")
                pub.publish(all_str)

                #print("sleeping")
                #rospy.sleep(UPDATE_FREQUENCY)
                rate.sleep()
                #print("slept" + str(i))

    rospy.init_node('KAE_Metrics', anonymous=True)
    __thread = Thread(target = ServiceThread.loop)
    __thread2 = Thread(target = ServiceThread.rosNode)
    __thread.start()
    __thread2.start()

else:
    class ServiceThread():

        def loop():
            print("update thread started")
            global topics, services, nodes, hostname, port, memory_usage, cpu_usage, network_usage, process_list
            while True:
                temp_topics = ROS2.getTopics()
                general_lock.acquire()
                topics = temp_topics
                general_lock.release()
                
                temp_nodes = ROS2.getNodes()
                general_lock.acquire()
                nodes = temp_nodes
                general_lock.release()

                """
                temp_services = ROS2.getServices()
                general_lock.acquire()
                services = temp_services
                general_lock.release()


                temp_hostname, temp_port = ROS2.getHostnamePort()
                general_lock.acquire()
                hostname = temp_hostname
                port = temp_port
                general_lock.release()
                """

                temp_memory_usage = get_memory_usage()
                general_lock.acquire()
                memory_usage = temp_memory_usage
                general_lock.release()

                temp_cpu_usage = get_total_cpu_usage()
                general_lock.acquire()
                cpu_usage = temp_cpu_usage
                general_lock.release()

                temp_network_usage = get_network_usage_dict()
                general_lock.acquire()
                network_usage = temp_network_usage
                general_lock.release()

                temp_process_list = get_process_cpu_usage()
                general_lock.acquire()
                process_list = temp_process_list
                general_lock.release()

                time.sleep(UPDATE_FREQUENCY)

    __thread = Thread(target = ServiceThread.loop)
    __thread.start()


