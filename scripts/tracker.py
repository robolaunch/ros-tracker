from flask import Flask, jsonify
from flask_restful import Api, Resource
from system_info import get_memory_usage, get_network_usage_dict, get_process_cpu_usage, get_total_cpu_usage
from ros1_info import ROS1
from ros2_info import ROS2
import globals
from threading import Thread
import time
from std_msgs.msg import String

# start a Flask web server
app = Flask(__name__)
api = Api(app)

class ROS2ServiceThread():
    def loop():
        while True:
            current_time = time.time()
            temp_topics = ROS2.getTopics()
            globals.general_lock.acquire()
            globals.topics = temp_topics
            globals.general_lock.release()
            print("ROS2 topics update time: " + str(time.time() - current_time))
            current_time = time.time()
            
            temp_nodes = ROS2.getNodes()
            globals.general_lock.acquire()
            globals.nodes = temp_nodes
            globals.general_lock.release()
            print("ROS2 nodes update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_services = ROS2.getServices()
            globals.general_lock.acquire()
            globals.services = temp_services
            globals.general_lock.release()
            print("ROS2 services update time: " + str(time.time() - current_time))
            current_time = time.time()


            """
            temp_hostname, temp_port = ROS2.getHostnamePort()
            globals.general_lock.acquire()
            globals.hostname = temp_hostname
            globals.port = temp_port
            globals.general_lock.release()
            """

            temp_memory_usage = get_memory_usage()
            globals.general_lock.acquire()
            globals.memory_usage = temp_memory_usage
            globals.general_lock.release()
            print("ROS2 memory usage update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_cpu_usage = get_total_cpu_usage()
            globals.general_lock.acquire()
            globals.cpu_usage = temp_cpu_usage
            globals.general_lock.release()
            print("ROS2 cpu usage update time: " + str(time.time() - current_time))

            temp_network_usage = get_network_usage_dict()
            globals.general_lock.acquire()
            globals.network_usage = temp_network_usage
            globals.general_lock.release()
            print("ROS2 network usage update time: " + str(time.time() - current_time))
            current_time = time.time()

            """
            temp_process_list = get_process_cpu_usage()
            globals.general_lock.acquire()
            globals.process_list = temp_process_list
            globals.general_lock.release()
            print("ROS2 process list update time: " + str(time.time() - current_time))
            current_time = time.time()
            """

            print("-------------------------------------------------------")
            time.sleep(globals.UPDATE_FREQUENCY)

class ROS1ServiceThread():
    def loop():
        while True:
            current_time = time.time()
            temp_topics = ROS1.getTopics()
            globals.general_lock.acquire()
            globals.topics = temp_topics
            globals.general_lock.release()
            print("ROS1 topics update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_services = ROS1.getServices()
            globals.general_lock.acquire()
            globals.services = temp_services
            globals.general_lock.release()
            print("ROS1 services update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_nodes = ROS1.getNodes()
            globals.general_lock.acquire()
            globals.nodes = temp_nodes
            globals.general_lock.release()
            print("ROS1 nodes update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_hostname, temp_port = ROS1.getHostnamePort()
            globals.general_lock.acquire()
            globals.hostname = temp_hostname
            globals.port = temp_port
            globals.general_lock.release()
            print("ROS1 hostname and port update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_memory_usage = get_memory_usage()
            globals.general_lock.acquire()
            globals.memory_usage = temp_memory_usage
            globals.general_lock.release()
            print("ROS1 memory usage update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_cpu_usage = get_total_cpu_usage()
            globals.general_lock.acquire()
            globals.cpu_usage = temp_cpu_usage
            globals.general_lock.release()
            print("ROS1 cpu usage update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_network_usage = get_network_usage_dict()
            globals.general_lock.acquire()
            globals.network_usage = temp_network_usage
            globals.general_lock.release()
            print("ROS1 network usage update time: " + str(time.time() - current_time))
            current_time = time.time()

            temp_process_list = get_process_cpu_usage()
            globals.general_lock.acquire()
            globals.process_list = temp_process_list
            globals.general_lock.release()
            print("ROS1 process list update time: " + str(time.time() - current_time))
            current_time = time.time()

            time.sleep(globals.UPDATE_FREQUENCY)

    def rosNode():
        import rospy
        rospy.init_node('KAE_Metrics', anonymous=True)
        pub = rospy.Publisher('/statistics', String, queue_size=10)
        # rate according to the update frequency
        rate = rospy.Rate(globals.UPDATE_FREQUENCY)
        i = 0
        while not rospy.is_shutdown():
            i += 1
            #print("Publishing to topic /statistics")
            globals.general_lock.acquire()
            all_str = str({'topics': globals.topics, 'services': globals.services, 'nodes': globals.nodes, 'hostname': globals.hostname, 'port': globals.port, 'memory_usage': globals.memory_usage, 'cpu_usage': globals.cpu_usage, 'network_usage': globals.network_usage, 'process_list': globals.process_list})
            globals.general_lock.release()

            #print("published")
            pub.publish(all_str)

            #print("sleeping")
            #rospy.sleep(globals.UPDATE_FREQUENCY)
            rate.sleep()
            #print("slept" + str(i))
            
def openParameterUpdateThread():
    if globals.ROS_VERSION == 1:
        __thread = Thread(target = ROS1ServiceThread.loop)
        #__thread2 = Thread(target = ROS1ServiceThread.rosNode)
        __thread.start()
        #__thread2.start()

    else:
        __thread = Thread(target = ROS2ServiceThread.loop)
        __thread.start()



class SystemInfo(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"memory_usage": globals.memory_usage, "cpu_usage": globals.cpu_usage, "network_usage": globals.network_usage}
        globals.general_lock.release()
        return jsonify(output)
class Processes(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"process_list": globals.process_list}
        globals.general_lock.release()
        return jsonify(output)
class ROS1Topic(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"topics": globals.topics}
        globals.general_lock.release()
        return jsonify(output)
class ROS1Service(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"services": globals.services}
        globals.general_lock.release()
        return jsonify(output)
class ROS1Nodes(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"nodes": globals.nodes}
        globals.general_lock.release()
        return jsonify(output)
class ROS1NetworkInfo(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"hostname": globals.hostname, "port": globals.port}
        globals.general_lock.release()
        return jsonify(output)

class ROS2Topic(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"topics": globals.topics}
        globals.general_lock.release()
        return jsonify(output)
class ROS2Service(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"services": globals.services}
        globals.general_lock.release()
        return jsonify(output)
class ROS2Nodes(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"nodes": globals.nodes}
        globals.general_lock.release()
        return jsonify(output)
class ROS2NetworkInfo(Resource):
    def get(self):
        globals.general_lock.acquire()
        output = {"hostname": globals.hostname, "port": globals.port}
        globals.general_lock.release()
        return jsonify(output)



if __name__ == "__main__":
    openParameterUpdateThread()

    # add the class to the API
    api.add_resource(SystemInfo, '/system')
    api.add_resource(Processes, '/processes')
    
    if globals.ROS_VERSION == 1:
        api.add_resource(ROS1Topic, '/ros1/topics')
        api.add_resource(ROS1Service, '/ros1/services')
        api.add_resource(ROS1Nodes, '/ros1/nodes')
        api.add_resource(ROS1NetworkInfo, '/ros1/network')
    elif globals.ROS_VERSION == 2:
        api.add_resource(ROS2Topic, '/ros2/topics')
        api.add_resource(ROS2Service, '/ros2/services')
        api.add_resource(ROS2Nodes, '/ros2/nodes')
        api.add_resource(ROS2NetworkInfo, '/ros2/network')


    app.run(debug=False, use_reloader=False)