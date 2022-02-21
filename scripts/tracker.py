from flask import Flask, jsonify
from flask_restful import Api, Resource
from system_info import *
from ros_info import *

    

# open a RESTful server with flask-restful package that returns whole process information on the system
def open_restful_server():
    # start a Flask web server
    app = Flask(__name__)
    api = Api(app)

    # define a class for the RESTful API returns json
    class ProcessInfo(Resource):
        def get(self):
            # get the list of processes
            process_list = get_process_cpu_usage()
            # get the total memory
            memory_usage = get_memory_usage()
            # get the list of ros topics
            ros_topics = get_ros_topics()
            # get the list of ros services
            ros_services = get_ros_services()
            # get the total CPU usage
            total_cpu_usage = get_total_cpu_usage()
            # get the list of running nodes
            node_list = get_node_list()

            # return the json
            return jsonify({'process_list': process_list, 'total_memory': memory_usage, 'ros_topics': ros_topics, 'ros_services': ros_services, 'total_cpu_usage': total_cpu_usage, 'node_list': node_list})

    class SystemInfo(Resource):
        def get(self):
            # get the total memory
            total_memory = get_memory_usage()
            # get the total CPU usage
            total_cpu_usage = get_total_cpu_usage()
            # get the network usage
            network_usage = get_network_usage_dict()

            # return the json
            return jsonify({'total_memory': total_memory, 'total_cpu_usage': total_cpu_usage, 'network_usage': network_usage})

    class ROS1Info(Resource):
        def get(self):
            # get the list of ros topics
            ros_topics = get_ros_topics()
            # get the list of ros services
            ros_services = get_ros_services()
            # get the list of running nodes
            node_list = get_node_list()

            # return the json
            return jsonify({'ros_topics': ros_topics, 'ros_services': ros_services, 'node_list': node_list})

    # add the class to the API
    api.add_resource(ProcessInfo, '/')
    api.add_resource(SystemInfo, '/system')
    api.add_resource(ROS1Info, '/ros1')

    app.run(debug=True)

if __name__ == "__main__":
    #print(get_process_cpu_usage())
    #print(get_total_cpu_usage())
    open_restful_server()