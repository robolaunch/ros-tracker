from flask import Flask, jsonify
from flask_restful import Api, Resource
from system_info import *
from ros1_info import *

    

def open_restful_server():
    # start a Flask web server
    app = Flask(__name__)
    api = Api(app)

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


    class Processes(Resource):
        def get(self):
            # get the list of processes
            process_list = get_process_cpu_usage()

            # return the json
            return jsonify({'process_list': process_list})

    class ROS1Info(Resource):
        def get(self):
            # get the list of ros topics
            ros_topics = get_ros_topics()
            # get the list of ros services
            ros_services = get_ros_services()
            # get the list of running nodes
            node_list = get_node_list()

            # return the json
            return jsonify({ 'nodes': node_list, 'topics': ros_topics, 'services': ros_services})


    # add the class to the API
    api.add_resource(SystemInfo, '/system')
    api.add_resource(ROS1Info, '/ros1_bash')
    api.add_resource(Processes, '/processes')
    app.run(debug=True)

if __name__ == "__main__":
    open_restful_server()