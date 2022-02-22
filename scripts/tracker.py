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

    class ROS1Topic(Resource):
        def get(self):
            # get the list of ros topics
            ros_topics = ROS1.getTopics()
            # return the json
            return jsonify({'topics': ros_topics})
    
    class ROS1Service(Resource):
        def get(self):
            # get the list of ros services
            ros_services = ROS1.getServices()
            # return the json
            return jsonify({'services': ros_services})

    class ROS1Nodes(Resource):
        def get(self):
            # get the list of running nodes
            node_list = ROS1.getNodes()
            # return the json
            return jsonify({'nodes': node_list})

    class ROS1NetworkInfo(Resource):
        def get(self):
            # get the hostname and port
            hostname, port = ROS1.getHostnamePort()

            # return the json
            return jsonify({'hostname': hostname, 'port': port})

    # add the class to the API
    api.add_resource(SystemInfo, '/system')
    api.add_resource(ROS1Topic, '/ros1/topics')
    api.add_resource(ROS1Service, '/ros1/services')
    api.add_resource(ROS1Nodes, '/ros1/nodes')
    api.add_resource(ROS1NetworkInfo, '/ros1/network')
    api.add_resource(Processes, '/processes')

    app.run(debug=True)

if __name__ == "__main__":
    open_restful_server()