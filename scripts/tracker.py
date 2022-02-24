from flask import Flask, jsonify
from flask_restful import Api, Resource
from system_info import *
from ros1_info import *
from ros2_info import *
import globals


def open_restful_server():
    # start a Flask web server
    app = Flask(__name__)
    api = Api(app)

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


    app.run(debug=True)

if __name__ == "__main__":
    open_restful_server()