import psutil
import subprocess
from flask import Flask, jsonify
from flask_restful import Api, Resource

# This function executes "rosnode list" command and returns the output
def get_node_list():
    output = subprocess.check_output(["rosnode", "list"])
    output = output.split()
    i = 0
    while i < len(output):
        # convert from bytes to string
        output[i] = output[i].decode("utf-8")[1:]
        i += 1
    return output


# This function returns the total memory of the system
def get_total_memory():
    total_memory = psutil.virtual_memory().total
    total_memory /= 1048576
    return total_memory


# This function gets the current ros topics with the command "rostopic list"
def get_ros_topics():
    output = subprocess.check_output(["rostopic", "list"])
    output = output.split()
    i = 0
    while i < len(output):
        # convert from bytes to string
        output[i] = output[i].decode("utf-8")[1:]
        i += 1
    return output

# This function gets the current ros services with the command "rosservice list"
def get_ros_services():
    output = subprocess.check_output(["rosservice", "list"])
    output = output.split()
    i = 0
    while i < len(output):
        # convert from bytes to string
        output[i] = output[i].decode("utf-8")[1:]
        i += 1
    return output

# Returns all processes and their CPU usages in a list
# This function takes too much time, because while taking CPU usages, we get an average usage of "0.01" seconds for each process.
# This must be executed in a separate thread.
def get_process_cpu_usage():
    process_cpu_usage = []
    for proc in psutil.process_iter():
        try:
            # Get process name & pid from process object.
            processName = proc.name()
            processID = proc.pid

            process_cpu_usage.append([processName, processID, proc.cpu_percent(interval=0.01)])
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return process_cpu_usage

# This function returns the total CPU usage of the system
def get_total_cpu_usage():
    # this returns 0 in every first call, I don't know the reason
    total_cpu_usage = psutil.cpu_percent()
    return total_cpu_usage

    

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
            total_memory = get_total_memory()
            # get the list of ros topics
            ros_topics = get_ros_topics()
            # get the list of ros services
            ros_services = get_ros_services()
            # get the total CPU usage
            total_cpu_usage = get_total_cpu_usage()
            # get the list of running nodes
            node_list = get_node_list()

            # return the json
            return jsonify({'process_list': process_list, 'total_memory': total_memory, 'ros_topics': ros_topics, 'ros_services': ros_services, 'total_cpu_usage': total_cpu_usage, 'node_list': node_list})


    # add the class to the API
    api.add_resource(ProcessInfo, '/')

    app.run(debug=True)

if __name__ == "__main__":
    #print(get_process_cpu_usage())
    #print(get_total_cpu_usage())
    open_restful_server()