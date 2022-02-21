import subprocess


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
