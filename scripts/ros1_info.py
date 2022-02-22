from subprocess import Popen, PIPE

# This function gets the current ros topics with the command "rostopic list"
def get_ros_topics():
    p = Popen(["rostopic", "list"], stdout=PIPE, stderr=PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        return "ERROR! " + error.decode() + " with error code " + str(p.returncode)
    
    output = output.split()
    i = 0
    while i < len(output):
        # convert from bytes to string
        output[i] = output[i].decode("utf-8")[1:]
        i += 1
    return output


# This function executes "rosnode list" command and returns the output
def get_node_list():
    p = Popen(["rosnode", "list"], stdout=PIPE, stderr=PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        return "ERROR! " + error.decode() + " with error code " + str(p.returncode)

    output = output.split()
    i = 0
    while i < len(output):
        # convert from bytes to string
        output[i] = output[i].decode("utf-8")[1:]
        i += 1
    return output

# This function gets the current ros services with the command "rosservice list"
def get_ros_services():
    p = Popen(["rosservice", "list"], stdout=PIPE, stderr=PIPE)
    output, error = p.communicate()
    if p.returncode != 0:
        return "ERROR! " + error.decode() + " with error code " + str(p.returncode)

    output = output.split()
    i = 0
    while i < len(output):
        # convert from bytes to string
        output[i] = output[i].decode("utf-8")[1:]
        i += 1
    return output
