from subprocess import Popen, PIPE
import os

# This class is just to wrap ROS1 functions into one structure
class ROS1:

    # This function gets the current ros topics with the command "rostopic list"
    def getTopics():
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
    def getNodes():
        p = Popen(["rosnode", "list"], stdout=PIPE, stderr=PIPE)
        output, error = p.communicate()
        if p.returncode != 0:
            return "ERROR! " + error.decode() + " with error code " + str(p.returncode)

        output = output.split()
        i = 0
        while i < len(output):
            # convert from bytes to string
            output_together = output[i].decode("utf-8")
            # separate namespace
            output[i] = {"namespace": output_together[:output_together.find('/')+1], "node_name": output_together[output_together.find('/')+1:]}
            i += 1
        return output

    # This function gets the current ros services with the command "rosservice list"
    def getServices():
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

    # This function gets the current ROS hostname and port
    def getHostnamePort():
        address = os.environ.get("ROS_MASTER_URI")
        # http://localhost:11311
        if address[:7] != "http://":
            print("ERROR! ROS_MASTER_URI is not set correctly")
            return
        address = address[7:]
        address = address.split(':')
        return address