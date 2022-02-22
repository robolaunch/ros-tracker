from subprocess import Popen, PIPE
import os
import line_profiler

# This class is just to wrap ROS1 functions into one structure
class ROS1:

    # This function gets the current ros topics with the command "rostopic list"
    #@profile
    def getTopics():
        p = Popen(["rostopic", "list"], stdout=PIPE, stderr=PIPE)
        output, error = p.communicate()
        if p.returncode != 0:
            return "ERROR! " + error.decode() + " with error code " + str(p.returncode)
        
        output = output.split()
        i = 0
        while i < len(output):
            topic_name = output[i].decode("utf-8")

            p2 = Popen(["rostopic", "info", topic_name], stdout=PIPE, stderr=PIPE)
            output2, error2 = p2.communicate()
            if p2.returncode != 0:
                return "ERROR! " + error2.decode() + " with error code " + str(p2.returncode)
            
            output2 = output2.split()

            # for now
            # 0 is "Type"
            # 1 is the type
            # 2 is publishers
            # 3 is *
            # 4 is first publisher
            # 5 is the URI of the publisher
            # 6 is * if there are more publishers
            # otherwise, it is "Subscribers:" 
            # the same loop again...

            type_of_topic = output2[1].decode("utf-8")

            publisher_cnt = 3
            publishers = []
            while True:
                # that means publishers are over
                if output2[publisher_cnt].decode("utf-8") != "*":
                    if output2[publisher_cnt].decode("utf-8") == "None":
                        publisher_cnt += 1
                    break
                
                publisher_name = output2[publisher_cnt+1].decode("utf-8")
                publisher_uri = output2[publisher_cnt+2].decode("utf-8")
                publishers.append({"name": publisher_name, "uri": publisher_uri})

                publisher_cnt += 3

            subscribers_cnt = publisher_cnt + 1
            subscribers = []
            while True:
                if subscribers_cnt >= len(output2):
                    break
                # that means subscribers are over
                if output2[subscribers_cnt].decode("utf-8") != "*":

                    break
                
                subscriber_name = output2[subscribers_cnt+1].decode("utf-8")
                subscriber_uri = output2[subscribers_cnt+2].decode("utf-8")
                subscribers.append({"name": subscriber_name, "uri": subscriber_uri})

                subscribers_cnt += 3


            output[i] = {"name": topic_name, "type": type_of_topic, "publishers": publishers, "subscribers": subscribers}
            i += 1
        return output

    # This function executes "rosnode list" command and returns the output
    #@profile
    def getNodes():
        p = Popen(["rosnode", "list"], stdout=PIPE, stderr=PIPE)
        output, error = p.communicate()
        if p.returncode != 0:
            return "ERROR! " + error.decode() + " with error code " + str(p.returncode)

        output = output.split()
        i = 0
        while i < len(output):
            output_together = output[i].decode("utf-8")
            # separate namespace
            output[i] = {"namespace": output_together[:output_together.find('/')+1], "node_name": output_together[output_together.find('/')+1:]}
            i += 1
        return output

    # This function gets the current ros services with the command "rosservice list"
    #@profile
    def getServices():
        p = Popen(["rosservice", "list"], stdout=PIPE, stderr=PIPE)
        output, error = p.communicate()
        if p.returncode != 0:
            return "ERROR! " + error.decode() + " with error code " + str(p.returncode)

        output = output.split()
        i = 0
        while i < len(output):
            service_name = output[i].decode("utf-8")

            p2 = Popen(["rosservice", "info", service_name], stdout=PIPE, stderr=PIPE)
            output2, error2 = p2.communicate()
            if p2.returncode != 0:
                return "ERROR! " + error2.decode() + " with error code " + str(p2.returncode)
            
            output2 = output2.split()

            # for now
            # 0 is "Node:"
            # 1 is the node name
            # 2 is "URI"
            # 3 is the URI
            # 4 is "Type:"
            # 5 is the type
            # 6 is "Args:" ###### Will not used

            node_name = output2[1].decode("utf-8")
            uri = output2[3].decode("utf-8")
            type_of_service = output2[5].decode("utf-8")
            
            output[i] = {"service_name": service_name, "node_name": node_name, "uri": uri, "type": type_of_service}
            i += 1
        return output

    # This function gets the current ROS hostname and port
    #@profile
    def getHostnamePort():
        address = os.environ.get("ROS_MASTER_URI")
        # http://localhost:11311
        if address[:7] != "http://":
            print("ERROR! ROS_MASTER_URI is not set correctly")
            return
        address = address[7:]
        address = address.split(':')
        return address

