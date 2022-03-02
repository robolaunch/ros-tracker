from subprocess import Popen, PIPE
import os
from globals import NoROScoreError, CannotParseError

# This class is just to wrap ROS1 functions into one structure
class ROS1:
    @staticmethod
    def parseTopic(string):
        if isinstance(string, (bytes, bytearray)):
            string = string.decode("utf-8") # just if the data comes from a byte array

        splitted = string.split()

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
        try:
            type_of_topic = splitted[1]
        except IndexError:
            raise CannotParseError

        if splitted[0] != "Type:" or splitted[2] != "Publishers:":
            raise CannotParseError

        publisher_cnt = 3
        publishers = []
        while True:
            if publisher_cnt >= len(splitted):
                raise CannotParseError
            # that means publishers are over
            if splitted[publisher_cnt] != "*":
                if splitted[publisher_cnt] == "None":
                    publisher_cnt += 1
                break
            
            publisher_name = splitted[publisher_cnt+1]
            publisher_uri = splitted[publisher_cnt+2]
            publishers.append({"name": publisher_name, "uri": publisher_uri})

            publisher_cnt += 3

        if splitted[publisher_cnt] != "Subscribers:":
            raise CannotParseError
            
        subscribers_cnt = publisher_cnt + 1
        subscribers = []        
        while True:
            if subscribers_cnt >= len(splitted):
                break
            # that means subscribers are over
            if splitted[subscribers_cnt] != "*":
                break
            
            subscriber_name = splitted[subscribers_cnt+1]
            subscriber_uri = splitted[subscribers_cnt+2]
            subscribers.append({"name": subscriber_name, "uri": subscriber_uri})

            subscribers_cnt += 3

        return {"type": type_of_topic, "publishers": publishers, "subscribers": subscribers}
    
    # This function gets the current ros topics with the command "rostopic list"
    @staticmethod
    def getTopics():
        p = Popen(["rostopic", "list"], stdout=PIPE, stderr=PIPE)
        stdout_list, stderr_list = p.communicate()
        if p.returncode != 0:
            raise NoROScoreError

        topic_list_splitted = stdout_list.split()
        parsed_topics = []
        i = 0
        while i < len(topic_list_splitted):
            topic_name = topic_list_splitted[i].decode("utf-8")

            p2 = Popen(["rostopic", "info", topic_name], stdout=PIPE, stderr=PIPE)
            stdout_info, stderr_info = p2.communicate()
            if p2.returncode != 0:
                raise NoROScoreError

            ret = ROS1.parseTopic(stdout_info)
            ret["topic_name"] = topic_name

            parsed_topics.append(ret)
            i += 1
        return parsed_topics

    @staticmethod
    def parseNode(string):
        if isinstance(string, (bytes, bytearray)):
            string = string.decode("utf-8") # just if the data comes from a byte array

        splitted = string.split()
        parsed_node_info = []
        if len(splitted) == 0:
            raise CannotParseError
        i = 0
        while i < len(splitted):
            output_together = splitted[i]
            if output_together[0] != '/':
                raise CannotParseError
            # separate namespace
            #output_together = output_together[1:] # separating the first '/'
            try:
                parsed_node_info.append({"namespace": output_together[:output_together.rfind('/')], "node_name": output_together[output_together.rfind('/')+1:]})
                if parsed_node_info[-1]["namespace"] == "":
                    parsed_node_info[-1]["namespace"] = '/'
            except:
                raise CannotParseError
            i += 1
        return parsed_node_info
    
    # This function executes "rosnode list" command and returns the output
    @staticmethod
    def getNodes():
        p = Popen(["rosnode", "list"], stdout=PIPE, stderr=PIPE)
        stdout, stderr = p.communicate()
        if p.returncode != 0:
            raise NoROScoreError

        return ROS1.parseNode(stdout)

    @staticmethod
    def parseService(string):
        if isinstance(string, (bytes, bytearray)):
            string = string.decode("utf-8") # just if the data comes from a byte array
        splitted = string.split()

        # for now
        # 0 is "Node:"
        # 1 is the node name
        # 2 is "URI"
        # 3 is the URI
        # 4 is "Type:"
        # 5 is the type
        # 6 is "Args:" ###### Will not used

        if len(splitted) < 6:
            raise CannotParseError
        
        if splitted[0] != "Node:" or splitted[2] != "URI:" or splitted[4] != "Type:" or splitted[6] != "Args:":
            raise CannotParseError

        node_name = splitted[1]
        uri = splitted[3]
        type_of_service = splitted[5]
        
        return {"node_name": node_name, "uri": uri, "type": type_of_service}

    @staticmethod
    # This function gets the current ros services with the command "rosservice list"
    def getServices():
        p = Popen(["rosservice", "list"], stdout=PIPE, stderr=PIPE)
        stdout_list, stderr_list = p.communicate()
        if p.returncode != 0:
            raise NoROScoreError

        service_list_splitted = stdout_list.split()
        i = 0
        while i < len(service_list_splitted):
            service_name = service_list_splitted[i].decode("utf-8")

            p2 = Popen(["rosservice", "info", service_name], stdout=PIPE, stderr=PIPE)
            stdout_info, stderr_info = p2.communicate()
            if p2.returncode != 0:
                raise NoROScoreError

            service_list_splitted[i] = ROS1.parseService(stdout_info)
            service_list_splitted[i]["service_name"] = service_name
            i += 1
        return service_list_splitted

    # This function gets the current ROS hostname and port
    @staticmethod
    def getHostnamePort():
        try:
            address = os.environ.get("ROS_MASTER_URI")
        except:
            raise NoROScoreError
        # http://localhost:11311
        if address[:7] != "http://":
            print("ERROR! ROS_MASTER_URI is not set correctly")
            return
        address = address[7:]
        address = address.split(':')
        return address
