from subprocess import Popen, PIPE
import os
from globals import NoROScoreError, CannotParseError


# I am assuming that in one topic there will not be more than one type
# BUT ROS2 ACTUALLY ALLOWS THIS!

# This class is just to wrap ROS1 functions into one structure
class ROS2:

    @staticmethod
    def parseTopic(string):
        topic_splitted = string.split()
        if len(topic_splitted) < 4:
            raise CannotParseError
        
        # for now
        """
        out 0: Type:
        out 1: rcl_interfaces/msg/ParameterEvent
        out 2: Publisher
        out 3: count:
        out 4: 1
        out 5: Node
        out 6: name:
        out 7: _ros2cli_daemon_0
        out 8: Node
        out 9: namespace:
        out 10: /
        out 11: Topic
        out 12: type:
        out 13: rcl_interfaces/msg/ParameterEvent
        out 14: Endpoint
        out 15: type:
        out 16: PUBLISHER
        out 17: GID:
        out 18: 01.0f.f4.a0.9e.41.00.00.01.00.00.00.00.00.04.03.00.00.00.00.00.00.00.00
        out 19: QoS
        out 20: profile:
        out 21: Reliability:
        out 22: RMW_QOS_POLICY_RELIABILITY_RELIABLE
        out 23: Durability:
        out 24: RMW_QOS_POLICY_DURABILITY_VOLATILE
        out 25: Lifespan:
        out 26: 2147483651294967295
        out 27: nanoseconds
        out 28: Deadline:
        out 29: 2147483651294967295
        out 30: nanoseconds
        out 31: Liveliness:
        out 32: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
        out 33: Liveliness
        out 34: lease
        out 35: duration:
        out 36: 2147483651294967295
        out 37: nanoseconds
        out 38: Subscription
        out 39: count:
        out 40: 0
        """

        type_of_topic = topic_splitted[1].decode("utf-8")
        publisher_count = topic_splitted[4].decode("utf-8")
        subscriber_count = 0

        node_start_index = 5
        is_in_publishers = True
        publisher_nodes = []
        subscriber_nodes = []
        while node_start_index+31 < len(topic_splitted):
            if topic_splitted[node_start_index] == b'Subscription':
                is_in_publishers = False
                subscriber_count = topic_splitted[node_start_index+2].decode("utf-8")
                node_start_index += 3
            node_name = topic_splitted[node_start_index+2].decode("utf-8")
            node_namespace = topic_splitted[node_start_index+5].decode("utf-8")
            topic_type = topic_splitted[node_start_index+8].decode("utf-8") # this is probably duplicat
            endpoint_type = topic_splitted[node_start_index+11].decode("utf-8") # this is probably unimportant
            gid = topic_splitted[node_start_index+13].decode("utf-8")
            # qos profile
            reliability = topic_splitted[node_start_index+17].decode("utf-8")
            durability = topic_splitted[node_start_index+19].decode("utf-8")
            lifespan = topic_splitted[node_start_index+21].decode("utf-8") # I suppose this things unit will always be in nanoseconds
            deadline = topic_splitted[node_start_index+24].decode("utf-8")
            liveliness = topic_splitted[node_start_index+27].decode("utf-8")
            lease_duration = topic_splitted[node_start_index+31].decode("utf-8")
            
            if is_in_publishers:
                publisher_nodes.append({"node_name": node_name, "node_namespace": node_namespace, "topic_type": topic_type, "endpoint_type": endpoint_type, "gid": gid, "reliability": reliability, "durability": durability, "lifespan": lifespan, "deadline": deadline, "liveliness": liveliness, "lease_duration": lease_duration})
            else:
                subscriber_nodes.append({"node_name": node_name, "node_namespace": node_namespace, "topic_type": topic_type, "endpoint_type": endpoint_type, "gid": gid, "reliability": reliability, "durability": durability, "lifespan": lifespan, "deadline": deadline, "liveliness": liveliness, "lease_duration": lease_duration})
            node_start_index += 33
        # I will write subscribers later TODO

        return {"type_of_topic": type_of_topic, "publisher_count": publisher_count, "publisher_nodes": publisher_nodes, "subscriber_count": subscriber_count, "subscriber_nodes": subscriber_nodes}

    # This function gets the current ros topics with the command "rostopic list"
    @staticmethod
    def getTopics():
        try:
            p = Popen(["ros2", "topic", "list", "--include-hidden-topics"], stdout=PIPE, stderr=PIPE)
        except:
            raise NoROScoreError
        output, error = p.communicate()
        if p.returncode != 0:
            raise NoROScoreError
        
        output = output.split()
        return_list = []
        for out in output:
            topic_name = out.decode("utf-8")

            p2 = Popen(["ros2", "topic", "info", "-v", topic_name], stdout=PIPE, stderr=PIPE)
            output2, error2 = p2.communicate()
            if p2.returncode != 0:
                raise NoROScoreError

            topic_info = ROS2.parseTopic(output2)
            topic_info["topic_name"] = topic_name
            return_list.append(topic_info)
            
        return return_list

    @staticmethod
    def parseNode(string):
        splitted_nodes = string.split()

        """
        0 is b'/minimal_publisher'
        1 is b'Subscribers:'
        2 is b'Publishers:'
        3 is b'/parameter_events:'
        4 is b'rcl_interfaces/msg/ParameterEvent'
        5 is b'/rosout:'
        6 is b'rcl_interfaces/msg/Log'
        8 is b'Service'
        9 is b'Servers:'
        10 is b'/minimal_publisher/describe_parameters:'
        11 is b'rcl_interfaces/srv/DescribeParameters'
        12 is b'/minimal_publisher/get_parameter_types:'
        13 is b'rcl_interfaces/srv/GetParameterTypes'
        14 is b'/minimal_publisher/get_parameters:'
        15 is b'rcl_interfaces/srv/GetParameters'
        16 is b'/minimal_publisher/list_parameters:'
        17 is b'rcl_interfaces/srv/ListParameters'
        18 is b'/minimal_publisher/set_parameters:'
        19 is b'rcl_interfaces/srv/SetParameters'
        20 is b'/minimal_publisher/set_parameters_atomically:'
        21 is b'rcl_interfaces/srv/SetParametersAtomically'
        22 is b'Service'
        23 is b'Clients:'
        24 is b'Action'
        25 is b'Servers:'
        26 is b'Action'
        27 is b'Clients:'
        """
        # first get subscribers
        index = 2
        subscribers = []
        while True:
            if splitted_nodes[index].decode("utf-8") == "Publishers:":
                break
            subscribers.append({"topic_name": splitted_nodes[index].decode("utf-8")[:-1], "topic_type": splitted_nodes[index+1].decode("utf-8")})
            
            index += 2

        index += 1

        # now get publishers
        publishers = []
        while True:
            if splitted_nodes[index].decode("utf-8") == "Service":
                break
            publishers.append({"topic_name": splitted_nodes[index].decode("utf-8")[:-1], "topic_type": splitted_nodes[index+1].decode("utf-8")})
            
            index += 2
        
        index += 1

        # now get services
        services = []
        index += 1# because it is "service servers"
        while True:
            if splitted_nodes[index].decode("utf-8") == "Service":
                break
            services.append({"service_name": splitted_nodes[index].decode("utf-8")[:-1], "service_type": splitted_nodes[index+1].decode("utf-8")})
            
            index += 2

        index += 1

        # now get action clients
        action_clients = []
        index +=1
        while True:
            if splitted_nodes[index].decode("utf-8") == "Action":
                break
            action_clients.append({"action_name": splitted_nodes[index].decode("utf-8")[:-1], "action_type": splitted_nodes[index+1].decode("utf-8")})
            
            index += 2

        index += 2

        # not get action servers
        action_servers = []
        index += 1
        while (index + 1) < len(splitted_nodes):
            action_servers.append({"action_name": splitted_nodes[index].decode("utf-8")[:-1], "action_type": splitted_nodes[index+1].decode("utf-8")})
            
            index += 2

        # separate namespace
        return {"subscribers": subscribers, "publishers": publishers, "services": services, "action_clients": action_clients, "action_servers": action_servers}
        
    # This function executes "rosnode list" command and returns the output
    @staticmethod
    def getNodes():
        in_except = False
        output = error = None
        p = None
        try:
            p = Popen(["ros2", "node", "list", "-a"], stdout=PIPE, stderr=PIPE)
        except:
            in_except = True
            # Sometimes Popen does not assign anything to p in case of an error. 
            # This check is to prevent p.communicate() from throwing an exception.
            if p is None:
                raise NoROScoreError
            output, error = p.communicate()
            if error != 2:
                # error code 2 is:
                # There are 2 nodes in the graph with the exact name "/_ros2cli_daemon_0". You are seeing information about only one of them.
                # This is not crucial, and not showing that ROS2 is not running.
                # passing this error
                raise NoROScoreError

        if not in_except:
            output, error = p.communicate()
        if p.returncode != 0:
            raise NoROScoreError


        output = output.split()
        i = 0
        return_list = []
        while i < len(output):
            output_together = output[i].decode("utf-8")
    
            p2 = Popen(["ros2", "node", "info", "--include-hidden", output_together], stdout=PIPE, stderr=PIPE)
            p2.wait()
            p2.wait()
            output2, error = p2.communicate()
            if p2.returncode != 0 and p2.returncode != 1:
                print("Error: " + str(p2.returncode) + " " + str(p2.returncode != 1))
                raise NoROScoreError

            temp_dict = ROS2.parseNode(output2)
            temp_dict["namespace"] = output_together[:output_together.find('/')+1]
            temp_dict["node_name"] = output_together[output_together.find('/')+1:]
            return_list.append(temp_dict)
            i += 1
            
        return return_list

    # There is way less information in this than I imagined
    # It may be checked again.
    # This function gets the current ros services with the command "rosservice list"
    @staticmethod
    def getServices():
        try:
            p = Popen(["ros2", "service","--include-hidden-services" , "list", "-t"], stdout=PIPE, stderr=PIPE)
        except:
            raise NoROScoreError
        output, error = p.communicate()
        if p.returncode != 0:
            raise NoROScoreError

        # ros2 service do not have info command, it only gives name and type of the service
        output = output.split()
        return_list = []
        i = 0
        while i < len(output):
            service_name = output[i].decode("utf-8")
            service_type = output[i+1].decode("utf-8")
            return_list.append({"service_name": service_name, "service_type": service_type})
            i += 2
        return return_list

    @staticmethod
    def parseActions(string):
        """
        Action: /fibonacci
        Action clients: 0
        Action servers: 1
        /minimal_action_server [example_interfaces/action/Fibonacci]
        """
        splitted = string.decode("utf-8").split()

        # I am assuming that there will be only one action server
        return {"action_name": splitted[1], "action_client_count": splitted[4], "action_server_count": splitted[7], "action_node_name": splitted[8], "action_node_type": splitted[9][1:-1]}


    @staticmethod
    def getActions():
        try:
            p = Popen(["ros2", "action" , "list"], stdout=PIPE, stderr=PIPE)
        except:
            raise NoROScoreError
        output, error = p.communicate()
        if p.returncode != 0:
            raise NoROScoreError

        # ros2 service do not have info command, it only gives name and type of the service
        output = output.split()
        return_list = []
        for action_name in output:
            p2 = Popen(["ros2", "action", "info", action_name.decode("utf-8"), "-t"], stdout=PIPE, stderr=PIPE)
            action_info, error = p2.communicate()
            return_list.append(ROS2.parseActions(action_info))
            
        return return_list


    # This function gets the current ROS hostname and port
    @staticmethod
    def getHostnamePort():
        address = os.environ.get("ROS_MASTER_URI")
        # http://localhost:11311
        if address[:7] != "http://":
            print("ERROR! ROS_MASTER_URI is not set correctly")
            return
        address = address[7:]
        address = address.split(':')
        return address

