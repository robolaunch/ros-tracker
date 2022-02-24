from subprocess import Popen, PIPE
import os
import line_profiler

# I am assuming that in one topic there will not be more than one type
# BUT ROS2 ACTUALLY ALLOWS THIS!

# This class is just to wrap ROS1 functions into one structure
class ROS2:

    # This function gets the current ros topics with the command "rostopic list"
    #@profile
    def getTopics():
        p = Popen(["ros2", "topic", "list", "--include-hidden-topics"], stdout=PIPE, stderr=PIPE)
        output, error = p.communicate()
        if p.returncode != 0:
            return "ERROR! " + error.decode() + " with error code " + str(p.returncode)
        
        output = output.split()
        i = 0
        while i < len(output):
            topic_name = output[i].decode("utf-8")

            p2 = Popen(["ros2", "topic", "info", "-v", topic_name], stdout=PIPE, stderr=PIPE)
            output2, error2 = p2.communicate()
            if p2.returncode != 0:
                return "ERROR! " + error2.decode() + " with error code " + str(p2.returncode)
            
            output2 = output2.split()
                      
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

            type_of_topic = output2[1].decode("utf-8")
            publisher_count = output2[4].decode("utf-8")

            node_start_index = 5
            publisher_nodes = []
            while True:
                node_name = output2[node_start_index+2].decode("utf-8")
                node_namespace = output2[node_start_index+5].decode("utf-8")
                topic_type = output2[node_start_index+8].decode("utf-8") # this is probably duplicat
                endpoint_type = output2[node_start_index+11].decode("utf-8") # this is probably unimportant
                gid = output2[node_start_index+13].decode("utf-8")
                # qos profile
                reliability = output2[node_start_index+17].decode("utf-8")
                durability = output2[node_start_index+19].decode("utf-8")
                lifespan = output2[node_start_index+21].decode("utf-8") # I suppose this things unit will always be in nanoseconds
                deadline = output2[node_start_index+24].decode("utf-8")
                liveliness = output2[node_start_index+27].decode("utf-8")
                lease_duration = output2[node_start_index+31].decode("utf-8")
                
                publisher_nodes.append({"node_name": node_name, "node_namespace": node_namespace, "topic_type": topic_type, "endpoint_type": endpoint_type, "gid": gid, "reliability": reliability, "durability": durability, "lifespan": lifespan, "deadline": deadline, "liveliness": liveliness, "lease_duration": lease_duration})

                node_start_index += 33
                if output2[node_start_index].decode("utf-8") != "Node":
                    break

            # I will write subscribers later
            



            output[i] = {"topic_name": topic_name, "type_of_topic": type_of_topic, "publisher_count": publisher_count, "publisher_nodes": publisher_nodes}

            #output[i] = {"name": topic_name, "type": type_of_topic, "publishers": publishers, "subscribers": subscribers}
            i += 1
        return output

    # This function executes "rosnode list" command and returns the output
    #@profile
    def getNodes():
        print("getting ros2 nodes")
        p = Popen(["ros2", "node", "list"], stdout=PIPE, stderr=PIPE)
        output, error = p.communicate()
        if p.returncode != 0:
            return "ERROR! " + error.decode() + " with error code " + str(p.returncode)

        output = output.split()
        i = 0
        print("output is " + str(output))

        while i < len(output):
            print("i is " + str(i))
            output_together = output[i].decode("utf-8")

            print("node name is " + output_together)
            p2 = Popen(["ros2", "node", "info", output_together], stdout=PIPE, stderr=PIPE)
            p2.wait()
            p2.wait()
            output2, error = p2.communicate()
            print("executed")
            if p2.returncode != 0:
                print("ERROR! " + error.decode() + " with error code " + str(p2.returncode))
                return "ERROR! " + error.decode() + " with error code " + str(p2.returncode)
            
            output2 = output2.split()
            print("output2 is " + str(output2))
            """
            i = 0
            while i < len(output2):
                print("%d is " %i + str(output2[i]))
                i += 1
            """

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
                print("subscriber:" + str(index))
                print("output2[index] is " + str(output2[index]))
                if output2[index].decode("utf-8") == "Publishers:":
                    break
                # topic and its topic together
                subscribers.append({"topic_name": output2[index].decode("utf-8")[:-1], "topic_type": output2[index+1].decode("utf-8")})
                
                index += 2

            index += 1

            # now get publishers
            publishers = []
            while True:
                print("publisher:" + str(index))
                print("output2[index] is " + str(output2[index]))
                if output2[index].decode("utf-8") == "Service":
                    break
                # topic and its topic together
                publishers.append({"topic_name": output2[index].decode("utf-8")[:-1], "topic_type": output2[index+1].decode("utf-8")})
                
                index += 2
            
            index += 1

            # now get services
            services = []
            index += 1# because it is "service servers"
            while True:
                if output2[index].decode("utf-8") == "Service":
                    break
                # topic and its topic together
                services.append({"service_name": output2[index].decode("utf-8")[:-1], "service_type": output2[index+1].decode("utf-8")})
                
                index += 2

            index += 1

            # now get action clients
            action_clients = []
            index +=1
            while True:
                if output2[index].decode("utf-8") == "Action":
                    break
                # topic and its topic together
                action_clients.append({"action_name": output2[index].decode("utf-8")[:-1], "action_type": output2[index+1].decode("utf-8")})
                
                index += 2

            index += 2

            # not get action servers
            action_servers = []
            index += 1
            while (index + 1) < len(output2):
                # topic and its topic together
                action_servers.append({"action_name": output2[index].decode("utf-8")[:-1], "action_type": output2[index+1].decode("utf-8")})
                
                index += 2

            # separate namespace
            output[i] = {"namespace": output_together[:output_together.find('/')+1], "node_name": output_together[output_together.find('/')+1:], "subscribers": subscribers, "publishers": publishers, "services": services, "action_clients": action_clients, "action_servers": action_servers}
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

