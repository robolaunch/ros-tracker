import rosnode
import rospy
import rosservice

def get_node_list_rosnode():
    return rosnode.get_node_names()

# The problem about this function is rospy.get_published_topics gets the topics that is subscribed by a node
# This is probably a optimization, since it is not necessary to hold information about the topics that is not being used by any node
# However not having an option to get all topics is a problem
# If we want to get all topics, we need to use rostopic list, or in C++ there is a function getTopics that does the same thing
# https://stackoverflow.com/a/26788880/13399661
# It may be used in the future
def get_topic_list_rosnode():
    topics = rospy.get_published_topics()
    ans = []
    for topic in topics:
        ans.append(topic[0])
    return ans

def get_service_list_rospy():
    return rosservice.get_service_list()