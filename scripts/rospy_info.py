import rosnode
import rospy
import rosservice

def get_node_list_rosnode():
    return rosnode.get_node_names()

def get_topic_list_rosnode():
    topics = rospy.get_published_topics()
    ans = []
    for topic in topics:
        ans.append(topic[0])
    return ans

def get_service_list_rospy():
    return rosservice.get_service_list()