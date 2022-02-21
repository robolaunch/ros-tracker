import roslibpy

client = roslibpy.Ros(host='localhost', port=11311)
client.run()

def get_node_list_roslib():
    return client.get_nodes()

def get_topic_list_roslib():
    return client.get_topics()

def get_service_list_roslib():
    return client.get_services()