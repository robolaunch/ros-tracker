#!/usr/bin/python3

import argparse
import importlib.util

from click import launch

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

parser = argparse.ArgumentParser(description='Robolaunch parses your launch file and separates the nodes into multiple launch files')

# Required positional argument
parser.add_argument('launch_file', type=str,
                    help='The path of the launch file')

parser.add_argument('launch_number', type=int, nargs='?', default=None,
                    help='The number of the node to be executed')

args = parser.parse_args()

launch_file = args.launch_file
launch_number = args.launch_number
#print(launch_file)
#print(launch_number)

# find the last '/' and split the string from there
# this will give us the name of the launch file
# without the path
launch_path = launch_file.rsplit('/', 1)[0]
launch_name = launch_file.rsplit('/', 1)[1]

spec = importlib.util.spec_from_file_location(launch_name, launch_file)
launch_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_module)

entities = launch_module.generate_launch_description().entities

def countNodesandProcesses(entitites):
    nodes = 0
    processes = 0
    for entity in entities:
        if type(entity) == Node:
            nodes += 1
        elif type(entity) == ExecuteProcess:
            processes += 1

    return nodes, processes

if launch_number is None:
    nodes, processes = countNodesandProcesses(entities)
    print("Number of nodes: " + str(nodes))
    print("Number of processes: " + str(processes))
    exit(0)
else:
    ######## NOW! Create a new launch file for each node in the launch file
    i = 0
    before_entities = []
    while i < launch_number:
        curr_entity = entities[i]
        if type(curr_entity) == DeclareLaunchArgument:
            before_entities.append(curr_entity)
        i+=1
    curr_node = entities[i]

    launch_service = LaunchService()
    launch_description = None
    if before_entities == []:
        launch_description = LaunchDescription()
        launch_description.add_entity(curr_node)
        print("empty")
    else:
        # for now I am assuming that the only thing may be a DeclareLaunchArgument
        # otherwise the logic must be changed! TODO TODO
        launch_description = LaunchDescription(before_entities[0])###!!!!!!!!!!
        launch_description.add_entity(curr_node)
    print("launching node: " + str(launch_description))
    launch_service.include_launch_description(launch_description)
    launch_service.run()
    print(launch_service.context)
    print("done")






#print(launch_path)
#print(launch_name)

