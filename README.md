# ros-tracker

Requirements:
    flask
    flask-restful
    line-profiler if you want to profile some functions

To use, just run `tracker.py` in the scripts folder.

Currently the following addresses are in use:
``` sh
/system
/processes
/rosX/topics
/rosX/services
/rosX/nodes
/rosX/network
```
X is either 1 or 2.

`system` gives system usage information.
`processes` gives a list of running processes, their PIDs, and CPU usages.
`topics` gives a list of ROS topics.
`services` gives a list of ROS services.
`nodes` gives a list of ROS nodes.
`network` gives a list of network interfaces and their IP addresses.

ROS1 and ROS2 outputs are completely different. For now, it only gives the CLI interface outputs.

For fast check, you can execute `curl http://127.0.0.1:5000/ros2/topics` and inspect the output to see if the server is running and functioning.

