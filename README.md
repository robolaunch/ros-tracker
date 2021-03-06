# ROS-tracker
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This project collects system, network and ROS statistics and information and gives all these information with a RESTful API.

To use, just run `tracker.py` in the scripts folder.

### Interface

Currently the following addresses are in use:
``` sh
/system
/processes
/rosX/topics
/rosX/services
/rosX/actions
/rosX/nodes
/rosX/network
/rosX/bag
```
X is either 1 or 2.

`system` gives system usage information.

`processes` gives a list of running processes, their PIDs, and CPU usages.

`topics` gives a list of ROS topics.

`services` gives a list of ROS services.

`actions` gives a list of ROS actions.

`nodes` gives a list of ROS nodes.

`network` gives a list of network interfaces and their IP addresses.

ROS1 and ROS2 outputs are completely different. For now, it only gives the CLI interface outputs.

For fast check, you can execute `curl http://127.0.0.1:5000/rosX/topics` and inspect the output to see if the server is running and functioning.

For testing, please run
```sh
python3 -m pytest
```

### To-Do
- [ ] System Info
- - [X] Total CPU usage
- - [X] Each core usage
- - [X] Memory usage
- - [X] Network usage
- - [ ] Some network statistics may be needed here?
- - [X] Uptime
- [ ] Process Info
- - [X] Process name
- - [X] Process PID
- - [ ] Process CPU usage(this is actually given, but current data can be incostistent. Needs further consideration.)
- [X] ROS2 Topics (All data that CLI gives are included here.)
- - [X] Publisher count
- - [X] Publisher nodes along with all QoS information that `ros2 topic info -v` gives(this part needs a check and feedback from network people)
- - [X] Subscriber count
- - [X] Subscriber nodes along with QoS information
- [X] ROS2 Services
- [X] ROS2 Nodes
- [X] ROS2 Actions
- [X] ROS1 Topics
- - [X] Publisher nodes and URIs(I think these URIs are special for topics and services, but need to check)
- - [X] Subscriber nodes and URIs
- [X] ROS1 Services
- - [X] Server Node
- - [X] Service Type
- - [X] Service URI
- [X] ROS1 Network
- - [X] Roscore hostname (needs to be checked)
- - [X] Roscore port
- [ ] Others
- - [ ] Latency
- - [ ] Publisher/Subscriber throughput
- - [ ] Heartbeat count
- - [X] GPU usage
- - [ ] Network info other than ROS2 gives for QoS

### Roadmap:

- [X] Tests for the currently implemented parts
- [X] ROS2 lacking parts
- [X] Storing custom topics with rosbag
- [ ] Other lacking parts
