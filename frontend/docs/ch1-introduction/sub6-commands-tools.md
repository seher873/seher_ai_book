---
sidebar_position: 7
---

# 1.6. Basic ROS2 Commands and Tools

## Introduction to ROS2 Command-Line Tools

ROS2 provides a comprehensive set of command-line tools that make it easier to develop, debug, and operate ROS2-based robot systems. These tools enable you to interact with nodes, topics, services, and actions without writing additional code.

## Essential ROS2 Commands

### ros2 node

The `ros2 node` command allows you to interact with running nodes in the ROS2 system.

**Listing nodes:**
```bash
ros2 node list
```
This shows all currently running nodes on the ROS2 network.

**Getting node information:**
```bash
ros2 node info <node_name>
```
This displays detailed information about a specific node, including its publishers, subscribers, services, and actions.

### ros2 topic

The `ros2 topic` command provides tools for interacting with topics in the ROS2 system.

**Listing topics:**
```bash
ros2 topic list
```
Shows all active topics in the current ROS2 domain.

**Getting topic information:**
```bash
ros2 topic info <topic_name>
```
Displays information about a specific topic, including publisher and subscriber counts and message type.

**Echoing topic data:**
```bash
ros2 topic echo <topic_name>
```
Prints messages published to a topic to the terminal. This is useful for monitoring data streams.

**Publishing to a topic:**
```bash
ros2 topic pub <topic_name> <msg_type> <args>
```
Publishes a message to a topic. For example:
```bash
ros2 topic pub /chatter std_msgs/String "data: Hello World"
```

### ros2 service

The `ros2 service` command provides tools for interacting with ROS2 services.

**Listing services:**
```bash
ros2 service list
```
Shows all available services in the current ROS2 domain.

**Calling a service:**
```bash
ros2 service call <service_name> <service_type> <request_args>
```
Calls a service with specified arguments. For example:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### ros2 action

The `ros2 action` command provides tools for interacting with ROS2 actions.

**Listing actions:**
```bash
ros2 action list
```
Shows all available actions in the current ROS2 domain.

**Sending action goals:**
```bash
ros2 action send_goal <action_name> <action_type> <goal_args>
```
Sends a goal to an action server.

### ros2 param

The `ros2 param` command allows you to interact with node parameters.

**Listing parameters:**
```bash
ros2 param list <node_name>
```

**Getting parameter values:**
```bash
ros2 param get <node_name> <param_name>
```

**Setting parameter values:**
```bash
ros2 param set <node_name> <param_name> <value>
```

## Debugging Tools

### rqt Tools

rqt is a GUI framework for ROS2 that provides various visualization and debugging tools.

**Running rqt:**
```bash
rqt
```

Common rqt plugins include:
- **rqt_graph**: Visualizes the ROS2 graph showing nodes and topics
- **rqt_plot**: Plots numerical data from topics
- **rqt_console**: Displays ROS2 log messages
- **rqt_bag**: Plays and records ROS2 bag files

### ros2 run and ros2 launch

**Running executables:**
```bash
ros2 run <package_name> <executable_name>
```
Executes a node directly from its package.

**Launching multiple nodes:**
```bash
ros2 launch <package_name> <launch_file>.py
```
Launches multiple nodes according to a launch file.

## Bag Files for Data Recording

ROS2 bag is the tool for recording and playing back ROS2 message data.

**Recording data:**
```bash
ros2 bag record <topic_names>
```
Records messages from specified topics to a bag file.

**Playing back data:**
```bash
ros2 bag play <bag_file_path>
```
Plays back messages from a bag file.

**Info about bag file:**
```bash
ros2 bag info <bag_file_path>
```
Displays information about the contents of a bag file.

## Network and Discovery Tools

### Domain ID Management

ROS2 uses domain IDs to separate different ROS2 networks:

```bash
# Set domain ID
export ROS_DOMAIN_ID=10

# Run nodes with specific domain
ROS_DOMAIN_ID=10 ros2 run <package> <node>
```

### Network debugging

**Checking network connectivity:**
```bash
# Check if nodes can see each other
ros2 topic list
ros2 node list

# Verify DDS communication
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # or other RMW
```

## Performance Monitoring

### Resource usage

**Monitoring node resource usage:**
```bash
# View CPU and memory usage of nodes
colcon build --symlink-install
source install/setup.bash
# Run nodes and monitor with system tools like htop
```

### Topic monitoring

**Checking topic rates:**
```bash
# Monitor message rate on a topic
ros2 topic hz <topic_name>
```

**Checking topic delay:**
```bash
# Monitor delay on a topic
ros2 topic delay <topic_name>
```

## Development Workflow Commands

### Workspace management

**Building packages:**
```bash
colcon build --packages-select <pkg_name>  # Build specific package
colcon build --packages-up-to <pkg_name>   # Build package and dependencies
colcon build --symlink-install             # Use symlinks for easier development
```

### Package management

**Finding packages:**
```bash
# List all packages
ros2 pkg list

# Get package information
ros2 pkg info <pkg_name>

# Find package location
ros2 pkg prefix <pkg_name>
```

## Security and Permissions

### Setting up user permissions

For hardware access:
```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Log out and back in for changes to take effect
```

## Common Troubleshooting Commands

### General debugging

**Check ROS2 installation:**
```bash
# Verify ROS2 environment
printenv | grep ROS

# Check ROS2 version
ros2 --version
```

**Network issues:**
```bash
# Reset ROS2 daemon (sometimes needed for discovery issues)
ros2 daemon stop
ros2 daemon start
```

**Process management:**
```bash
# Kill all ROS2 processes
pkill -f ros
```

## Learning Objectives for This Section

After completing this section, you will be able to:
- Use essential ROS2 command-line tools for node, topic, and service interaction
- Debug ROS2 systems using command-line and GUI tools
- Record and replay data using bag files
- Understand and manage ROS2 network configuration
- Apply performance monitoring techniques
- Troubleshoot common ROS2 issues

## Summary

ROS2 provides a rich set of command-line and graphical tools that are essential for developing, debugging, and operating robot systems. Mastering these tools will significantly improve your productivity and ability to understand and fix issues in your robotic applications. The commands introduced in this section form the foundation of daily ROS2 development work, from running nodes to monitoring system performance.