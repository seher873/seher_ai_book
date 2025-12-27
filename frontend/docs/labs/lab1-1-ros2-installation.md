---
sidebar_position: 15
---

# Lab 1.1: ROS2 Installation and Basic Commands

## Objective

To successfully install ROS2 Humble Hawksbill and become familiar with basic ROS2 commands for system introspection and communication.

## Prerequisites

- Ubuntu 22.04 LTS installed
- Basic command line knowledge
- Internet connection for package downloads
- Administrative access (sudo)

## Required Equipment/Software

- Ubuntu 22.04 LTS computer
- Minimum 4GB RAM, 5GB available disk space
- Internet connection

## Learning Goals

After completing this lab, you will be able to:
- Install ROS2 Humble Hawksbill on Ubuntu 22.04
- Set up the ROS2 environment
- Verify the installation using basic ROS2 commands
- Understand ROS2's distributed architecture
- Use tools for introspection and debugging

## Setup Instructions

### Step 1: Prepare your system

1. Update your system packages:
   ```bash
   sudo apt update && sudo apt upgrade
   ```

2. Ensure your locale is set to UTF-8:
   ```bash
   locale  # check for UTF-8
   sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

### Step 2: Install ROS2

1. Add the ROS2 GPG key and repository:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

2. Install ROS2 Humble Desktop:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

3. Install additional tools:
   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
   ```

### Step 3: Set up the environment

1. Source the ROS2 setup script:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. To automatically source the environment in new terminals:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Step 4: Initialize rosdep
```bash
sudo rosdep init
rosdep update
```

## Procedure

### Step 1: Verify Installation (15 minutes)

1. Check the ROS2 version:
   ```bash
   ros2 --version
   ```
   You should see output similar to: `ros2 version 0.19.0`

2. List available ROS2 commands:
   ```bash
   ros2
   ```
   This will show all available ROS2 command groups.

3. Check available packages:
   ```bash
   ros2 pkg list | head -20
   ```
   This will list the installed ROS2 packages.

### Step 2: Understanding the ROS2 Graph (20 minutes)

1. Open a new terminal and source ROS2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. List current nodes:
   ```bash
   ros2 node list
   ```
   Initially, the list might be empty.

3. Run a simple demo node in the first terminal:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp talker
   ```
   You should see messages being published every 500ms.

4. In a second terminal (with ROS2 sourced), list nodes again:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 node list
   ```
   You should now see `/talker` in the list.

5. Get detailed information about the talker node:
   ```bash
   ros2 node info /talker
   ```

### Step 3: Topic Communication (20 minutes)

1. While keeping the talker node running, check the topics:
   ```bash
   ros2 topic list
   ```
   You should see `/chatter` listed.

2. Get information about the `/chatter` topic:
   ```bash
   ros2 topic info /chatter
   ```

3. Echo the messages being published:
   ```bash
   ros2 topic echo /chatter
   ```
   This will show the messages from the talker node in real-time.

4. Open another terminal and run the listener node:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp listener
   ```
   You should see the same messages that the talker is publishing.

### Step 4: Understanding Message Types (10 minutes)

1. Check the message type of the `/chatter` topic:
   ```bash
   ros2 topic type /chatter
   ```
   You should see: `std_msgs/msg/String`

2. Get detailed information about the message type:
   ```bash
   ros2 interface show std_msgs/msg/String
   ```
   This shows the structure of the String message type.

### Step 5: Services (15 minutes)

1. In a new terminal, list available services:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 service list
   ```
   You might see some services related to parameter services.

2. Let's create and experiment with a simple service:
   ```bash
   # Terminal 1: Start the add_two_ints service server
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp add_two_ints_server
   ```

   ```bash
   # Terminal 2: Call the service
   source /opt/ros/humble/setup.bash
   ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 3}"
   ```
   You should see the result of adding 1 + 3.

### Step 6: Using GUI Tools (15 minutes)

1. Install rqt tools:
   ```bash
   sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins
   ```

2. Run rqt:
   ```bash
   rqt
   ```

3. In rqt, explore the following plugins:
   - **Node Graph** (under Plugins → Services/Node → Node Graph)
     - Shows the ROS2 graph with nodes and topics
     - Run your talker and listener nodes to see them appear in the graph
   
   - **Topic Monitor** (under Plugins → Topic → Topic Monitor)
     - Shows messages on various topics in real-time

## Expected Outcomes

After completing this lab, you will have:
- Successfully installed ROS2 Humble Hawksbill
- Verified the installation by running a simple publisher-subscriber demo
- Used various ROS2 command-line tools (`ros2 node`, `ros2 topic`, `ros2 service`)
- Observed the distributed nature of ROS2 in action
- Used GUI tools (rqt) for introspection
- Understood basic ROS2 concepts like nodes, topics, and services

## Troubleshooting

### Issue: Commands not found
**Symptoms**: `command not found: ros2` or similar
**Solution**: Ensure you've sourced the ROS2 environment:
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Nodes don't see each other
**Symptoms**: Nodes running on the same machine don't communicate
**Solution**: Check that ROS_DOMAIN_ID is the same for all nodes:
```bash
echo $ROS_DOMAIN_ID  # should return the same value for all terminals
```

### Issue: Permission error when creating workspaces
**Symptoms**: Cannot create directories or files
**Solution**: Check your permissions and ensure you're running in your home directory

## Assessment Criteria

- [ ] Installation of ROS2 Humble completed successfully
- [ ] Verification steps completed with expected output
- [ ] Commands from all procedures executed correctly
- [ ] Understanding demonstrated by explaining communication flow between nodes
- [ ] Lab report submitted with screenshots of commands and output

## Extensions

For advanced students:
1. Try creating a simple publisher that publishes custom messages
2. Experiment with Quality of Service settings
3. Create a simple launch file to run multiple nodes together

## Notes for Instructors

- Ensure enough disk space is available on student machines
- Consider pre-installing ROS2 on lab computers to save time
- Have a backup plan if internet access is slow for package downloads
- Emphasize the distributed nature of ROS2 versus centralized systems