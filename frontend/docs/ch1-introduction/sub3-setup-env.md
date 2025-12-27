---
sidebar_position: 4
---

# 1.3. Setting up Development Environment

## Prerequisites

Before installing ROS2, ensure your system meets the following requirements:

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish) - recommended for this textbook
- **Processor**: 2-core or better
- **Memory**: 4GB RAM minimum, 8GB+ recommended
- **Disk Space**: 5GB available space minimum
- **Internet Connection**: Required for installation and package updates

### Software Requirements
- `python3` (version 3.8 or higher)
- `wget` or `curl` for downloading packages
- `gnupg` for key management
- `lsb-release` to identify Ubuntu version
- `software-properties-common` for managing APT repositories

## Installing ROS2 Humble Hawksbill

ROS2 Humble Hawksbill is the LTS (Long Term Support) version that will be used throughout this textbook. It is supported until May 2027, making it ideal for educational use.

### Step 1: Set Locale

Ensure your locale is set to UTF-8:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Setup Sources

Add the ROS2 GPG key and repository:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the ROS2 repository to your sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS2 Packages

Update the package list and install ROS2:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

This command installs the "desktop" variant which includes all the commonly used packages for robotics development.

### Step 4: Environment Setup

Source the ROS2 setup script to make it available in your current terminal:

```bash
source /opt/ros/humble/setup.bash
```

To automatically source the ROS2 environment in new terminals, add it to your `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 5: Install Additional Tools

Install essential development tools:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

## Verifying Installation

Test that ROS2 is properly installed by running a simple demo:

```bash
# Open a new terminal and source ROS2
source /opt/ros/humble/setup.bash

# Run the talker demo in one terminal
ros2 run demo_nodes_cpp talker
```

In another terminal, run the listener:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see messages being published by the talker and received by the listener.

## Setting up Your First Workspace

Create a workspace directory structure following ROS2 conventions:

```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace
```

Your workspace structure should look like:
```
ros2_workspace/        # Your workspace
├── src/               # Source code
├── build/             # Build space (created after build)
├── install/           # Install space (created after build)
└── log/               # Logs (created after build)
```

## Development Tools

### Recommended IDEs
- **VS Code** with ROS2 extension
- **CLion** for C++ development
- **PyCharm** for Python development

### Essential Commands

```bash
# Create a new package
ros2 pkg create --build-type ament_cmake <package_name>

# Build packages in workspace
colcon build --packages-select <package_name>

# Source the workspace after building
source install/setup.bash

# List all available nodes
ros2 node list

# List all available topics
ros2 topic list
```

## Troubleshooting Common Issues

### Issue: GPG Key Error
If you encounter GPG key errors, ensure the key was properly added:
```bash
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Issue: Package Not Found
If packages are not found after installation:
```bash
sudo apt update
source /opt/ros/humble/setup.bash
```

### Issue: Permission Error
If you encounter permission errors with devices:
```bash
# Add your user to the dialout group for serial device access
sudo usermod -a -G dialout $USER
```

## Learning Objectives for This Section

After completing this section, you will be able to:
- Install ROS2 Humble Hawksbill on Ubuntu 22.04
- Set up the ROS2 environment in your terminal
- Verify that ROS2 is properly installed and running
- Create and configure a basic development workspace
- Troubleshoot common installation issues

## Next Steps

Once your environment is set up, you can:
- Explore the various ROS2 tutorials
- Create your first ROS2 package
- Start experimenting with ROS2 concepts

## Summary

Setting up a proper development environment is crucial for working with ROS2. Following the steps in this section ensures you have a stable foundation for learning and implementing robotic systems. The installation process may seem complex at first, but it provides access to a comprehensive ecosystem of tools and libraries that will be used throughout this textbook.