# Quickstart Guide: Physical AI Textbook

## Getting Started with the Physical AI & Humanoid Robotics Textbook

This quickstart guide will help you set up the environment and begin engaging with the Physical AI & Humanoid Robotics textbook content.

## Prerequisites

Before starting with the textbook, ensure you have:

- Basic programming knowledge (Python recommended)
- Understanding of fundamental mathematics (linear algebra, calculus)
- Familiarity with basic physics concepts
- A computer capable of running simulation software (minimum 8GB RAM, 4 cores recommended)

## Setup Environment

### 1. Install Required Software

The textbook uses several key technologies. Install them in this order:

#### Ubuntu 22.04 LTS (Recommended)
Most examples in the textbook are designed for Ubuntu. If you don't have it:
- Use a virtual machine
- Use WSL2 if on Windows
- Use a cloud instance

#### ROS2 Humble Hawksbill
```bash
# Add ROS2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# Install ROS2
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2
sudo rosdep init
rosdep update

# Source ROS2
source /opt/ros/humble/setup.bash
```

#### Python 3.10+ and Essential Libraries
```bash
sudo apt install python3-pip python3-dev
pip3 install numpy scipy matplotlib opencv-python transforms3d
```

#### Gazebo Garden
```bash
curl -sSL http://get.gazebosim.org | sh
```

### 2. Download Textbook Resources

Clone the textbook companion repository (if available):
```bash
git clone https://github.com/[publisher]/physical-ai-textbook-examples.git
cd physical-ai-textbook-examples
```

### 3. Verify Installation

Test your setup with a simple ROS2 command:
```bash
# In one terminal
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

You should see messages passing between the talker and listener nodes.

## Getting Started with the Content

### For Students

1. **Start with Chapter 1** - This establishes essential concepts and sets up your development environment.
2. **Follow the weekly plan** - The textbook includes a 14-week curriculum. Try to stick to the schedule to build knowledge progressively.
3. **Complete all labs** - Each week includes hands-on lab exercises. These are critical for understanding practical applications.
4. **Use the appendices** - The glossary and troubleshooting guides will be valuable references throughout your learning journey.

### For Instructors

1. **Review the curriculum plan** - The 14-week plan provides a structured approach to teaching physical AI concepts.
2. **Adapt to your institution** - Hardware recommendations are tiered to accommodate different budgets and capabilities.
3. **Prepare lab environments** - Ensure your lab has the necessary hardware and software before starting.
4. **Integrate with your schedule** - The modular structure allows for adaptation to different semester lengths.

## Essential Hardware (Recommended)

For the best experience with the textbook, consider acquiring:

### Entry Level (Budget Conscious)
- TurtleBot3 Burger platform (or similar differential drive robot)
- Computer with ROS2-compatible OS
- Intel RealSense D435i camera

### Intermediate Level (Well-Equipped Lab)
- 6-dof robotic arm (UR3e or similar)
- High-performance workstation with dedicated GPU
- Multiple sensors (LiDAR, cameras, IMU)

### Advanced Level (Research-Focused)
- Mobile manipulator platform
- High-end computing cluster
- Advanced sensors and actuators

## Common First Steps

1. **Week 1-2**: Focus on ROS2 fundamentals and basic robot control
2. **Week 3-4**: Explore perception systems and simulation environments
3. **Week 5-6**: Dive into the Isaac platform for advanced robotics applications

## Troubleshooting

### Common Issues

- **ROS2 commands not found**: Make sure you've sourced the ROS2 environment (`source /opt/ros/humble/setup.bash`)
- **Simulation performance**: Close unnecessary applications; consider reducing visual quality in simulation settings
- **Code examples don't work**: Ensure you're using the exact versions of libraries specified in the textbook

### Getting Help

- Check Appendix B (Troubleshooting Guide) for detailed solutions
- Consult the ROS2 documentation at docs.ros.org
- Join the textbook's community forum (if available)

## Next Steps

After completing the initial setup:
1. Read Chapter 1 to understand the foundational concepts of Physical AI
2. Complete Lab 1.1 and Lab 1.2 to verify your environment is properly configured
3. Set up your development workflow with version control
4. Join the community or discussion forum related to the textbook (if available)

## Additional Resources

- ROS2 Documentation: docs.ros.org
- Gazebo Documentation: gazebosim.org
- Isaac ROS Documentation: nvidia.com/isaac-ros
- Computer Vision Resources: OpenCV documentation and tutorials

---

Start your journey into Physical AI with confidence. The textbook is designed to take you from fundamental concepts to advanced implementations, with ample opportunities for hands-on learning at each step.