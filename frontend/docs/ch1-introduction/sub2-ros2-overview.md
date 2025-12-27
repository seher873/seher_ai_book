---
sidebar_position: 3
---

# 1.2. Overview of Robotic Operating System (ROS2)

## What is ROS2?

The Robot Operating System 2 (ROS2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Despite its name, ROS2 is not a real operating system but rather a middleware that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## Key Differences from ROS1

ROS2 was developed to address limitations in the original ROS framework:

1. **Real-time Support**: ROS2 provides real-time capabilities that were missing in ROS1
2. **Multi-robot Systems**: Better support for multiple robots working together
3. **Commercial Deployment**: Improved security and reliability features for production use
4. **Architecture**: Based on DDS (Data Distribution Service) for communication
5. **Quality of Service**: Configurable delivery guarantees for messages

## ROS2 Architecture

ROS2 uses a distributed architecture based on the DDS (Data Distribution Service) standard. This provides:

- **Decentralized communication**: No central master node
- **Language independence**: Support for multiple programming languages
- **Platform portability**: Runs on various operating systems
- **Quality of Service (QoS)**: Configurable reliability and performance options

### Core Components

- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Goal-oriented communication with feedback
- **Parameters**: Configuration values that can be changed at runtime

## Installation and Setup

ROS2 is designed to work on multiple platforms:
- Ubuntu Linux (primary platform)
- Windows
- macOS
- Real-time systems

The most common distributions are released with Ubuntu LTS versions and are supported for 5 years.

## Learning Objectives for This Section

After completing this section, you will be able to:
- Explain the purpose and benefits of ROS2
- Identify key differences between ROS1 and ROS2
- Understand the core architectural concepts of ROS2
- Recognize the main components of a ROS2 system

## ROS2 Ecosystem

The ROS2 ecosystem includes:

- **Distributions**: Time-based releases of the ROS2 platform
- **Packages**: Collections of code and resources for specific functions
- **Tools**: Command-line tools and GUI applications for development
- **Community**: Active community providing support and resources

## Standards and Conventions

ROS2 follows several important conventions:

- **ROS Enhancement Proposals (REPs)**: Standards for ROS2 development
- **Common Interfaces**: Standardized message types and services
- **Repository Organization**: Consistent package structure and layout
- **Documentation Standards**: Consistent documentation across packages

## Community and Support

The ROS2 community provides various resources:
- Tutorials and documentation
- Community forums and support
- Regular events and conferences
- Training and certification programs

## Summary

ROS2 represents a significant evolution from ROS1, providing improved real-time capabilities, enhanced security, and better support for commercial applications. Its distributed architecture based on DDS enables more robust robot applications that can scale from simple research platforms to complex multi-robot systems.