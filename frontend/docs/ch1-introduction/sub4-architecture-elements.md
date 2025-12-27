---
sidebar_position: 5
---

# 1.4. Nodes, Topics, Services, and Actions

## Introduction to ROS2 Communication

ROS2 uses a distributed computing model where different parts of a robot application can run on different machines and communicate with each other. This communication happens through several patterns: nodes, topics, services, and actions. Understanding these concepts is fundamental to developing effective robotic applications.

## Nodes

A **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS2 system. Multiple nodes are usually combined to form a complete robot application.

### Creating Nodes
- Nodes can be written in different programming languages (C++, Python, etc.)
- Each node runs within a process and communicates with other nodes
- Nodes can be started and stopped independently
- A single process can run multiple nodes if needed

### Node Responsibilities
- Executing specific robot functionality
- Publishing and subscribing to messages
- Providing and using services
- Executing actions

## Topics and Message Passing

**Topics** provide asynchronous communication through a publish/subscribe model. This is the most common communication pattern in ROS2.

### Key Characteristics
- **Unidirectional**: Publishers send messages, subscribers receive them
- **Asynchronous**: Publisher and subscriber don't need to run simultaneously
- **Many-to-many**: Multiple publishers can publish to one topic; multiple subscribers can subscribe to one topic
- **Typed**: Messages on a topic must all be of the same type

### Example Use Cases
- Sensor data streaming (camera images, LiDAR scans)
- Robot state monitoring (joint positions, battery levels)
- Control commands (velocity commands, motor commands)

### Quality of Service (QoS)
ROS2 introduces QoS settings that allow you to fine-tune communication behavior:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local
- **History**: Keep last N messages or keep all messages
- **Deadline**: Maximum time between consecutive messages

## Services

**Services** provide synchronous request/response communication. This is typically used for operations that need confirmation or return a specific result.

### Key Characteristics
- **Synchronous**: Client waits for response from server
- **Bidirectional**: Request goes one way, response goes the other
- **One-to-one**: One service server, one client at a time
- **Typed**: Both request and response have defined types

### Example Use Cases
- Map saving services
- Transform lookup services
- Calibration services
- Emergency stop activation

## Actions

**Actions** are for long-running tasks that require feedback, goal management, and the ability to cancel. They combine the best of services and topics.

### Key Characteristics
- **Long-running**: Suitable for operations that take time
- **Feedback**: Provides ongoing status during execution
- **Goal management**: Can accept/reject goals, monitor progress
- **Cancel capability**: Client can cancel ongoing goals
- **Preemption**: New goals can preempt existing ones

### Example Use Cases
- Navigation to goal location
- Trajectory execution
- Object manipulation tasks
- Calibration procedures

## Communication Architecture

### Publisher-Subscriber Pattern
```
[PUBLISHER NODE] ----publishes----> [TOPIC] ----subscribes----> [SUBSCRIBER NODE]
```

### Client-Server Pattern
```
[CLIENT NODE] ----requests----> [SERVICE] ----responses----> [SERVER NODE]
```

### Action Client-Server Pattern
```
[ACTION CLIENT] <---> [ACTION SERVER]
      |                  |
   goals/feedback    goals/feedback/results
```

## Quality of Service in Practice

QoS settings are crucial for mission-critical applications:

- **Reliable**: Ensures all messages are delivered (at cost of potential delay)
- **Best Effort**: Messages may be lost but delivered with minimal delay
- **Transient Local**: Late-joining subscribers can receive previous messages
- **Volatile**: Only new messages after subscription are available

## Code Example: Simple Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Code Example: Simple Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Communication Best Practices

### Topic Best Practices
- Use intuitive and descriptive topic names
- Follow naming conventions (lowercase, underscores)
- Choose appropriate message types
- Set appropriate QoS policies
- Avoid overloading topics with too much data

### Service Best Practices
- Use services for operations that return results
- Consider if the operation is appropriate for synchronous communication
- Handle service failures gracefully
- Set appropriate timeouts

### Action Best Practices
- Use actions for long-running operations
- Provide meaningful feedback during execution
- Implement goal preemption when appropriate
- Set realistic timeouts for goal execution

## Learning Objectives for This Section

After completing this section, you will be able to:
- Explain the differences between nodes, topics, services, and actions
- Choose the appropriate communication pattern for different use cases
- Understand Quality of Service settings and their applications
- Implement basic publishers and subscribers
- Recognize when to use services vs. actions vs. topics

## Summary

Nodes, topics, services, and actions form the backbone of ROS2 communication. Nodes encapsulate functionality, topics enable asynchronous message passing, services provide synchronous request-response patterns, and actions handle long-running operations with feedback. Understanding these concepts is essential for building robust robotic applications that can handle the complexity and real-time requirements of physical AI systems.