---
sidebar_position: 10
---

# Module 1: ROS2 Deep Dive

## Technical Topic: Advanced ROS2 concepts including custom message types, parameters, lifecycle nodes, and performance optimization.

## Content

## 1. Custom Message and Service Definitions

### Creating Custom Messages

ROS2 allows you to define custom message types for specialized data structures. Custom messages are defined using the `.msg` specification language.

**Message Definition Structure:**
```
# Comments begin with # character
field_type field_name
field_type field_name  # Comments can appear after fields
```

**Example custom message (geometry_msgs/Point32.msg):**
```
# This contains the position of a point in free space
float32 x
float32 y
float32 z
```

### Creating Message Package

```bash
# Create a new package for messages
ros2 pkg create --build-type ament_cmake my_robot_msgs
```

Then add the message definitions to the `msg/` directory in the package.

**package.xml additions:**
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**CMakeLists.txt additions:**
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "srv/CustomService.srv"
  "action/CustomAction.action"
)
```

### Custom Services

Service definitions use the `.srv` extension and follow this format:
```
# Request
field_type field_name
---
# Response
field_type field_name
```

### Custom Actions

Action definitions use the `.action` extension with three sections:
```
# Goal
field_type field_name
---
# Result
field_type field_name
---
# Feedback
field_type field_name
```

## 2. Parameter Management and Configuration

### Parameter Basics

Parameters in ROS2 are key-value pairs that configure node behavior. They can be:

- Declared at compile time
- Dynamically configured at runtime
- Loaded from configuration files

### Declaring Parameters

```cpp
// In C++
this->declare_parameter("param_name", default_value);
```

```python
# In Python
self.declare_parameter('param_name', default_value)
```

### Parameter Callbacks

You can define callbacks that execute when parameters change:

```python
def parameter_callback(self, parameters):
    for param in parameters:
        if param.name == 'my_param':
            # Handle parameter change
            print(f"Parameter changed: {param.value}")
    return SetParametersResult(successful=True)

self.set_parameters_callback(self.parameter_callback)
```

### Loading Parameters from YAML

Parameters can be loaded from YAML files:

```yaml
# my_robot_params.yaml
/**:
  ros__parameters:
    frequency: 10.0
    timeout: 5.0
    calibration_data: [1.0, 2.0, 3.0]
```

Loading parameters:
```bash
ros2 param load <node_name> <param_file.yaml>
```

## 3. Lifecycle Nodes for Robotic Applications

### Understanding Lifecycle Nodes

Lifecycle nodes provide a state machine for robot components, allowing for more robust management of complex systems. The states are:

- Unconfigured
- Inactive
- Active
- Finalized

### Creating a Lifecycle Node

```python
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import on_configure, on_activate, on_deactivate, on_cleanup, on_shutdown

class MyLifecycleNode(LifecycleNode):
    def __init__(self):
        super().__init__('my_lifecycle_node')
    
    @on_configure
    def on_configure(self, state):
        # Initialize resources, but don't activate
        self.get_logger().info("Configuring node")
        # Setup publishers/subscribers (not yet active)
        self.pub = self.create_publisher(String, 'topic', 10)
        return TransitionCallbackReturn.SUCCESS
    
    @on_activate
    def on_activate(self, state):
        # Start operations
        self.get_logger().info("Activating node")
        # Activate publishers/subscribers
        self.pub.on_activate()
        # Start timers, threads, etc.
        return TransitionCallbackReturn.SUCCESS
    
    @on_deactivate
    def on_deactivate(self, state):
        # Pause operations
        self.get_logger().info("Deactivating node")
        # Deactivate publishers/subscribers
        self.pub.on_deactivate()
        return TransitionCallbackReturn.SUCCESS
```

### Managing Lifecycle Nodes

```bash
# Get current state
ros2 lifecycle get <node_name>

# Trigger state transitions
ros2 lifecycle configure <node_name>
ros2 lifecycle activate <node_name>
ros2 lifecycle deactivate <node_name>
ros2 lifecycle cleanup <node_name>
ros2 lifecycle shutdown <node_name>
```

## 4. Performance Profiling and Optimization

### CPU Profiling

ROS2 provides several tools for performance analysis:

**Tracetools for tracing:**
```bash
# Install required packages
sudo apt install ros-humble-tracetools ros-humble-tracetools-launch

# Record traces
ros2 trace my_trace --all
```

**Using ros2 doctor for diagnostics:**
```bash
ros2 doctor
```

### Memory Optimization

- Use message pools to reduce allocation overhead
- Implement object reuse where possible
- Monitor memory usage with system tools

**Example of message pooling:**
```cpp
#include <rclcpp/rclcpp.hpp>

auto msg = std::make_shared<std_msgs::msg::String>();
msg->data = "Hello World";
publisher->publish(msg);
```

### Network Optimization

#### Quality of Service (QoS) Settings

Choose appropriate QoS profiles for your use case:

```cpp
// For critical data
rclcpp::QoS qos_profile(10);
qos_profile.reliable();
qos_profile.durability_volatile();

// For real-time data
rclcpp::QoS qos_profile(10);
qos_profile.best_effort();
qos_profile.durability_volatile();
```

#### Common QoS Settings
- **Reliability**: RELIABLE for important data, BEST_EFFORT for real-time
- **Durability**: TRANSIENT_LOCAL for latching, VOLATILE for streaming
- **History**: KEEP_LAST or KEEP_ALL
- **Depth**: Size of message queue

### Threading Models

ROS2 supports different executor models:

**Single Threaded Executor:**
```python
executor = SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

**Multi Threaded Executor:**
```python
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

## 5. Testing with rostest and gtest

### Writing Component Tests

**For C++ using gtest:**
```cpp
#include <gtest/gtest.h>

TEST(TestSuite, TestName) {
    // Your test code here
    ASSERT_EQ(result, expected_value);
}
```

**In CMakeLists.txt:**
```cmake
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_name test_file.cpp)
target_link_libraries(test_name ${PROJECT_NAME}_library)
```

### Integration Tests with launch testing

```python
import launch
import launch_testing
import pytest

@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='my_package',
        executable='my_node',
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ])

def test_node_output(output):
    output.assertWaitFor('Node started', timeout=10)
```

## 6. Advanced Debugging Techniques

### Remote Debugging

You can debug ROS2 nodes running on remote systems using GDB:

```bash
# Attach to a running process
gdb -p $(pgrep -f node_name)

# Or start with GDB
gdb --args ros2 run package_name node_name
```

### Logging Configuration

Configure logging levels and outputs:

```cpp
// Set logging level at runtime
rclcpp::Logger logger = rclcpp::get_logger("node_name");
RCLCPP_INFO(logger, "This is an info message");
RCLCPP_DEBUG(logger, "This is a debug message");
RCLCPP_ERROR(logger, "This is an error message");
```

### Using lldb for debugging

```bash
# Using lldb instead of gdb
lldb -- ros2 run package_name node_name
```

## Key Takeaways

1. Custom messages allow you to define specialized data structures for your robot applications
2. Parameters provide a flexible way to configure node behavior without recompilation
3. Lifecycle nodes offer robust state management for complex robotic systems
4. Performance profiling is essential for identifying bottlenecks in real-time systems
5. Proper testing and debugging techniques are crucial for reliable robot applications

## Summary

This module explored advanced ROS2 concepts essential for developing production-ready robotic applications. From custom message definitions to lifecycle management and performance optimization, these techniques enable more robust and efficient robot software systems.

## Related Resources

- [Glossary of Terms](../glossary/index.md)
- [Appendix B: Chatbot and RAG Technologies](../appendices/appendix-b-chatbot-rag.md)