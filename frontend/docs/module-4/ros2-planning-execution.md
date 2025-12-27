---
title: Chapter 5 - ROS 2 Planning & Execution for Voice Commands
sidebar_label: Chapter 5
description: Learn about implementing ROS 2-based planning systems for voice-controlled robotics, integrating perception, planning, and execution in a unified framework.
keywords: [ROS 2, robot planning, execution, voice control, task planning, action execution, robot operating system, behavioral trees]
---

# Chapter 5: ROS 2 Planning & Execution for Voice Commands

## Learning Objectives

By the end of this chapter, you will be able to:
- Design ROS 2-based planning systems for voice-controlled robotics
- Implement action execution pipelines that respond to natural language commands
- Integrate perception, planning, and execution in a unified ROS 2 framework
- Ensure safety and reliability in voice-driven robotic systems

## Introduction to ROS 2 for Voice-Controlled Robotics

ROS 2 (Robot Operating System 2) provides the middleware and framework for developing robotic applications. For voice-controlled systems, ROS 2 enables the integration of speech recognition, natural language understanding, task planning, and robot execution in a distributed, reliable architecture.

### Key ROS 2 Concepts for Voice Control

- **Nodes**: Individual processes that perform specific functions (speech recognition, planning, execution)
- **Topics**: Communication channels for streaming data
- **Services**: Synchronous request-response communication
- **Actions**: Asynchronous goal-oriented communication with feedback
- **Parameters**: Configuration values that can be dynamically adjusted

## Architecture for Voice-Controlled ROS 2 Systems

### System Overview

```
[Voice Command] → [Speech Recognition] → [NLU] → [Task Planner] → [Execution Manager] → [Robot]
```

Each component runs as a ROS 2 node with well-defined interfaces.

### Key Node Types

#### Voice Interface Node
- Subscribes to audio streams or provides speech recognition service
- Publishes recognized text and confidence scores
- Handles wake word detection

#### Natural Language Understanding (NLU) Node
- Receives text from speech recognition
- Outputs structured commands or semantic representations
- Performs entity recognition and intent classification

#### Task Planning Node
- Translates high-level commands into executable action sequences
- Maintains world state and handles task decomposition
- Interfaces with perception systems for current state

#### Execution Manager Node
- Coordinates execution of action sequences
- Interfaces with low-level robot controllers
- Handles execution monitoring and failure recovery

## ROS 2 Action Architecture

Actions are particularly important for voice-controlled robotics as they provide long-running operations with feedback:

### Action Structure
- **Goal**: Desired outcome of the action
- **Feedback**: Intermediate status information
- **Result**: Final outcome of the action

### Example: Navigation Action
```
Goal: NavigationGoal(goal_pose=Pose(x=1.0, y=2.0, theta=0.0))
Feedback: NavigationFeedback(current_pose=Pose(...), distance_remaining=0.5)
Result: NavigationResult(accuracy=0.1, execution_time=5.2)
```

### Action Client Implementation
```python
# Example ROS 2 action client for navigation
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class VoiceNavigationController:
    def __init__(self):
        self.navigation_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
    
    async def execute_navigation(self, x, y, theta):
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = quat_from_euler(0, 0, theta)
        
        future = await self.navigation_client.send_goal_async(goal)
        return await future.result()
```

## Planning Systems in ROS 2

### Hierarchical Task Networks (HTNs)

Represent complex tasks as compositions of simpler tasks:

```python
# Example HTN for object transportation
def transport_object(object_id, destination):
    return [
        navigate_to(get_object_location(object_id)),
        grasp_object(object_id),
        navigate_to(destination),
        release_object(object_id)
    ]
```

### Integration with ROS 2

HTNs can be implemented as ROS 2 services or action servers:

```python
# Task planning service
from rclpy.node import Node
from std_msgs.msg import String

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        self.srv = self.create_service(
            PlanTask, 
            'plan_task', 
            self.plan_task_callback
        )

    def plan_task_callback(self, request, response):
        # Parse high-level command and generate action sequence
        action_sequence = self.decompose_task(request.command)
        response.plan = action_sequence
        return response
```

## Execution Framework

### Behavior Trees in ROS 2

Behavior trees provide a structured way to compose complex behaviors:

```xml
<!-- Example behavior tree for object fetching -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <FindObject object="{target_object}" />
            <NavigateTo pose="{object_location}" />
            <GraspObject object="{target_object}" />
            <NavigateTo pose="{delivery_location}" />
            <ReleaseObject object="{target_object}" />
        </Sequence>
    </BehaviorTree>
</root>
```

### ROS 2 Integration

Behavior trees can be implemented using `behaviortree_cpp` with ROS 2:

```cpp
// Custom ROS 2 action node for behavior tree
class ROSNavigateTo : public BT::AsyncActionNode
{
public:
    ROSNavigateTo(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config), node_(rclcpp::Node::make_shared("bt_navigate"))
    {
        nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    // Implementation of the tick method
    BT::NodeStatus tick() override;
    void halt() override;
};
```

## Safety and Reliability Considerations

### Safety Levels

ROS 2 supports different safety levels for voice-controlled systems:

1. **Safety Level 1**: Basic collision avoidance
2. **Safety Level 2**: Human detection and avoidance
3. **Safety Level 3**: Emergency stop on verbal command

### Safety Implementation

```python
class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        self.subscription = self.create_subscription(
            ObstacleArray,
            'obstacles',
            self.obstacle_callback,
            10
        )
        self.emergency_publisher = self.create_publisher(
            EmergencyStop,
            'emergency_stop',
            10
        )

    def obstacle_callback(self, msg):
        if self.detect_collision_risk(msg):
            self.emergency_publisher.publish(EmergencyStop())
```

### Failure Handling

Voice-controlled systems must handle various failure modes:

#### Speech Recognition Failure
- Retry with different acoustic models
- Request user to repeat command
- Switch to alternative input mode

#### Planning Failure
- Identify infeasible goals
- Propose alternative solutions
- Request clarification from user

#### Execution Failure
- Monitor execution progress
- Recover from partial failures
- Report status to user

## Voice Command Processing Pipeline

### Complete ROS 2 Pipeline

```
Microphone → Audio Topic → Speech Recognition Node → Text Topic → NLU Node
     ↓
Text Topic → Task Planning Node → Action Sequence → Execution Manager → Robot
     ↓
Action Results ← Robot Controllers ← Action Goals ← Execution Manager
```

### Example Implementation

```python
# Complete voice command processing node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from your_msgs.action import ExecuteTask

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')
        
        # Subscribe to recognized speech
        self.speech_subscriber = self.create_subscription(
            String,
            'recognized_speech',
            self.speech_callback,
            10
        )
        
        # Create action client for task execution
        self.task_executor = ActionClient(
            self, ExecuteTask, 'execute_task'
        )
        
    def speech_callback(self, msg):
        # Parse command and execute appropriate task
        command = self.parse_command(msg.data)
        self.execute_task(command)
    
    def parse_command(self, text):
        # Natural language parsing logic
        # Convert text to structured command
        pass
    
    def execute_task(self, command):
        # Send command to task execution system
        goal = ExecuteTask.Goal(command=command)
        self.task_executor.send_goal_async(goal)
```

## Coordination and Synchronization

### State Management

Maintaining consistent state across all nodes:

```python
# State tracking service
from rclpy.qos import QoSProfile
from your_msgs.srv import GetState, SetState

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        self.state = {}
        
        self.get_state_service = self.create_service(
            GetState, 'get_state', self.get_state_callback
        )
        self.set_state_service = self.create_service(
            SetState, 'set_state', self.set_state_callback
        )
    
    def get_state_callback(self, request, response):
        response.state = self.state.get(request.key, None)
        return response
```

### Task Coordination

Coordinating tasks with other system activities:

```python
# Task coordinator node
class TaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')
        
        # Monitor current tasks
        self.active_tasks = []
        
        # Coordinate new tasks with active ones
        self.task_queue = []
    
    def can_execute_task(self, new_task):
        # Check for conflicts with active tasks
        for active_task in self.active_tasks:
            if self.tasks_conflict(active_task, new_task):
                return False
        return True
```

## Performance Optimization

### Real-time Considerations

Voice-controlled systems require low latency:

- **Audio Processing**: &lt;10ms for real-time response
- **Speech Recognition**: &lt;200ms for natural interaction
- **Planning**: &lt;500ms for task decomposition
- **Execution**: &lt;1000ms for feedback

### Resource Management

Efficient use of computational resources:

```python
# Resource monitor node
class ResourceMonitor(Node):
    def __init__(self):
        super().__init__('resource_monitor')
        
        # Monitor CPU, memory, and I/O usage
        self.resource_timer = self.create_timer(
            1.0, self.check_resources
        )
    
    def check_resources(self):
        cpu_usage = self.get_cpu_usage()
        memory_usage = self.get_memory_usage()
        
        if cpu_usage > 0.9:
            self.get_logger().warn("High CPU usage detected")
        
        if memory_usage > 0.9:
            self.get_logger().warn("High memory usage detected")
```

## Testing and Validation

### Simulation Testing

Testing voice-controlled behaviors in simulation before deployment:

```python
# Example test case
import unittest
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing.actions

def generate_test_description():
    return LaunchDescription([
        # Launch voice control nodes
        Node(
            package='voice_control',
            executable='voice_processor',
            name='voice_processor'
        ),
        # Run tests
        launch_testing.actions.ReadyToTest(),
    ])

class TestVoiceControl(unittest.TestCase):
    def test_navigation_command(self, proc_output):
        # Test that navigation command is correctly processed
        pass
```

### Integration Testing

Testing the complete voice-to-action pipeline:

- End-to-end command processing
- Error handling and recovery
- Safety system activation
- Performance under stress

## Summary

This chapter covered the implementation of ROS 2-based planning and execution systems for voice-controlled robotics. The next chapter will provide a complete capstone example integrating all concepts from this module.

## Review Questions

1. What are the advantages of using ROS 2 actions over topics for voice-controlled robotics?
2. Explain how hierarchical task networks can be implemented in ROS 2.
3. What are the key safety considerations for voice-controlled robotic systems?
4. How can behavior trees be integrated with ROS 2 for complex task execution?

## Next Steps

Previous: [Chapter 4: Multimodal Perception](../module-4/04-multimodal-perception.md)
Continue to [Chapter 6: Capstone Autonomous Humanoid](../module-4/06-capstone-humanoid.md)