# Quickstart Guide: MDX Textbook Chapter Authoring

**Feature**: 002-mdx-textbook-chapters
**Date**: 2025-12-12
**Audience**: Content authors, ContentWriter Agent

## Overview

This guide walks you through creating a new textbook chapter for the Physical AI & Humanoid Robotics curriculum. Each chapter must include:

- ✅ Proper MDX frontmatter with learning objectives
- ✅ At least 3 executable Python/C++ code examples
- ✅ 4 exercises (2 beginner + 2 intermediate) with solutions
- ✅ Personalization markers for different experience levels
- ✅ Technical terminology preservation for translation

## Prerequisites

- Node.js 18+ (for Docusaurus)
- ROS 2 Humble (for code validation)
- Git access to repository

## Quick Start

### 1. Create Chapter File

```bash
# Navigate to appropriate module
cd docs/module-1-ros2-fundamentals/  # or module-2-isaac-sim, module-3-applications

# Create new chapter file
touch chapter-2-services.mdx
```

### 2. Add Required Frontmatter

Every chapter MUST start with this frontmatter structure:

```yaml
---
title: "Chapter 2: ROS 2 Topics & Services"
sidebar_position: 2
description: "Learn publish-subscribe patterns and request-response communication in ROS 2"
keywords: ["ROS2", "topics", "services", "QoS", "communication"]
tags: ["ros2", "fundamentals", "communication"]
learning_objectives:
  - "Implement publish-subscribe communication patterns"
  - "Create ROS 2 services for request-response interactions"
  - "Configure Quality of Service (QoS) settings"
  - "Debug communication issues between nodes"
difficulty_level: beginner
ros_version: humble
prerequisites: ["ch01"]
estimated_time: 60
personalization_variants:
  experience_level: true
  ros_familiarity: true
  hardware_access: false
---
```

### 3. Chapter Structure

Follow this consistent structure for all chapters:

```mdx
# Chapter N: Title

[Introduction paragraph - 2-3 sentences about the chapter topic]

## Learning Objectives

After completing this chapter, you will:
- [Objective 1]
- [Objective 2]
- [Objective 3]
- [Objective 4]

## N.1 Core Concept 1

### Key Concepts
[Explanatory content]

### Python Implementation
```python
# test: true
# ROS 2 Humble | Python 3.10+
# Dependencies: rclpy, std_msgs
[code]
```

### C++ Implementation
```cpp
// test: true
// ROS 2 Humble | C++17
// Dependencies: rclcpp, std_msgs
[code]
```

## N.2 Core Concept 2
[Continue pattern...]

## N.3 Practical Applications
[Real-world usage examples]

## Summary
[Bullet points of key learnings]

## Exercises

### Exercise 1: [Title] (Beginner)
[Full exercise with solution]

### Exercise 2: [Title] (Beginner)
[Full exercise with solution]

### Exercise 3: [Title] (Intermediate)
[Full exercise with solution]

### Exercise 4: [Title] (Intermediate)
[Full exercise with solution]

## Next Steps
[Preview of next chapter]
```

### 4. Code Example Requirements

Every code example must:

1. **Include test marker** (if executable):
```python
# test: true
# ROS 2 Humble | Python 3.10+
# Dependencies: rclpy, std_msgs
```

2. **Use standard ROS 2 packages only**:
   - Python: `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`
   - C++: `rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`

3. **Include inline comments** explaining key concepts:
```python
# Create publisher with topic name and queue size
self.publisher_ = self.create_publisher(
    String,         # Message type
    'hello_topic',  # Topic name
    10              # Queue size (QoS depth)
)
```

4. **Provide expected output**:
```
[INFO] [simple_publisher]: Publishing: "Hello World! Count: 0"
[INFO] [simple_publisher]: Publishing: "Hello World! Count: 1"
```

### 5. Personalization Sections

Use the `PersonalizedSection` component for experience-level variants:

```mdx
import PersonalizedSection from '@site/src/components/PersonalizedSection';

<PersonalizedSection level="beginner">

## Understanding Nodes (Beginner)

A **node** is like a small program that does one specific thing. Think of it as a worker in a factory - each worker has a specific job.

</PersonalizedSection>

<PersonalizedSection level="advanced">

## Node Architecture (Advanced)

ROS 2 nodes inherit from `rclcpp::Node` or `rclpy.node.Node` base classes, implementing the DDS (Data Distribution Service) middleware abstraction. Managed nodes extend this with lifecycle state machine support.

</PersonalizedSection>
```

### 6. Exercise Format

Follow this template for all exercises:

```mdx
### Exercise 1: Create a Temperature Publisher (Beginner)

**Learning Objective**: Apply publisher concepts to sensor data

#### Problem
Create a ROS 2 node that publishes simulated temperature readings every 2 seconds to a topic called `/temperature`.

#### Requirements
- Use `std_msgs/Float64` message type
- Generate random temperatures between 20.0 and 30.0°C
- Log each published value

#### Starter Code (Optional)
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# TODO: Import appropriate message type

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        # TODO: Create publisher
        # TODO: Create timer

    def timer_callback(self):
        # TODO: Generate and publish temperature
        pass

def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Solution
```python
# test: true
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float64, '/temperature', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Temperature publisher started')

    def timer_callback(self):
        msg = Float64()
        msg.data = random.uniform(20.0, 30.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Expected Output
```
[INFO] [temperature_publisher]: Temperature publisher started
[INFO] [temperature_publisher]: Publishing: 23.45°C
[INFO] [temperature_publisher]: Publishing: 27.82°C
[INFO] [temperature_publisher]: Publishing: 21.17°C
```

#### Troubleshooting
- **Import error for Float64**: Ensure you're importing from `std_msgs.msg`, not `std_msgs`
- **No output appearing**: Check that `rclpy.spin()` is being called
- **Topic not visible**: Run `ros2 topic list` to verify the topic is published
```

### 7. Technical Term Marking

Mark technical terms for translation preservation:

```mdx
import TechnicalTerm from '@site/src/components/TechnicalTerm';

The <TechnicalTerm term="publisher" /> sends messages to a <TechnicalTerm term="topic" />.
```

### 8. Validation Checklist

Before submitting a chapter, verify:

#### Frontmatter
- [ ] All required fields present
- [ ] `learning_objectives` has 3-7 items
- [ ] `tags` use lowercase with hyphens only
- [ ] `prerequisites` lists valid chapter IDs
- [ ] `estimated_time` is 30-90 minutes

#### Content
- [ ] Introduction explains chapter purpose
- [ ] Learning objectives listed at top
- [ ] 3+ code examples with both Python and C++
- [ ] All code examples have `# test: true` marker
- [ ] Inline comments explain key concepts
- [ ] Summary section recaps key learnings

#### Exercises
- [ ] 2 beginner exercises
- [ ] 2 intermediate exercises
- [ ] Each has problem, solution, expected output, troubleshooting
- [ ] Solutions are executable and tested

#### Personalization
- [ ] PersonalizedSection used for different experience levels
- [ ] Content variants are meaningful (not just simpler wording)

#### Quality
- [ ] Run `npm run build` to check for MDX errors
- [ ] Check readability (aim for Flesch-Kincaid 12-14)
- [ ] No broken links or missing images

## Common Mistakes to Avoid

### ❌ Wrong: No test marker
```python
import rclpy
# Missing # test: true
```

### ✅ Correct: Include test marker
```python
# test: true
import rclpy
```

### ❌ Wrong: Custom packages
```python
from my_custom_package.msg import CustomMessage
# Not available in CI environment
```

### ✅ Correct: Standard packages only
```python
from std_msgs.msg import String
# Available in ros:humble Docker image
```

### ❌ Wrong: Missing expected output
```mdx
### Solution
```python
[code]
```
# No expected output section
```

### ✅ Correct: Include expected output
```mdx
### Solution
```python
[code]
```

### Expected Output
```
[INFO] [node_name]: Output message
```
```

## File Structure Reference

```
docs/
├── module-1-ros2-fundamentals/
│   ├── _category_.json         # Module metadata
│   ├── chapter-1-publishers.mdx  # Ch1: Nodes & Lifecycle
│   ├── chapter-2-services.mdx    # Ch2: Topics & Services
│   └── chapter-3-parameters.mdx  # Ch3: Parameters & Launch
├── module-2-isaac-sim/
│   ├── _category_.json
│   ├── chapter-1-introduction.mdx # Ch4: Environment Setup
│   ├── chapter-2-urdf.mdx         # Ch5: Robot Models & URDF
│   └── chapter-3-sensors.mdx      # Ch6: Sensors & Physics
└── module-3-applications/
    ├── _category_.json
    ├── chapter-1-kinematics.mdx   # Ch7: Kinematics
    ├── chapter-2-navigation.mdx   # Ch8: Navigation & Path Planning
    ├── chapter-3-perception.mdx   # Ch9: Perception & Sensor Fusion
    └── chapter-4-humanoid.mdx     # Ch10: Humanoid Control
```

## Need Help?

- **MDX Syntax**: [Docusaurus MDX Guide](https://docusaurus.io/docs/markdown-features/react)
- **ROS 2 API**: [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- **Quality Issues**: Run `/sp.analyze` to check for consistency
