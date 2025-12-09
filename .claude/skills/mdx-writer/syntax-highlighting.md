# MDX Syntax Highlighting Configuration

This document provides syntax highlighting support for ROS 2 and Isaac Sim code examples in MDX files.

## Supported Languages

### ROS 2
- Python (`python` or `py`)
- C++ (`cpp` or `cxx`)
- YAML (`yaml` or `yml`)
- Launch XML (`xml`)

### Isaac Sim
- Python (`python` with Isaac Sim imports)
- USD (`usd`)
- JSON (`json`)

### Robotics-Specific
- URDF (`xml` with urdf class)
- SDF (`xml` with sdf class)
- Robot Framework (`robotframework`)

## Code Block Examples

### ROS 2 Python Node
```python
"""Example ROS 2 node for joint control"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

    def joint_callback(self, msg):
        self.get_logger().info(f'Received: {msg.position}')
```

### Isaac Sim Script
```python
"""Isaac Sim simulation setup"""
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path

async def create_scene():
    world = World()
    assets_root_path = get_assets_root_path()

    # Load robot asset
    result, robot = omni.kit.commands.execute(
        "IsaacRobotLoadCommand",
        robot_path=f"{assets_root_path}/Isaac/Robots/Humanoid/humanoid.usd"
    )

    return world
```

### URDF Robot Description
```xml
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_joint" type="revolute">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>
</robot>
```

## Custom CSS Classes

Add these classes to your Docusaurus configuration for better robotics code formatting:

```css
/* ROS 2 specific styling */
.language-ros2 .token.builtin {
  color: #e06c75;
}

/* Isaac Sim specific styling */
.language-isaac .token.function {
  color: #61afef;
}
```

## MDX Component Usage

```mdx
import CodeBlock from '@theme/CodeBlock';

<CodeBlock className="language-python" title="ROS 2 Node">
{`import rclpy
from rclpy.node import Node`}
</CodeBlock>
```