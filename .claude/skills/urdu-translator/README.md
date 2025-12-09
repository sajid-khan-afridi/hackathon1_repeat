# Urdu Translator Skill

A Claude Code skill for translating technical MDX content to Urdu while preserving code blocks, technical terms, and formatting.

## Features

- **Technical Term Preservation**: Keeps technical terms like ROS, NVIDIA, SLAM, etc. in English
- **Code Block Preservation**: Maintains code blocks and inline code exactly as written
- **URL and File Path Preservation**: Keeps all URLs and file paths unchanged
- **RTL Support**: Automatically adds right-to-left direction for Urdu text
- **MDX Compatibility**: Handles frontmatter, JSX components, and MDX-specific syntax
- **Image Alt Text Translation**: Translates image alt text while preserving the image references
- **GPT-4 Powered**: Uses OpenAI's GPT-4 for high-quality translations

## Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set your OpenAI API key:
```bash
export OPENAI_API_KEY="your-api-key-here"
```

## Usage

### As a Tool

```python
from tools.translate_to_urdu import translate_to_urdu

content = """
# ROS Tutorial

This tutorial covers ROS (Robot Operating System) basics.

```python
import rospy
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
```

Visit https://www.ros.org for more information.
"""

result = translate_to_urdu(
    content,
    preserve_terms=["ROS", "Robot Operating System"],
    api_key="your-openai-api-key"
)

if result["success"]:
    print(result["translated_content"])
else:
    print(f"Error: {result['error']}")
```

### Command Line

```bash
python tools/translate-to-urdu.py input.mdx output.ur.mdx
```

## Configuration

### Environment Variables

- `OPENAI_API_KEY`: Your OpenAI API key (required)

### Preserve Terms

The translator automatically preserves these technical terms:
- ROS, NVIDIA, Isaac, SLAM, LiDAR, IMU, URDF
- API, JSON, XML, HTML, CSS, JavaScript, Python
- C++, CUDA, TensorRT, Docker, Kubernetes, Linux
- Ubuntu, Git, GitHub, CLI, SDK, IDE, CPU, GPU

You can add custom terms to preserve:
```python
custom_terms = ["YourCustomTerm", "AnotherTerm"]
translate_to_urdu(content, preserve_terms=custom_terms)
```

## Supported Elements

### Preserved Elements
- Code blocks (```...```)
- Inline code (`...`)
- URLs (https://...)
- File paths (./file.py)
- JSX components (<Component>)
- Import/export statements
- Frontmatter (YAML)
- Image URLs

### Translated Elements
- Regular text
- Image alt text
- Headings and titles
- List items
- Blockquotes

## Example

### Input:
```mdx
---
title: "ROS2 Navigation"
author: "Robotics Team"
---

# ROS2 Navigation Stack

This guide covers the ROS2 navigation stack.

<Callout type="warning">
Ensure your robot has proper sensor configuration (LiDAR, IMU).
</Callout>

## Configuration File

```yaml
ros__parameters:
  use_sim_time: true
  robot_base_frame: "base_link"
```

## Setup Instructions

1. Launch the navigation stack:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py
   ```

2. Set initial pose using RViz.

For more details, visit https://docs.ros.org.

![Navigation Stack](./images/nav2-stack.png)
```

### Output:
```mdx
---
title: "ROS2 Navigation"
author: "Robotics Team"
---

<div dir="rtl">

# ROS2 Navigation Stack

یہ گائیڈ ROS2 navigation سٹیک کو �ھالتا ہے۔

<Callout type="warning">
یقینی بنائیں کہ آپ کے روبوٹ کے پاس مناسب سینسر کنفیگریشن ہے (LiDAR, IMU)۔
</Callout>

## Configuration File

```yaml
ros__parameters:
  use_sim_time: true
  robot_base_frame: "base_link"
```

## Setup Instructions

1. Launch the navigation stack:
   ```bash
   ros2 launch nav2_bringup navigation_launch.py
   ```

2. RViz کا استعمال کرکے ابتدائی_pose سیٹ کریں۔

مزید تفصیلات کے لیے https://docs.ros.org ملاحظہ کریں۔

![Navigation Stack](./images/nav2-stack.png)

</div>
```

## Testing

Run the test suite:

```bash
python test_urdu_translator.py
```

## Limitations

1. Requires OpenAI API access
2. Translation quality depends on GPT-4's understanding of technical concepts
3. Some complex JSX components might need manual review
4. Very large files may need to be split due to API limits

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

MIT License - see LICENSE file for details.