#!/usr/bin/env python3
"""
Test suite for Urdu translator skill.
"""

import unittest
from unittest.mock import patch, MagicMock
import sys
import os

# Add the skill directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tools.translate_to_urdu import (
    MDXPreserver,
    UrduTranslator,
    translate_to_urdu
)
from utils.mdx_utils import MDXProcessor, ImageProcessor


class TestMDXPreserver(unittest.TestCase):
    """Test MDX preservation functionality."""

    def setUp(self):
        self.preserver = MDXPreserver(["ROS", "NVIDIA", "Isaac"])

    def test_extract_code_blocks(self):
        """Test extraction of code blocks."""
        content = """# Tutorial

Here is some Python code:

```python
def hello():
    print("Hello World")
```

And some JavaScript:

```javascript
const x = 10;
```"""

        preserved, placeholder_map = self.preserver.extract_and_preserve(content)

        # Check that code blocks are replaced with placeholders
        self.assertIn("__CODE_BLOCK_0__", preserved)
        self.assertIn("__CODE_BLOCK_1__", preserved)
        self.assertNotIn("def hello():", preserved)

        # Check placeholder map
        self.assertEqual(placeholder_map["__CODE_BLOCK_0__"]["lang"], "python")
        self.assertIn("def hello():", placeholder_map["__CODE_BLOCK_0__"]["content"])

    def test_extract_inline_code(self):
        """Test extraction of inline code."""
        content = "Use the `rosrun` command to run nodes."
        preserved, placeholder_map = self.preserver.extract_and_preserve(content)

        self.assertIn("__INLINE_CODE_0__", preserved)
        self.assertNotIn("`rosrun`", preserved)
        self.assertEqual(placeholder_map["__INLINE_CODE_0__"]["content"], "rosrun")

    def test_extract_urls(self):
        """Test extraction of URLs."""
        content = "Visit https://docs.ros.org for more info."
        preserved, placeholder_map = self.preserver.extract_and_preserve(content)

        self.assertIn("__URL_0__", preserved)
        self.assertEqual(
            placeholder_map["__URL_0__"]["content"],
            "https://docs.ros.org"
        )

    def test_extract_file_paths(self):
        """Test extraction of file paths."""
        content = "Edit the file src/main.py to change the behavior."
        preserved, placeholder_map = self.preserver.extract_and_preserve(content)

        self.assertIn("__FILEPATH_0__", preserved)
        self.assertEqual(
            placeholder_map["__FILEPATH_0__"]["content"],
            "src/main.py"
        )

    def test_restore_preserved(self):
        """Test restoration of preserved elements."""
        original = "Use `rosrun` and visit https://docs.ros.org"
        preserved, placeholder_map = self.preserver.extract_and_preserve(original)

        # Simulate translation (just add Urdu text)
        translated = f"برائے مہربانی __INLINE_CODE_0__ استعمال کریں اور __URL_0__ ملاحظہ کریں"

        restored = self.preserver.restore_preserved(translated, placeholder_map)

        self.assertIn("`rosrun`", restored)
        self.assertIn("https://docs.ros.org", restored)
        self.assertNotIn("__INLINE_CODE_0__", restored)


class TestMDXProcessor(unittest.TestCase):
    """Test MDX processing utilities."""

    def setUp(self):
        self.processor = MDXProcessor()

    def test_extract_frontmatter(self):
        """Test frontmatter extraction."""
        content = """---
title: "ROS Tutorial"
author: "John Doe"
date: "2024-01-01"
---

# Introduction

This is the content."""

        content_without, frontmatter_dict, raw = self.processor.extract_frontmatter(content)

        self.assertEqual(frontmatter_dict["title"], "ROS Tutorial")
        self.assertEqual(frontmatter_dict["author"], "John Doe")
        self.assertIn("This is the content", content_without)
        self.assertNotIn("---", content_without)

    def test_extract_jsx_components(self):
        """Test JSX component extraction."""
        content = """<Callout type="warning">
This is a warning message.
</Callout>

<CustomButton onClick={handleClick}>
Click me
</CustomButton>

<Logo />"""

        preserved, components = self.processor.extract_jsx_components(content)

        self.assertIn("__JSX_COMPONENT_0__", preserved)
        self.assertIn("__JSX_COMPONENT_1__", preserved)
        self.assertIn("__JSX_SELF_CLOSING_2__", preserved)

        self.assertEqual(components[0]["name"], "Callout")
        self.assertEqual(components[2]["name"], "Logo")
        self.assertTrue(components[2].get("self_closing"))

    def test_add_rtl_to_components(self):
        """Test adding RTL support to components."""
        content = "<p>Hello world</p>\n<div>No dir here</div>"
        rtl_content = self.processor.add_rtl_to_components(content)

        self.assertIn('dir="rtl"', rtl_content)
        self.assertIn("<p dir=\"rtl\">", rtl_content)


class TestImageProcessor(unittest.TestCase):
    """Test image processing."""

    def setUp(self):
        self.processor = ImageProcessor()

    def test_extract_markdown_images(self):
        """Test extraction of markdown images."""
        content = "![Robot Image](./images/robot.png)"
        preserved, images = self.processor.extract_images(content)

        self.assertIn("__IMAGE_0__", preserved)
        self.assertEqual(images[0]["alt_text"], "Robot Image")
        self.assertEqual(images[0]["url"], "./images/robot.png")

    def test_extract_jsx_images(self):
        """Test extraction of JSX img tags."""
        content = '<img src="./robot.jpg" alt="A robot" className="w-full" />'
        preserved, images = self.processor.extract_images(content)

        self.assertIn("__JSX_IMG_0__", preserved)
        self.assertEqual(images[0]["alt_text"], "A robot")
        self.assertTrue(images[0].get("jsx"))


class TestUrduTranslator(unittest.TestCase):
    """Test the main translator class."""

    def setUp(self):
        self.sample_content = """# ROS Tutorial

This tutorial covers ROS (Robot Operating System) basics.

## Code Example

```python
import rospy

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
```

## Important Links

Visit https://www.ros.org for more information.

![ROS Logo](./images/ros-logo.png)"""

    @patch('tools.translate_to_urdu.OpenAI')
    def test_translate_success(self, mock_openai):
        """Test successful translation."""
        # Mock the OpenAI response
        mock_client = MagicMock()
        mock_openai.return_value = mock_client

        mock_response = MagicMock()
        mock_response.choices = [
            MagicMock(message=MagicMock(content="__TRANSLATED_CONTENT__"))
        ]
        mock_client.chat.completions.create.return_value = mock_response

        translator = UrduTranslator(api_key="test_key")

        with patch.object(translator, '_add_rtl_support', return_value="__FINAL_TRANSLATION__"):
            result = translator.translate_to_urdu(self.sample_content)

        self.assertEqual(result.translated_content, "__FINAL_TRANSLATION__")
        self.assertEqual(result.original_content, self.sample_content)
        self.assertIn("ROS", result.preserved_terms)
        self.assertEqual(result.code_blocks_preserved, 1)

    @patch('tools.translate_to_urdu.OpenAI')
    def test_translate_api_error(self, mock_openai):
        """Test handling of API errors."""
        mock_openai.side_effect = Exception("API Error")

        translator = UrduTranslator(api_key="test_key")

        with self.assertRaises(RuntimeError):
            translator.translate_to_urdu(self.sample_content)

    def test_no_api_key(self):
        """Test initialization without API key."""
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(ValueError):
                UrduTranslator()


class TestIntegration(unittest.TestCase):
    """Integration tests for the complete translation workflow."""

    def test_full_translation_flow(self):
        """Test the complete translation workflow."""
        content = """---
title: "NVIDIA Isaac Tutorial"
---

# Isaac Simulation

This tutorial covers NVIDIA Isaac simulation.

<Callout type="info">
Use Isaac Sim for robot simulation.
</Callout>

## Code

```python
from isaacgym import gymapi
gym = gymapi.acquire_gym()
```

Visit https://developer.nvidia.com/isaac-sim for details.

![Isaac Interface](./images/isaac-ui.png)"""

        # Mock the actual translation since we don't have a real API key
        with patch('tools.translate_to_urdu.UrduTranslator.translate_to_urdu') as mock_translate:
            mock_translate.return_value = type('MockResult', (), {
                'translated_content': '<div dir="rtl">---\ntitle: "NVIDIA Isaac Tutorial"\n---\n\n# Isaac Simulation\n\nیہ ٹیوٹوریل NVIDIA Isaac simulation کو �ھالتا ہے۔\n\n<Callout type="info">\nIsaac Sim استعمال کریں۔\n</Callout>\n\n## Code\n\n```python\nfrom isaacgym import gymapi\ngym = gymapi.acquire_gym()\n```\n\nhttps://developer.nvidia.com/isaac-sim ملاحظہ کریں۔\n\n![Isaac Interface](./images/isaac-ui.png)</div>',
                'original_content': content,
                'preserved_terms': ["ROS", "NVIDIA", "Isaac"],
                'code_blocks_preserved': 1
            })()

            result = translate_to_urdu(content, ["NVIDIA", "Isaac"], api_key="test")

            self.assertTrue(result["success"])
            self.assertIn("dir=\"rtl\"", result["translated_content"])
            self.assertIn("NVIDIA", result["translated_content"])
            self.assertIn("```python", result["translated_content"])


if __name__ == '__main__':
    # Create a test suite
    suite = unittest.TestSuite()

    # Add test cases
    suite.addTest(unittest.makeSuite(TestMDXPreserver))
    suite.addTest(unittest.makeSuite(TestMDXProcessor))
    suite.addTest(unittest.makeSuite(TestImageProcessor))
    suite.addTest(unittest.makeSuite(TestUrduTranslator))
    suite.addTest(unittest.makeSuite(TestIntegration))

    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)