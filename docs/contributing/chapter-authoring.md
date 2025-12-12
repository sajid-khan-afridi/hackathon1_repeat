# Chapter Authoring Guide

This guide provides comprehensive instructions for authoring new chapters for the Physical AI & Humanoid Robotics textbook.

## Chapter Structure

### Frontmatter (Required)

Every chapter must include complete frontmatter with the following structure:

```yaml
---
title: "Chapter X: Chapter Title"
sidebar_position: X
description: "Brief description of chapter content (1-2 sentences)"
keywords:
  - keyword1
  - keyword2
  - keyword3
tags:
  - topic1
  - topic2
learning_objectives:
  - "Specific learning objective 1"
  - "Specific learning objective 2"
  - "Specific learning objective 3"
difficulty_level: beginner|intermediate|advanced
prerequisites:
  - "Prerequisite 1"
  - "Prerequisite 2"
estimated_time: XX  # in minutes
personalization_variants:
  experience_level: true|false
  ros_familiarity: true|false
  hardware_access: true|false
---
```

### Content Sections

1. **Introduction**
   - Brief overview of chapter content
   - Use `<PersonalizedSection>` components for different experience levels

2. **Numbered Sections (1.1, 1.2, etc.)**
   - Main content with detailed explanations
   - Include code examples with `# test: true` marker
   - Use `<TechnicalTerm>` for important terminology

3. **Exercises**
   - Minimum 4 exercises per chapter
   - 2 beginner and 2 intermediate exercises
   - Include complete solutions

4. **Summary**
   - Key takeaways
   - Transition to next chapter

## Code Examples

### Requirements

- All executable code must be marked with `# test: true` comment
- Specify ROS 2 version and dependencies
- Include comprehensive inline comments
- Use clear, descriptive variable names

### Format

```python
# test: true
# ROS 2 Humble | Python 3.10
# Dependencies: rclpy, numpy
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    """Brief description of class purpose"""

    def __init__(self):
        super().__init__('example_node')
        # initialization code

    def method_name(self):
        """Method description"""
        # implementation
        pass
```

### Best Practices

1. **Keep examples focused** on demonstrating specific concepts
2. **Include error handling** where appropriate
3. **Add logging statements** for debugging
4. **Use type hints** for better code clarity
5. **Validate inputs** and handle edge cases

## Personalization

### PersonalizedSection Component

Use for content that should adapt based on user experience:

```mdx
<PersonalizedSection level="beginner" rosFamiliarity="novice">
Content for beginners who are new to ROS
</PersonalizedSection>

<PersonalizedSection level="advanced" rosFamiliarity="expert">
Content for advanced users with ROS experience
</PersonalizedSection>
```

### TechnicalTerm Component

Mark technical terms for translation preservation:

```mdx
<TechnicalTerm term="node">A node is a process that performs computation</TechnicalTerm>
```

## Exercises

### Exercise Structure

```mdx
### Exercise X.X: Exercise Title (Difficulty)

**Learning Objective**: What the user will learn

#### Problem
Clear description of the exercise task

#### Starter Code (if applicable)
```python
# Partial implementation for user to complete
```

#### Solution
```python
# test: true
# Complete solution with test marker
```

#### Expected Output (if applicable)
```
Example output the user should see
```
```

### Exercise Guidelines

1. **Progressive difficulty**: Each exercise builds on previous concepts
2. **Practical applications**: Use real-world robotics scenarios
3. **Clear instructions**: Provide step-by-step guidance
4. **Complete solutions**: Include fully working code
5. **Test your code**: Ensure all examples execute successfully

## Readability Requirements

- Target Flesch-Kincaid score: 12-14
- Use clear, concise language
- Define technical terms on first use
- Use active voice where possible
- Break complex concepts into digestible sections

## File Naming Convention

- Use format: `chapter-X-slug.mdx`
- Slug should be lowercase with hyphens
- Example: `chapter-1-nodes-lifecycle.mdx`

## Review Checklist

Before submitting a chapter:

- [ ] Complete frontmatter with all required fields
- [ ] Minimum 3 executable code examples
- [ ] All code examples marked with `# test: true`
- [ ] Minimum 4 exercises (2 beginner, 2 intermediate)
- [ ] All exercises include complete solutions
- [ ] Technical terms marked with `<TechnicalTerm>`
- [ ] Personalized sections included where applicable
- [ ] Flesch-Kincaid score between 12-14
- [ ] Consistent formatting and structure
- [ ] Working code that executes without errors
- [ ] Clear explanations and examples

## Testing

### Local Testing

1. Run the development server: `npm run dev`
2. Navigate to your chapter
3. Verify all code examples render correctly
4. Test personalization features
5. Check exercises display properly

### CI Testing

- All code examples are automatically validated via GitHub Actions
- Readability scores are checked on push/PR
- Content is indexed for search functionality

## Submitting Changes

1. Create a feature branch from `main`
2. Make your changes in the appropriate module directory
3. Test locally following the checklist above
4. Commit changes with descriptive messages
5. Create a pull request with:
   - Clear description of changes
   - Testing performed
   - Any special considerations

## Getting Help

- Check existing chapters for examples
- Review the style guide
- Ask questions in GitHub discussions
- Refer to ROS 2 and Isaac Sim documentation

Thank you for contributing to the robotics textbook!