# MDX Writer Skill

## Overview

The MDX Writer skill creates high-quality MDX files for technical documentation, specifically designed for Physical AI & Humanoid Robotics textbook chapters.

## Features

- ✅ Automatic frontmatter generation with proper metadata
- ✅ Learning objectives formatted as checklists
- ✅ Support for admonitions (tips, warnings, important notes)
- ✅ Code syntax highlighting for ROS 2 and Isaac Sim
- ✅ Exercise generation based on module content
- ✅ Personalization button integration
- ✅ Sidebar position calculation

## Usage

```bash
/skill mdx-writer
```

### Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `chapter_title` | string | ✅ | Title of the chapter |
| `module_number` | integer | ✅ | Module number (1-4) |
| `learning_objectives` | array | ✅ | Array of learning objectives |
| `content_outline` | string | ✅ | Structured chapter content |
| `code_examples` | boolean | ❌ | Include code examples (default: true) |
| `chapter_id` | string | ❌ | Unique chapter identifier |
| `sidebar_position` | integer | ❌ | Navigation position |

### Example Input

```json
{
  "chapter_title": "Introduction to Kinematics and Dynamics",
  "module_number": 1,
  "learning_objectives": [
    "Understand forward and inverse kinematics",
    "Apply dynamic equations"
  ],
  "content_outline": "## Basic Kinematics\n\nForward kinematics determines..."
}
```

## Output Structure

The skill generates MDX files with the following structure:

```mdx
---
sidebar_position: {calculated}
title: {chapter_title}
description: {auto-generated}
chapter_id: {auto-generated}
module: {module_number}
---

# {chapter_title}

<PersonalizeButton chapterId="{id}" />

## Learning Objectives
- [ ] Objective 1
- [ ] Objective 2

## Introduction
{auto-generated intro}

{processed content with admonitions}

## Summary
{key takeaways}

## Exercises
{module-specific exercises}
```

## Supported Admonitions

The skill automatically detects and formats:

- `:::tip[Tip]` - For best practices
- `:::warning[Warning]` - For common pitfalls
- `:::note[Note]` - For additional information
- `:::important[Important]` - For critical concepts

## Code Highlighting

Supports syntax highlighting for:
- ROS 2 (Python, C++, YAML, Launch XML)
- Isaac Sim (Python, USD, JSON)
- Robotics formats (URDF, SDF)

## Module-Based Exercises

Each module includes tailored exercises:
- **Module 1**: Kinematics, dynamics, basic ROS 2
- **Module 2**: State estimation, control, simulation
- **Module 3**: Perception, planning, decision-making
- **Module 4**: System integration, safety, HRI