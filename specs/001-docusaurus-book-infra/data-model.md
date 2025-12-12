# Phase 1 Data Model: Docusaurus Book Infrastructure

**Feature**: 002-docusaurus-book-infra
**Date**: 2025-12-11
**Purpose**: Define entities, fields, relationships, and validation rules for Phase 1

---

## Entity Overview

Phase 1 is a **static site** with **file-based content**. No database required. All entities are represented as file system structures and TypeScript interfaces.

---

## Entity 1: Chapter

**Description**: Individual lesson unit covering a specific robotics topic (e.g., "ROS 2 Publishers", "Inverse Kinematics")

**File Representation**: MDX file in `docs/` directory

**Example Path**: `docs/module-1-ros2-fundamentals/chapter-1-publishers.mdx`

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `title` | string | Yes | 1-100 characters | Chapter display title (e.g., "ROS 2 Publishers and Subscribers") |
| `sidebar_position` | number | Yes | Positive integer | Order within module (1-based indexing) |
| `slug` | string | No | URL-safe | Custom URL path (auto-generated from filename if omitted) |
| `description` | string | No | Max 160 characters | SEO meta description |
| `keywords` | string[] | No | Array of strings | SEO keywords (e.g., ["ros2", "publisher", "tutorial"]) |
| `tags` | string[] | No | Array of strings | Content categorization (e.g., ["beginner", "python"]) |
| `content` | MDX | Yes | Valid MDX syntax | Chapter body with markdown and JSX components |

### Validation Rules

1. **Title uniqueness**: No two chapters in same module can have identical titles
2. **Sidebar position**: No gaps in sequence (1, 2, 3, ...) within a module
3. **MDX syntax**: Must pass MDX parser (checked during build)
4. **Code blocks**: Must specify language for syntax highlighting
5. **Images**: Must use relative paths from `docs/` or absolute paths from `static/`

### Example File

```mdx
---
title: "ROS 2 Publishers and Subscribers"
sidebar_position: 1
description: "Learn how to create publishers and subscribers in ROS 2 using Python"
keywords: ["ros2", "publisher", "subscriber", "python", "tutorial"]
tags: ["beginner", "ros2", "python"]
---

# ROS 2 Publishers and Subscribers

In this chapter, you'll learn how to create ROS 2 publishers and subscribers.

## Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

## Key Concepts

- **Node**: Base class for ROS 2 components
- **Publisher**: Sends messages to a topic
- **Topic**: Named channel for communication

```

### State Transitions

N/A (static content, no state management)

---

## Entity 2: Module

**Description**: Logical grouping of related chapters (e.g., "ROS 2 Fundamentals", "NVIDIA Isaac Sim")

**File Representation**: Directory in `docs/` with `_category_.json` metadata file

**Example Path**: `docs/module-1-ros2-fundamentals/_category_.json`

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `label` | string | Yes | 1-50 characters | Module display name in sidebar (e.g., "ROS 2 Fundamentals") |
| `position` | number | Yes | Positive integer | Order in sidebar (1-based indexing) |
| `collapsed` | boolean | No | true/false | Whether module is collapsed by default (default: true) |
| `collapsible` | boolean | No | true/false | Whether module can be collapsed (default: true) |
| `link` | object | No | Valid Docusaurus link | Optional landing page for module |

### Validation Rules

1. **Label uniqueness**: No two modules can have same label
2. **Position uniqueness**: No two modules can have same position
3. **Directory naming**: Must match slug pattern (lowercase, hyphens)

### Example File

```json
{
  "label": "ROS 2 Fundamentals",
  "position": 1,
  "collapsible": true,
  "collapsed": false,
  "link": {
    "type": "generated-index",
    "description": "Learn the fundamentals of ROS 2 - publishers, subscribers, services, and more."
  }
}
```

### Relationships

- **1 Module : N Chapters** (one module contains multiple chapters)
- Chapters inherit module's position for sidebar ordering

---

## Entity 3: Theme Preference

**Description**: User's selected color scheme (light, dark, or system default)

**Storage**: Browser `localStorage`

**Key**: `theme` (managed by Docusaurus)

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `mode` | string | Yes | "light" \| "dark" | Selected theme mode |
| `respectSystemPreference` | boolean | No | true/false | Whether to follow system preference (default: false after manual toggle) |

### Validation Rules

1. **Mode values**: Must be "light" or "dark" (Docusaurus enforces)
2. **localStorage availability**: Fallback to system preference if localStorage unavailable (private browsing)

### State Transitions

```
Initial Visit → System Preference Detected (prefers-color-scheme)
  ↓
User Toggles → Manual Choice Stored in localStorage
  ↓
Subsequent Visits → Manual Choice Restored (overrides system preference)
```

### Storage Schema

```typescript
// localStorage key: 'theme'
interface ThemePreference {
  mode: 'light' | 'dark';
  respectSystemPreference: boolean;
}

// Example:
localStorage.setItem('theme', JSON.stringify({ mode: 'dark', respectSystemPreference: false }));
```

---

## Entity 4: Navigation Structure

**Description**: Two-level hierarchical organization auto-generated from `docs/` folder structure

**File Representation**: Auto-generated by Docusaurus from directory structure + `sidebars.ts` config

**Example Path**: `sidebars.ts` (optional customization)

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `modules` | Module[] | Yes | Non-empty array | Top-level sidebar items (expandable groups) |
| `chapters` | Chapter[] | Yes | Non-empty array per module | Nested items within each module |
| `homeDoc` | string | No | Valid doc ID | Default landing page (e.g., "intro") |

### Validation Rules

1. **Hierarchy depth**: Maximum 2 levels (modules → chapters)
2. **Auto-generation**: Docusaurus generates from `docs/` structure unless `sidebars.ts` overrides
3. **Ordering**: Respects `sidebar_position` from frontmatter

### Example Configuration

```typescript
// sidebars.ts (optional - can be auto-generated)
import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Fundamentals',
      items: [
        'module-1-ros2-fundamentals/chapter-1-publishers',
        'module-1-ros2-fundamentals/chapter-2-subscribers',
        'module-1-ros2-fundamentals/chapter-3-services',
      ],
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac Sim',
      items: [
        'module-2-isaac-sim/chapter-1-introduction',
        'module-2-isaac-sim/chapter-2-environments',
      ],
    },
  ],
};

export default sidebars;
```

### Auto-Generated Alternative

If `sidebars.ts` is omitted, Docusaurus auto-generates sidebar from:
- `_category_.json` files in each directory
- `sidebar_position` frontmatter in MDX files

**Recommendation**: Use auto-generation for Phase 1 (simpler, less maintenance)

---

## Entity 5: Code Example

**Description**: Syntax-highlighted code block within a chapter, tagged with programming language

**File Representation**: Fenced code block in MDX content

**Example**:

````mdx
```python
# ROS 2 publisher example
import rclpy
from rclpy.node import Node
```
````

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `language` | string | Yes | Supported by Prism.js | Programming language (e.g., "python", "cpp", "yaml") |
| `code` | string | Yes | Valid syntax for language | Code content |
| `title` | string | No | 1-50 characters | Optional code block title (shown above block) |
| `showLineNumbers` | boolean | No | true/false | Whether to show line numbers (default: false) |
| `highlightLines` | number[] | No | Array of line numbers | Lines to highlight (e.g., [3, 5-7]) |

### Validation Rules

1. **Language support**: Must be one of: `python`, `cpp`, `yaml`, `javascript`, `typescript`, `json`, `bash`, `xml`
2. **Syntax validity**: Not enforced (code examples may be incomplete for teaching purposes)
3. **Character limits**: No hard limit (use judgment for readability)

### Example with Metadata

````mdx
```python title="minimal_publisher.py" showLineNumbers {3,5-7}
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```
````

---

## Entity 6: Build Artifact

**Description**: Generated static site output (HTML, CSS, JS, images) ready for deployment to GitHub Pages

**File Representation**: `build/` directory (generated by `npm run build`)

**Example Path**: `build/index.html`, `build/assets/js/main.abc123.js`

### Fields

| Field | Type | Required | Validation | Description |
|-------|------|----------|------------|-------------|
| `html_files` | File[] | Yes | Valid HTML5 | Generated HTML pages for each chapter |
| `css_files` | File[] | Yes | Minified CSS | Stylesheet bundles (light/dark themes) |
| `js_files` | File[] | Yes | Minified JS | JavaScript bundles (per route, code split) |
| `assets` | File[] | Yes | Images, fonts | Static assets from `static/` directory |
| `metadata` | object | Yes | JSON | Build metadata (timestamp, version, Docusaurus config) |

### Validation Rules

1. **HTML validation**: Must pass W3C validator (best-effort)
2. **No broken links**: Internal links must resolve (checked by Docusaurus build)
3. **Asset paths**: All references must be relative or absolute from site root
4. **Minification**: Production builds must be minified (automatic via webpack)

### Build Process

```bash
# Development build (not deployed)
npm run start  # Live preview with hot reload

# Production build (deployed to GitHub Pages)
npm run build  # Generates build/ directory
npm run serve  # Preview production build locally
```

### Directory Structure

```text
build/
├── index.html                    # Home page
├── module-1-ros2-fundamentals/
│   ├── chapter-1-publishers/
│   │   └── index.html            # Chapter page
│   └── ...
├── assets/
│   ├── css/
│   │   └── styles.abc123.css     # Hashed CSS bundle
│   ├── js/
│   │   ├── main.abc123.js        # Main JS bundle
│   │   └── runtime~main.xyz789.js # Webpack runtime
│   └── images/
│       └── optimized-image.webp  # Optimized images
└── sitemap.xml                   # SEO sitemap
```

### State Transitions

```
Source MDX Files (docs/)
  ↓
Docusaurus Build Process (npm run build)
  ↓
Build Artifacts (build/)
  ↓
GitHub Actions CI/CD (.github/workflows/deploy.yml)
  ↓
GitHub Pages Deployment (live site)
```

---

## Relationships Summary

```
Module (1) ──────┐
                 │ contains
                 ↓
Chapter (N) ─────┐
                 │ includes
                 ↓
Code Example (N)
```

```
User ───────────┐
                │ sets
                ↓
Theme Preference (1) ─────┐
                           │ persists in
                           ↓
                    Browser localStorage
```

```
docs/ Directory ─────┐
                     │ generates
                     ↓
Navigation Structure ─────┐
                           │ renders as
                           ↓
                    Sidebar UI Component
```

---

## Validation Strategy

### Build-Time Validation
- **MDX syntax**: Docusaurus parser checks all `.mdx` files
- **Broken links**: Docusaurus `onBrokenLinks: 'throw'` config
- **TypeScript**: `tsc --noEmit` checks all `.ts/.tsx` files
- **ESLint**: Checks code style and potential errors

### Runtime Validation
- **Theme persistence**: Manual testing (toggle dark mode, refresh page)
- **Responsive layout**: Playwright tests at 320px, 768px, 1024px viewports
- **Accessibility**: Lighthouse CI checks WCAG 2.1 AA compliance

### Post-Deployment Validation
- **Live site**: Manual smoke test (navigation, dark mode, mobile)
- **Lighthouse**: Performance, Accessibility, Best Practices scores
- **Analytics**: Monitor build time and deployment success rate

---

## Next Steps

**Phase 1 Remaining Tasks**:
1. ✅ Data model defined
2. ⏳ Generate API contracts (N/A for static site - skip)
3. ⏳ Generate quickstart.md (developer setup guide)
4. ⏳ Update agent context
5. ⏳ Re-evaluate Constitution Check
