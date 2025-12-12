# Research Findings: MDX Textbook Chapters

**Feature**: 002-mdx-textbook-chapters
**Date**: 2025-12-12
**Status**: Complete

## 1. MDX Frontmatter Schema for Educational Content

### Decision
Use YAML frontmatter with strict typed schema including: title, learning_objectives (array), tags (array), difficulty_level (enum), ros_version, prerequisites (array), estimated_time (minutes), personalization_variants (object).

### Rationale
- Structured metadata enables automatic content indexing for RAG chatbot
- Type validation catches authoring errors early
- Consistent schema across 10 chapters ensures curriculum coherence
- personalization_variants flags enable PersonalizationEngine to know which chapters support adaptive content

### Alternatives Considered
| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| YAML frontmatter | Human readable, Docusaurus native, good tooling | Requires manual validation | **Selected** |
| JSON frontmatter | Strict typing | Less readable, awkward in MDX | Rejected |
| No frontmatter | Simpler authoring | Loses all indexing/personalization capability | Rejected |
| Separate metadata files | Clean separation | Maintenance burden, sync issues | Rejected |

### Implementation Notes
- Use Zod schema for frontmatter validation in CI
- Schema defined in `contracts/frontmatter-schema.yaml`
- Docusaurus already parses YAML frontmatter automatically

---

## 2. ROS 2 Code Example Patterns

### Decision
Use rclpy (Python) and rclcpp (C++) with standard ROS 2 message types. All examples target ROS 2 Humble LTS. Mark testable examples with `# test: true` comment in first line of code block.

### Rationale
- ROS 2 Humble is current LTS (support until 2027)
- Standard packages (std_msgs, geometry_msgs, sensor_msgs) available in ros:humble Docker image
- Python preferred for beginners, C++ for performance-critical examples
- Test marker allows CI to identify executable examples

### Alternatives Considered
| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| Standard packages only | CI-friendly, no custom builds | Limited to common scenarios | **Selected** |
| Custom ROS packages | Realistic project structure | Complex CI, learner setup burden | Rejected (out of scope) |
| ROS 1 examples | Legacy support | Outdated, not our target platform | Rejected |
| Simulation-only | No hardware dependency | Limits learning for hardware users | Partial (offer both variants) |

### Standard Packages Used
```yaml
python:
  - rclpy
  - std_msgs
  - geometry_msgs
  - sensor_msgs
  - nav_msgs
  - tf2_ros

cpp:
  - rclcpp
  - std_msgs
  - geometry_msgs
  - sensor_msgs
  - nav_msgs
  - tf2_ros
```

### Code Block Conventions
```python
# test: true
# ROS 2 Humble | Python 3.10+
# Dependencies: rclpy, std_msgs
import rclpy
...
```

---

## 3. Content Personalization Approach

### Decision
Implement section-level personalization via `<PersonalizedSection>` React component. The component accepts props: level (beginner|intermediate|advanced), rosFamiliarity (novice|intermediate|expert), hardwareAccess (boolean).

### Rationale
- Single MDX file contains all variants (no content duplication)
- React component handles conditional rendering at runtime
- Section-level granularity balances flexibility with authoring complexity
- User profile context provided via React Context (from Phase 4B)

### Alternatives Considered
| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| Section-level components | Single file, clean MDX | Requires React component | **Selected** |
| Separate chapter files | Simple MDX | 3x content duplication | Rejected |
| Inline conditionals | No component needed | Messy MDX, hard to read | Rejected |
| Build-time variants | Smaller bundle | Can't adapt at runtime | Rejected |

### Component API
```tsx
interface PersonalizedSectionProps {
  level: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity?: 'novice' | 'intermediate' | 'expert';
  hardwareAccess?: boolean;
  children: React.ReactNode;
}
```

### Usage in MDX
```mdx
<PersonalizedSection level="beginner">
  ## Understanding Nodes (Beginner)
  A node is like a small program that does one specific thing...
</PersonalizedSection>

<PersonalizedSection level="advanced">
  ## Node Architecture (Advanced)
  ROS 2 nodes implement the rclcpp::Node base class with lifecycle management...
</PersonalizedSection>
```

---

## 4. Vector Database Chunking Strategy

### Decision
Chunk chapters by h2 sections. Each chunk includes section content, parent chapter metadata, and tags. Use all-MiniLM-L6-v2 (384 dimensions) for embeddings. Store in Qdrant collection `robotics_textbook_chapters`.

### Rationale
- H2 sections are semantically coherent units (100-500 words typical)
- 384-dimension embeddings balance quality with storage efficiency
- all-MiniLM-L6-v2 is fast and free (sentence-transformers)
- Qdrant HNSW index provides sub-second search

### Alternatives Considered
| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| H2 section chunks | Semantic units, good context | May be long for some sections | **Selected** |
| Paragraph chunks | Fine-grained search | Loses context, too many chunks | Rejected |
| Full chapter chunks | Maximum context | Too broad for precise search | Rejected |
| Fixed token chunks | Consistent size | Breaks semantic boundaries | Rejected |

### Chunk Schema
```json
{
  "id": "ch01-sec-1-2-creating-publisher",
  "vector": [0.123, ...],  // 384 dimensions
  "payload": {
    "chapter_id": "ch01",
    "chapter_title": "ROS 2 Nodes & Lifecycle",
    "section_id": "sec-1-2",
    "section_title": "Creating Your First Publisher",
    "content": "...",
    "tags": ["ros2", "publisher", "rclpy"],
    "difficulty_level": "beginner",
    "ros_version": "humble"
  }
}
```

### Qdrant Configuration
```yaml
collection: robotics_textbook_chapters
vector_size: 384
distance: Cosine
hnsw_config:
  m: 16
  ef_construct: 100
payload_indexes:
  - chapter_id: keyword
  - tags: keyword
  - difficulty_level: keyword
```

---

## 5. Code Example Validation in CI

### Decision
GitHub Actions workflow triggered on PR to main. Extracts code blocks marked with `# test: true`. Runs each in ros:humble Docker container. PR blocked if any example fails.

### Rationale
- Automated validation prevents broken examples in production
- ros:humble Docker image matches target environment
- PR-gated ensures no broken code reaches main branch
- Per-file reporting helps authors identify issues

### Alternatives Considered
| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| PR-gated CI | Catches errors before merge | Longer PR cycle | **Selected** |
| Manual testing | No CI setup | Error-prone, doesn't scale | Rejected |
| Post-merge validation | Faster PR cycle | Broken examples reach production | Rejected |
| Local-only testing | No CI overhead | Author discipline required | Rejected |

### Workflow Specification
```yaml
name: Validate Code Examples
on:
  pull_request:
    branches: [main]
    paths: ['docs/**/*.mdx']

jobs:
  validate:
    runs-on: ubuntu-latest
    container:
      image: ros:humble-ros-base-jammy
    steps:
      - uses: actions/checkout@v4
      - name: Extract and validate code examples
        run: |
          python scripts/extract-code-examples.py docs/
          python scripts/validate-code-examples.py
```

### Code Extraction Script Logic
1. Parse MDX files for fenced code blocks (```python, ```cpp)
2. Filter for blocks containing `# test: true` marker
3. Write each block to temporary file
4. Execute Python blocks with `python3`
5. Compile and execute C++ blocks with `colcon build`
6. Report pass/fail status

---

## 6. Readability Measurement

### Decision
Use textstat Python library to calculate Flesch-Kincaid Grade Level. Target range: 12.0-14.0 (college-level technical writing). Run as CI check on PR.

### Rationale
- Automated measurement ensures consistent quality
- Flesch-Kincaid is standard readability metric
- 12-14 range appropriate for graduate-level technical content
- textstat is lightweight with no dependencies

### Alternatives Considered
| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| textstat (Flesch-Kincaid) | Standard metric, automated | English-centric | **Selected** |
| Manual review | Human judgment | Subjective, doesn't scale | Rejected (supplementary only) |
| No measurement | No overhead | Quality varies wildly | Rejected |
| Hemingway App | Nice UI | Not automatable in CI | Rejected |

### Implementation
```python
import textstat

def validate_readability(mdx_content: str) -> tuple[float, bool]:
    """Returns (score, passes_gate)"""
    # Strip code blocks and frontmatter before analysis
    prose = extract_prose(mdx_content)
    score = textstat.flesch_kincaid_grade(prose)
    passes = 12.0 <= score <= 14.0
    return score, passes
```

### CI Integration
- Run on all MDX files in PR
- Fail if any chapter outside 12.0-14.0 range
- Report per-chapter scores in PR comment

---

## 7. Technical Term Preservation for Translation

### Decision
Maintain a glossary of 50+ technical terms that must be preserved in English during Urdu translation. Terms marked with `<TechnicalTerm>` component in MDX. Glossary stored in `src/data/technical-terms.json`.

### Rationale
- Technical terms (ROS 2, URDF, Gazebo) have no Urdu equivalent
- Consistent treatment across all chapters
- Automated validation ensures no terms accidentally translated
- Component enables contextual Urdu explanation alongside English term

### Technical Terms List (Sample)
```json
{
  "terms": [
    {"term": "ROS 2", "category": "platform", "explanation_ur": "روبوٹ آپریٹنگ سسٹم"},
    {"term": "publisher", "category": "ros-concept", "explanation_ur": "ڈیٹا بھیجنے والا"},
    {"term": "subscriber", "category": "ros-concept", "explanation_ur": "ڈیٹا وصول کرنے والا"},
    {"term": "URDF", "category": "format", "explanation_ur": "روبوٹ ماڈل فارمیٹ"},
    {"term": "Isaac Sim", "category": "platform", "explanation_ur": "این ویڈیا سمولیشن پلیٹ فارم"},
    {"term": "inverse kinematics", "category": "robotics", "explanation_ur": "الٹی حرکیات"},
    {"term": "node", "category": "ros-concept", "explanation_ur": "نوڈ - ایک پروسیس"},
    {"term": "topic", "category": "ros-concept", "explanation_ur": "ٹاپک - ڈیٹا چینل"}
  ]
}
```

### MDX Usage
```mdx
The <TechnicalTerm term="publisher" /> sends messages to a <TechnicalTerm term="topic" />.
```

---

## 8. Exercise Design Pattern

### Decision
Each chapter includes 2 beginner exercises and 2 intermediate exercises. Beginner exercises solvable with chapter content alone. Intermediate exercises require synthesis with prerequisites. All exercises include problem description, optional starter code, complete solution, expected output, and troubleshooting steps.

### Rationale
- Beginner exercises build confidence
- Intermediate exercises challenge and synthesize knowledge
- Starter code reduces friction for beginners
- Troubleshooting steps address common mistakes

### Exercise Template
```mdx
## Exercise 1: Create a Temperature Publisher (Beginner)

**Learning Objective**: Apply publisher concepts to sensor data

### Problem
Create a ROS 2 node that publishes simulated temperature readings every 2 seconds.

### Starter Code (Optional)
```python
# Start with this skeleton
import rclpy
from rclpy.node import Node
# TODO: Import appropriate message type
```

### Solution
```python
# test: true
# Complete solution
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
...
```

### Expected Output
```
[INFO] [temp_publisher]: Publishing: 23.5°C
[INFO] [temp_publisher]: Publishing: 24.1°C
```

### Troubleshooting
- **Import error**: Ensure `ros2` environment is sourced
- **No output**: Check that `rclpy.spin()` is called
```

---

## Summary of Decisions

| Area | Decision | Key Rationale |
|------|----------|---------------|
| Frontmatter | YAML with strict schema | Enables indexing + personalization |
| Code Examples | Standard ROS 2 packages, Python/C++ | CI-friendly, no custom builds |
| Personalization | Section-level React component | No content duplication |
| Chunking | H2 sections + all-MiniLM-L6-v2 | Semantic units, efficient storage |
| CI Validation | PR-gated ros:humble Docker | Catches errors before merge |
| Readability | textstat Flesch-Kincaid 12-14 | Automated college-level check |
| Translation | Technical term glossary | Preserves English terms in Urdu |
| Exercises | 2 beginner + 2 intermediate per chapter | Progressive difficulty |

---

## Open Questions (Resolved)

All NEEDS CLARIFICATION items from Technical Context have been resolved:
- ✅ Language/Version: TypeScript (Docusaurus), Python 3.10+, C++ 17+ (ROS 2)
- ✅ Primary Dependencies: Confirmed via spec
- ✅ Storage: MDX files + Qdrant
- ✅ Testing: Jest + GitHub Actions + textstat
- ✅ Target Platform: GitHub Pages + Ubuntu 22.04 (ROS 2)
- ✅ Performance Goals: Quantified (FK 12-14, NDCG >0.8, 100% code pass)
- ✅ Constraints: Free tier limits documented
- ✅ Scale/Scope: 10 chapters, 30+ examples, 40+ exercises
