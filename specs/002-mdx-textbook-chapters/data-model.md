# Data Model: MDX Textbook Chapters

**Feature**: 002-mdx-textbook-chapters
**Date**: 2025-12-12
**Status**: Complete

## Entity Relationship Diagram

```
┌─────────────────┐       ┌─────────────────┐
│     Chapter     │───────│     Section     │
│                 │ 1   * │                 │
├─────────────────┤       ├─────────────────┤
│ id              │       │ id              │
│ module          │       │ heading         │
│ chapterNumber   │       │ level           │
│ title           │       │ content         │
│ learningObj[]   │       └────────┬────────┘
│ tags[]          │                │
│ difficultyLevel │                │ 0..*
│ rosVersion      │       ┌────────▼────────┐
│ prerequisites[] │       │ Personalized    │
│ estimatedTime   │       │ Variant         │
│ personalization │       ├─────────────────┤
└────────┬────────┘       │ variantType     │
         │                │ targetValue     │
         │                │ replacement     │
         │ 1   *          └─────────────────┘
┌────────▼────────┐
│   CodeExample   │       ┌─────────────────┐
├─────────────────┤       │  TechnicalTerm  │
│ id              │       ├─────────────────┤
│ chapterId       │       │ term            │
│ language        │       │ translationStat │
│ code            │       │ contextExplan   │
│ rosVersion      │       └─────────────────┘
│ requiredPkgs[]  │
│ environment     │       ┌─────────────────┐
│ expectedOutput  │       │   SearchQuery   │
│ validated       │       ├─────────────────┤
│ testable        │       │ id              │
└─────────────────┘       │ query           │
                          │ type            │
┌─────────────────┐       │ targetChapter   │
│    Exercise     │       │ relevanceGrades │
├─────────────────┤       └─────────────────┘
│ id              │
│ chapterId       │       ┌─────────────────┐
│ difficultyLevel │       │  QdrantChunk    │
│ learningObj     │       ├─────────────────┤
│ problemDesc     │       │ id              │
│ starterCode     │       │ vector[384]     │
│ solution        │       │ payload         │
│ expectedResult  │       └─────────────────┘
│ troubleshoot[]  │
└─────────────────┘
```

## Entity Definitions

### Chapter

The primary content unit representing one educational module.

```typescript
interface Chapter {
  /**
   * Unique identifier following pattern: ch{NN}-{slug}
   * @example "ch01-ros2-nodes-lifecycle"
   */
  id: string;

  /**
   * Module grouping for curriculum organization
   */
  module: 'ros2-fundamentals' | 'isaac-sim' | 'applications';

  /**
   * Sequential number within overall curriculum (1-10)
   */
  chapterNumber: number;

  /**
   * Human-readable chapter title
   * @example "ROS 2 Nodes & Lifecycle"
   */
  title: string;

  /**
   * Specific, measurable learning outcomes
   * Minimum 3 items required per spec FR-002
   * @example ["Create ROS 2 nodes using rclpy", "Understand node lifecycle states"]
   */
  learningObjectives: string[];

  /**
   * Tags for content indexing and search
   * Used by Qdrant payload filtering
   */
  tags: string[];

  /**
   * Overall difficulty level of chapter content
   */
  difficultyLevel: 'beginner' | 'intermediate' | 'advanced';

  /**
   * Target ROS 2 distribution
   * @default "humble"
   */
  rosVersion: string;

  /**
   * Array of chapter IDs that should be completed before this chapter
   * @example ["ch01", "ch02"]
   */
  prerequisites: string[];

  /**
   * Estimated reading + practice time in minutes
   * Target: 20-30 min reading + 30-45 min hands-on
   */
  estimatedTime: number;

  /**
   * Flags indicating which personalization dimensions have content variants
   */
  personalizationVariants: {
    experienceLevel: boolean;  // beginner/intermediate/advanced variants exist
    rosFamiliarity: boolean;   // novice/intermediate/expert variants exist
    hardwareAccess: boolean;   // simulation-only vs hardware variants exist
  };

  /**
   * Ordered array of chapter sections
   */
  sections: Section[];

  /**
   * Practice exercises with solutions
   * Minimum 4 per chapter (2 beginner + 2 intermediate)
   */
  exercises: Exercise[];

  /**
   * Executable code examples
   * Minimum 3 per chapter per spec FR-003
   */
  codeExamples: CodeExample[];
}
```

### Section

A logical division within a chapter, corresponding to h2/h3/h4 headings.

```typescript
interface Section {
  /**
   * Unique identifier following pattern: sec-{chapter}-{number}
   * @example "sec-01-1-2"
   */
  id: string;

  /**
   * Section heading text
   * @example "1.2 Creating Your First Publisher"
   */
  heading: string;

  /**
   * Heading level (h2=2, h3=3, h4=4)
   * h2 sections are primary chunk boundaries for vector search
   */
  level: 2 | 3 | 4;

  /**
   * MDX content of the section
   * May contain code examples, images, and personalized variants
   */
  content: string;

  /**
   * Alternative content for different user profiles
   * Empty if no personalization for this section
   */
  personalizedVariants?: PersonalizedVariant[];
}
```

### PersonalizedVariant

Alternative content segment rendered based on user profile.

```typescript
interface PersonalizedVariant {
  /**
   * The profile dimension this variant targets
   */
  variantType: 'experience_level' | 'ros_familiarity' | 'hardware_access';

  /**
   * The specific profile value that triggers this variant
   * @example "beginner" for experience_level
   * @example "expert" for ros_familiarity
   * @example "true" for hardware_access
   */
  targetValue: string;

  /**
   * MDX content to render when user profile matches
   */
  replacementContent: string;
}
```

### CodeExample

An executable code snippet within a chapter.

```typescript
interface CodeExample {
  /**
   * Unique identifier following pattern: ex-{chapter}-{slug}
   * @example "ex-01-simple-publisher-py"
   */
  id: string;

  /**
   * Parent chapter reference
   */
  chapterId: string;

  /**
   * Programming language
   */
  language: 'python' | 'cpp';

  /**
   * Complete executable code
   * First line should contain metadata comments
   */
  code: string;

  /**
   * Target ROS 2 version for this example
   */
  rosVersion: string;

  /**
   * List of ROS 2 packages required to run this example
   * @example ["rclpy", "std_msgs"]
   */
  requiredPackages: string[];

  /**
   * Target execution environment
   * 'both' means code works in simulation and on hardware
   */
  environment: 'simulation' | 'hardware' | 'both';

  /**
   * Expected console output or behavior description
   * Used for learner verification
   */
  expectedOutput?: string;

  /**
   * Whether this example has passed CI validation
   * Set by GitHub Actions workflow
   */
  validated: boolean;

  /**
   * Whether this example should be included in CI testing
   * True if code contains '# test: true' marker
   */
  testable: boolean;
}
```

### Exercise

A practice problem with solution for learner assessment.

```typescript
interface Exercise {
  /**
   * Unique identifier following pattern: ex{NN}-ch{NN}-{slug}
   * @example "ex01-ch01-pub-sensor"
   */
  id: string;

  /**
   * Parent chapter reference
   */
  chapterId: string;

  /**
   * Difficulty classification
   * - beginner: Solvable with chapter content alone
   * - intermediate: Requires synthesis with prerequisites
   */
  difficultyLevel: 'beginner' | 'intermediate';

  /**
   * The specific learning objective this exercise targets
   * Should map to one of the chapter's learningObjectives
   */
  learningObjective: string;

  /**
   * Clear description of what the learner should build/solve
   */
  problemDescription: string;

  /**
   * Optional skeleton code to reduce boilerplate
   * Helps beginners get started faster
   */
  starterCode?: string;

  /**
   * Complete, working solution code
   * Must be executable and pass validation
   */
  solution: string;

  /**
   * Expected output or behavior when solution is run correctly
   */
  expectedResult: string;

  /**
   * Common mistakes and how to fix them
   * @example ["Import error: Ensure ros2 environment is sourced"]
   */
  troubleshootingSteps: string[];
}
```

### TechnicalTerm

Domain-specific vocabulary that must be preserved during translation.

```typescript
interface TechnicalTerm {
  /**
   * The English term that should not be translated
   * @example "ROS 2"
   * @example "inverse kinematics"
   */
  term: string;

  /**
   * How to handle this term during translation
   * - preserve: Keep exactly as-is (e.g., "ROS 2")
   * - transliterate: Phonetic representation in target script
   */
  translationStatus: 'preserve' | 'transliterate';

  /**
   * Explanation in Urdu to provide context
   * @example "روبوٹ آپریٹنگ سسٹم"
   */
  contextualExplanation?: string;

  /**
   * Category for grouping terms
   */
  category?: 'platform' | 'ros-concept' | 'format' | 'robotics' | 'math';
}
```

### SearchQuery

Test query for measuring NDCG@10 retrieval quality.

```typescript
interface SearchQuery {
  /**
   * Unique query identifier
   * @example "q01"
   */
  id: string;

  /**
   * The search query text
   * @example "how to create ROS 2 publisher"
   */
  query: string;

  /**
   * Query classification for analysis
   * - exact-match: Technical terminology queries
   * - conceptual: Understanding-based queries
   */
  type: 'exact-match' | 'conceptual';

  /**
   * The chapter that should rank highest for this query
   */
  targetChapter: string;

  /**
   * Relevance grades for all related chapters
   * Grading scale:
   * - 0: irrelevant - no useful information
   * - 1: marginal - tangentially related
   * - 2: relevant - contains useful information
   * - 3: highly relevant - directly answers query
   */
  relevanceGrades: Record<string, 0 | 1 | 2 | 3>;
}
```

### QdrantChunk

Vector database entry for semantic search.

```typescript
interface QdrantChunk {
  /**
   * Unique chunk identifier
   * @example "ch01-sec-1-2-creating-publisher"
   */
  id: string;

  /**
   * 384-dimensional embedding vector from all-MiniLM-L6-v2
   */
  vector: number[];

  /**
   * Searchable and filterable metadata
   */
  payload: {
    chapter_id: string;
    chapter_title: string;
    section_id: string;
    section_title: string;
    content: string;
    tags: string[];
    difficulty_level: 'beginner' | 'intermediate' | 'advanced';
    ros_version: string;
    module: 'ros2-fundamentals' | 'isaac-sim' | 'applications';
  };
}
```

## Validation Rules

### Chapter Validation
1. `id` must match pattern `/^ch[0-9]{2}-[a-z0-9-]+$/`
2. `learningObjectives` must have minimum 3 items
3. `chapterNumber` must be 1-10
4. `estimatedTime` must be 45-75 minutes
5. `exercises` must have minimum 4 items (2 beginner + 2 intermediate)
6. `codeExamples` must have minimum 3 items

### CodeExample Validation
1. Must include `# test: true` marker if `testable` is true
2. `requiredPackages` must only include standard ROS 2 packages
3. Python examples must be valid Python 3.10+ syntax
4. C++ examples must compile with C++17

### Exercise Validation
1. `solution` must be executable code
2. `troubleshootingSteps` must have minimum 2 items
3. `learningObjective` must reference a chapter learning objective

## State Transitions

### CodeExample Validation State
```
┌──────────┐      CI Run       ┌───────────┐
│ Authored │ ─────────────────► │ Validated │
│ (false)  │                   │  (true)   │
└──────────┘                   └───────────┘
      │                              │
      │        PR Rejected           │
      ◄──────────────────────────────┘
           (validation failed)
```

### Content Indexing State
```
┌──────────┐    Build    ┌───────────┐   Index    ┌─────────┐
│   MDX    │ ──────────► │  Chunked  │ ─────────► │ Qdrant  │
│  Files   │             │  Content  │            │ Vector  │
└──────────┘             └───────────┘            └─────────┘
```

## Database Schema (Qdrant)

### Collection: robotics_textbook_chapters

```json
{
  "collection_name": "robotics_textbook_chapters",
  "vectors": {
    "size": 384,
    "distance": "Cosine"
  },
  "hnsw_config": {
    "m": 16,
    "ef_construct": 100
  },
  "payload_schema": {
    "chapter_id": "keyword",
    "tags": "keyword",
    "difficulty_level": "keyword",
    "module": "keyword"
  }
}
```

### Sample Document

```json
{
  "id": "ch01-sec-1-2-creating-publisher",
  "vector": [0.123, -0.456, ...],
  "payload": {
    "chapter_id": "ch01",
    "chapter_title": "ROS 2 Nodes & Lifecycle",
    "section_id": "sec-01-1-2",
    "section_title": "1.2 Creating Your First Publisher",
    "content": "Let's create a simple publisher that sends a \"Hello World\" message every second...",
    "tags": ["ros2", "publisher", "rclpy", "node"],
    "difficulty_level": "beginner",
    "ros_version": "humble",
    "module": "ros2-fundamentals"
  }
}
```

## Relationships

| From | To | Cardinality | Description |
|------|-----|-------------|-------------|
| Chapter | Section | 1:N | A chapter contains multiple sections |
| Chapter | Exercise | 1:N | A chapter has 4+ exercises |
| Chapter | CodeExample | 1:N | A chapter has 3+ code examples |
| Section | PersonalizedVariant | 1:N | A section may have personalized content |
| Chapter | Chapter | N:M | Prerequisites relationship |
| QdrantChunk | Chapter | N:1 | Multiple chunks per chapter (one per h2 section) |

## Indexes

### MDX File System
- Files organized by module: `docs/module-{N}-{name}/chapter-{N}-{slug}.mdx`

### Qdrant Vector Database
- `chapter_id`: Keyword index for filtering by chapter
- `tags`: Keyword index for tag-based filtering
- `difficulty_level`: Keyword index for difficulty filtering
- `module`: Keyword index for module filtering

## Migration Strategy

### From Sample Content to Full Curriculum
1. Existing sample chapters (ch1-publishers, ch4-introduction) serve as templates
2. New chapters follow same frontmatter schema
3. Content indexed into fresh Qdrant collection
4. Test queries validate search quality before launch
