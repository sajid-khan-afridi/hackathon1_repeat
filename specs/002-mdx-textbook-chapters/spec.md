# Feature Specification: MDX Textbook Chapters for Physical AI & Humanoid Robotics

**Feature Branch**: `002-mdx-textbook-chapters`
**Created**: 2025-12-12
**Status**: Draft
**Input**: User description: "Phase 2 Content Creation - Create 10 MDX chapters for Physical AI & Humanoid Robotics textbook covering ROS 2, NVIDIA Isaac Sim, and robotics fundamentals. Chapters must include: (1) proper MDX frontmatter with learning objectives, tags, and metadata for content-indexer skill, (2) executable Python/C++ code examples tested in ROS 2 Humble, (3) exercises with solutions at beginner/intermediate levels, (4) chapter structure optimized for content-adapter skill personalization (experience_level, ros_familiarity, hardware_access variants), (5) technical terminology preserved for urdu-translator skill compatibility. Agent orchestration: content-publisher-agent creates chapters using mdx-writer skill, then indexes via content-indexer skill into Qdrant. Personalization-agent validates content-adapter compatibility. RAG-chatbot-agent tests searchability via rag-pipeline skill. Quality gates: code examples execute 100%, Flesch-Kincaid 12-14, NDCG@10 > 0.8 on test queries after indexing."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Educational Content Discovery and Learning (Priority: P1)

Students and self-learners access the robotics textbook to learn ROS 2 and humanoid robotics concepts. They navigate chapters, read explanations, run code examples locally, and test their understanding through exercises.

**Why this priority**: Core educational value - without accessible, well-structured content with working examples, the entire textbook platform fails to deliver its primary purpose. This is the foundation upon which all other features depend.

**Independent Test**: Can be fully tested by a learner opening a chapter, reading content, copying code examples into ROS 2 Humble environment, executing them successfully, and completing exercises. Delivers immediate learning value through comprehensible educational material.

**Acceptance Scenarios**:

1. **Given** a student visits the textbook platform, **When** they navigate to a chapter on ROS 2 publishers, **Then** they see structured content with clear learning objectives, explanations, and code examples
2. **Given** a student is reading a chapter, **When** they copy a Python code example for creating a ROS 2 node, **Then** the code executes without errors in ROS 2 Humble environment
3. **Given** a student completes reading a chapter, **When** they attempt the beginner-level exercises, **Then** they can verify their solutions against provided answers
4. **Given** a student searches for "inverse kinematics" in the textbook, **When** search results are returned, **Then** they find relevant chapters with highlighted content sections

---

### User Story 2 - Personalized Learning Experience (Priority: P2)

Learners with different backgrounds (complete beginners vs experienced programmers, ROS experts vs ROS newcomers, those with hardware vs simulation-only) receive content adapted to their experience level and available resources.

**Why this priority**: Enhances learning effectiveness by tailoring explanations and exercises to individual needs. Not essential for MVP but significantly improves user satisfaction and learning outcomes.

**Independent Test**: Can be tested by configuring user profiles with different experience levels (beginner/intermediate), ROS familiarity (novice/advanced), and hardware access (yes/no), then verifying that chapter content adapts appropriately (e.g., more detailed explanations for beginners, hardware-specific vs simulation-only code examples).

**Acceptance Scenarios**:

1. **Given** a user profile indicates beginner experience level, **When** they view a chapter on robot transforms, **Then** they see additional foundational explanations and simplified code examples
2. **Given** a user has no hardware access, **When** they view robot control examples, **Then** code samples use NVIDIA Isaac Sim simulation instead of physical robot APIs
3. **Given** a user is experienced with ROS 1, **When** they view ROS 2 migration content, **Then** the chapter emphasizes differences and migration patterns rather than basic ROS concepts

---

### User Story 3 - Multilingual Technical Content Access (Priority: P3)

Urdu-speaking students access technical robotics content in their native language while preserving critical technical terminology in English for accuracy and industry alignment.

**Why this priority**: Expands accessibility to non-English speakers, particularly in Pakistan and South Asia. Valuable for inclusivity but not required for core functionality. Can be added after English content is validated.

**Independent Test**: Can be tested by switching language preference to Urdu, viewing a chapter, and verifying that explanatory text is translated while technical terms (ROS 2, publisher, subscriber, Isaac Sim) remain in English with Urdu contextual explanations.

**Acceptance Scenarios**:

1. **Given** a user selects Urdu as their language preference, **When** they open a chapter on ROS 2 nodes, **Then** descriptive text appears in Urdu while code and technical terms remain in English
2. **Given** a chapter contains the term "differential drive kinematics", **When** displayed in Urdu mode, **Then** the term remains in English with an Urdu explanation of the concept
3. **Given** a code example with comments, **When** viewed in Urdu mode, **Then** comments are translated while variable names and code syntax remain unchanged

---

### User Story 4 - Content Quality Assurance and Validation (Priority: P1)

Content creators and educators validate that published chapters meet educational quality standards: code examples execute correctly, readability is appropriate for target audience, and content is discoverable through search.

**Why this priority**: Essential for maintaining educational integrity and platform credibility. Without quality validation, learners may encounter broken code or incomprehensible content, damaging trust and learning outcomes.

**Independent Test**: Can be tested by running automated quality gates on a completed chapter: execute all code examples in ROS 2 Humble and verify 100% success rate, measure Flesch-Kincaid readability score (target: 12-14), perform test searches and measure NDCG@10 (target: >0.8).

**Acceptance Scenarios**:

1. **Given** a chapter contains 5 Python code examples, **When** quality validation runs, **Then** all 5 examples execute successfully in a fresh ROS 2 Humble environment
2. **Given** a chapter is submitted for publication, **When** readability analysis runs, **Then** the Flesch-Kincaid grade level falls between 12-14 (college-level, technical but accessible)
3. **Given** a chapter covers "robot URDF models", **When** test queries like "how to define robot links" are executed, **Then** this chapter appears in top results with NDCG@10 score above 0.8
4. **Given** a chapter includes C++ examples, **When** quality validation runs, **Then** all examples compile and execute without warnings or errors

---

### Edge Cases

- What happens when a code example requires external dependencies (e.g., specific ROS 2 packages, NVIDIA Isaac Sim installation)? The chapter must specify prerequisites clearly in frontmatter and provide setup instructions.
- How does the system handle version mismatches (e.g., user has ROS 2 Foxy instead of Humble)? Chapters should document tested versions and note compatibility in metadata.
- What if a student's system configuration prevents code execution (e.g., GPU unavailable for Isaac Sim)? Chapters should offer alternative simulation/visualization approaches.
- How does personalization handle conflicting preferences (e.g., beginner + hardware access)? System prioritizes experience level and provides hardware examples with additional explanations.
- What happens when technical terminology has no equivalent Urdu translation? Terms remain in English with phonetic Urdu transcription and contextual explanation.
- How does search handle acronyms and abbreviations (ROS vs Robot Operating System)? Content indexer must map synonyms and expand acronyms in metadata.
- What if code examples require substantial computational resources? Chapters should warn users and provide lightweight alternatives or cloud-based options.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST generate 10 complete MDX chapter files covering ROS 2 fundamentals, NVIDIA Isaac Sim, and humanoid robotics concepts
- **FR-002**: Each chapter MUST include structured frontmatter with fields: title, learning_objectives (array), tags (array), difficulty_level, ros_version, prerequisites (array), estimated_time, and personalization_variants
- **FR-003**: Each chapter MUST contain at least 3 executable code examples in Python and/or C++ validated against ROS 2 Humble
- **FR-004**: Each chapter MUST provide exercises at beginner and intermediate difficulty levels with complete solution code
- **FR-005**: Code examples MUST execute successfully with 100% pass rate in a clean ROS 2 Humble environment
- **FR-006**: Chapter content MUST achieve Flesch-Kincaid reading grade level between 12-14 (college-level technical writing)
- **FR-007**: Chapters MUST include personalization markers for content-adapter skill to vary content based on experience_level (beginner/intermediate/advanced), ros_familiarity (novice/intermediate/expert), and hardware_access (true/false)
- **FR-008**: Technical terminology MUST be marked for preservation during translation (untranslatable terms for urdu-translator skill)
- **FR-009**: All chapter content MUST be indexed into Qdrant vector database with proper embeddings and metadata
- **FR-010**: Indexed content MUST achieve NDCG@10 score above 0.8 on predefined test queries covering each chapter's topics
- **FR-011**: System MUST preserve technical terms (ROS 2, Isaac Sim, URDF, Gazebo, etc.) during language translation
- **FR-012**: Each code example MUST include inline comments explaining key concepts and setup/teardown instructions
- **FR-013**: Chapters MUST specify hardware/software prerequisites clearly in frontmatter (ROS 2 Humble, Python 3.10+, NVIDIA GPU for Isaac Sim, etc.)
- **FR-014**: Exercise solutions MUST include expected output and common troubleshooting steps
- **FR-015**: Chapter structure MUST follow consistent sections: Introduction, Core Concepts, Code Examples, Practical Applications, Exercises, Summary

### Key Entities *(include if feature involves data)*

- **Chapter**: Represents a single educational module; key attributes include unique identifier, title, topic area (ROS 2 basics, Isaac Sim, kinematics, etc.), difficulty level, learning objectives, prerequisite chapters, estimated completion time
- **Code Example**: Executable code snippet within a chapter; attributes include language (Python/C++), ROS 2 version compatibility, required packages, execution environment (simulation/hardware), expected output, validation status
- **Exercise**: Practice problem with solution; attributes include difficulty level (beginner/intermediate), learning objective mapping, problem description, starter code (optional), complete solution, expected results
- **Personalization Variant**: Alternative content segment for different user profiles; attributes include variant type (experience_level/ros_familiarity/hardware_access), target profile value, replacement content section
- **Technical Term**: Domain-specific vocabulary requiring preservation; attributes include term text, language (English), translation status (preserve/translate/transliterate), contextual explanation
- **Search Query**: User search input for content discovery; attributes include query text, target topics, expected relevant chapters, NDCG score
- **Quality Gate**: Validation checkpoint; attributes include gate type (code_execution/readability/searchability), pass/fail status, metrics (execution success rate, Flesch-Kincaid score, NDCG@10)

### MDX Frontmatter Schema (Strict)

```typescript
interface ChapterFrontmatter {
  title: string;                          // Required - Chapter title
  learning_objectives: string[];          // Required - Min 3 items
  tags: string[];                         // Required - Indexing tags
  difficulty_level: 'beginner' | 'intermediate' | 'advanced';  // Required
  ros_version: string;                    // Required - e.g., "humble"
  prerequisites: string[];                // Optional - Default []
  estimated_time: number;                 // Required - Minutes
  personalization_variants: {
    experience_level: boolean;            // Required - Has variants
    ros_familiarity: boolean;             // Required - Has variants
    hardware_access: boolean;             // Required - Has variants
  };
}
```

### PersonalizedSection Component API

```typescript
interface PersonalizedSectionProps {
  level: 'beginner' | 'intermediate' | 'advanced';  // Required
  rosFamiliarity?: 'novice' | 'intermediate' | 'expert';  // Optional
  hardwareAccess?: boolean;               // Optional
  children: React.ReactNode;              // Required - Content to render
}

// Usage example:
// <PersonalizedSection level="beginner" hardwareAccess={false}>
//   Simulation-only content for beginners...
// </PersonalizedSection>
```

### Test Query Set Specification

**File**: `tests/search-queries.json`

**Structure**:
```json
{
  "queries": [
    {
      "id": "q01",
      "query": "how to create ROS 2 publisher",
      "type": "exact-match",
      "target_chapter": "ch01",
      "relevance_grades": {
        "ch01": 3,
        "ch02": 2,
        "ch03": 1
      }
    },
    {
      "id": "q02",
      "query": "how do robots sense their environment",
      "type": "conceptual",
      "target_chapter": "ch06",
      "relevance_grades": {
        "ch06": 3,
        "ch09": 3,
        "ch04": 1
      }
    }
  ],
  "grading_scale": {
    "0": "irrelevant - no useful information",
    "1": "marginal - tangentially related",
    "2": "relevant - contains useful information",
    "3": "highly relevant - directly answers query"
  }
}
```

**Query Distribution** (2 per chapter):
- Ch1-3 (ROS 2): 6 queries covering nodes, topics, services, parameters, launch files
- Ch4-6 (Isaac Sim): 6 queries covering setup, URDF, sensors, physics
- Ch7-10 (Applications): 8 queries covering kinematics, navigation, perception, humanoid control

**Query Types**:
- **Exact-match**: Technical terminology queries (e.g., "ROS 2 lifecycle states")
- **Conceptual**: Understanding-based queries (e.g., "how robots plan paths around obstacles")

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 10 chapters are generated in valid MDX format with complete frontmatter including learning objectives, tags, difficulty level, and personalization markers
- **SC-002**: 100% of code examples (minimum 30 examples across 10 chapters) execute successfully in ROS 2 Humble environment without errors
- **SC-003**: All chapters achieve Flesch-Kincaid reading grade level between 12.0 and 14.0 (college-level technical readability)
- **SC-004**: Content retrieval achieves NDCG@10 score greater than 0.8 for a test set of at least 20 domain-specific search queries
- **SC-005**: Each chapter includes minimum 2 exercises per difficulty level (beginner and intermediate) with complete, validated solutions
- **SC-006**: Personalized content variants exist for at least 3 user profile combinations per chapter (e.g., beginner+no-hardware, intermediate+hardware, advanced+simulation)
- **SC-007**: Technical terminology preservation is validated for at least 50 critical terms across all chapters
- **SC-008**: Chapter navigation supports sequential learning with clearly documented prerequisites, enabling learners to complete the full curriculum path
- **SC-009**: Search and retrieval latency remains under 2 seconds for 95% of queries
- **SC-010**: Code examples include comprehensive inline comments covering at least 70% of non-trivial code lines

### Quality Validation

- All code examples must pass automated execution tests in isolated ROS 2 Humble environment
- Readability scores verified using standard Flesch-Kincaid calculator
- Search performance measured using NDCG@10 metric with predefined test query set
- Personalization variants validated by content-adapter skill for proper rendering
- Translation compatibility confirmed by urdu-translator skill for term preservation

## Assumptions

1. **ROS 2 Humble Environment**: Code examples assume ROS 2 Humble LTS distribution with standard packages (rclpy, rclcpp, geometry_msgs, sensor_msgs, tf2) installed
2. **NVIDIA Isaac Sim**: Simulation examples target Isaac Sim 2023.1.0 or later with Python API enabled; users expected to have NVIDIA GPU with compute capability 7.0+ for optimal performance
3. **Python Version**: All Python examples assume Python 3.10 or later (compatible with ROS 2 Humble requirements)
4. **Qdrant Vector Database**: Content indexing uses Qdrant with collection `robotics_textbook_chapters`, 384-dimensional vectors from `all-MiniLM-L6-v2` embedding model
5. **Development Platform**: Code examples developed and tested on Ubuntu 22.04 LTS (primary ROS 2 Humble support platform); Windows/macOS compatibility noted where different
6. **User Technical Background**: Learners expected to have basic programming knowledge (variables, loops, functions) and command-line familiarity; no prior robotics experience assumed
7. **Content Format**: MDX format chosen for compatibility with modern documentation frameworks (Docusaurus, Next.js MDX, etc.) supporting React component embedding
8. **Embedding Model**: Content indexing uses `sentence-transformers/all-MiniLM-L6-v2` model (384 dimensions, optimized for speed and quality balance)
9. **Test Query Set**: NDCG evaluation uses 20 curated queries (2 per chapter) stored in `tests/search-queries.json` with graded relevance (0=irrelevant, 1=marginal, 2=relevant, 3=highly relevant); includes both exact-match and conceptual query types
10. **Translation Scope**: Urdu translation targets explanatory prose only; code, commands, API names, and technical constants remain in English
11. **Hardware Access Variants**: "Hardware access" assumes availability of physical robot platforms compatible with ROS 2 (e.g., TurtleBot3, custom humanoid); simulation variants provide equivalent learning outcomes
12. **Exercise Difficulty Calibration**: "Beginner" exercises solvable with chapter content alone; "Intermediate" exercises require synthesis of current chapter + 1-2 prerequisite chapters
13. **Chapter Scope**: Each chapter targets 20-30 minutes reading time plus 30-45 minutes hands-on coding/exercises for typical learner
14. **Agent Orchestration**: content-publisher-agent, personalization-agent, and rag-chatbot-agent operate with sufficient computational resources and API access to external services (vector database, LLM for content generation if applicable)

## Clarifications

### Session 2025-12-12

- Q: What are the specific chapter titles and learning objectives for the 10 chapters? → A: Option A - ROS 2 Core (Ch1-3: Nodes & Lifecycle, Topics & Services, Parameters & Launch), Isaac Sim (Ch4-6: Environment Setup, Robot Models & URDF, Sensors & Physics), Applications (Ch7-10: Kinematics, Navigation & Path Planning, Perception & Sensor Fusion, Humanoid Control)
- Q: What is the exact MDX frontmatter schema and PersonalizedSection component API? → A: Option A - Strict Schema with typed fields (title: string required, learning_objectives: string[] required min 3, tags: string[] required, difficulty_level: enum required, ros_version: string required, prerequisites: string[] optional, estimated_time: number required, personalization_variants: object required). PersonalizedSection props: level, rosFamiliarity?, hardwareAccess?, children
- Q: What is the Qdrant collection configuration and embedding model? → A: Option A - Model: all-MiniLM-L6-v2 (384-dim), Distance: Cosine, Index: HNSW m=16 ef_construct=100, Collection: robotics_textbook_chapters with payload indexing on chapter_id, tags, difficulty_level
- Q: What are the GitHub Actions workflow specifics for ROS 2 code validation? → A: Option A - PR-Gated: Trigger on PR to main, ros:humble Docker image, extract code blocks via script parsing ```python/```cpp with `# test: true` marker, run each in isolated container, fail PR if any fails, report per-file results
- Q: How is the test query set defined for NDCG@10 measurement? → A: Option A - Structured Test Set: 20 queries (2 per chapter), relevance grades 0-3 (irrelevant/marginal/relevant/highly-relevant), stored in tests/search-queries.json, mix of exact-match and conceptual queries

## Resolved Decisions

The following clarifications were requested and resolved with user input:

1. **Chapter Topic Distribution**: **DECISION: Foundational sequence**
   - Structure: 3 chapters on ROS 2 basics → 3 chapters on NVIDIA Isaac Sim → 4 chapters on robotics applications combining both
   - Rationale: Creates clear learning progression from fundamentals to integration; sequential prerequisites support learner confidence building
   - Chapter breakdown:
     - **Chapter 1**: ROS 2 Nodes & Lifecycle - Learning objectives: Create ROS 2 nodes, understand node lifecycle states, implement lifecycle callbacks
     - **Chapter 2**: ROS 2 Topics & Services - Learning objectives: Publish/subscribe patterns, service client/server implementation, QoS configuration
     - **Chapter 3**: ROS 2 Parameters & Launch - Learning objectives: Parameter declaration and retrieval, launch file creation, multi-node orchestration
     - **Chapter 4**: Isaac Sim Environment Setup - Learning objectives: Install and configure Isaac Sim, navigate UI, connect to ROS 2 bridge
     - **Chapter 5**: Robot Models & URDF - Learning objectives: Create URDF models, import robots into Isaac Sim, configure joints and links
     - **Chapter 6**: Sensors & Physics - Learning objectives: Add sensors (camera, lidar, IMU), configure physics properties, collect sensor data via ROS 2
     - **Chapter 7**: Kinematics - Learning objectives: Forward/inverse kinematics, implement IK solvers, control robot arm positioning
     - **Chapter 8**: Navigation & Path Planning - Learning objectives: Nav2 stack integration, path planning algorithms, obstacle avoidance
     - **Chapter 9**: Perception & Sensor Fusion - Learning objectives: Process camera/lidar data, sensor fusion techniques, object detection integration
     - **Chapter 10**: Humanoid Control - Learning objectives: Bipedal locomotion basics, balance control, full-body motion planning

2. **Code Example Testing Infrastructure**: **DECISION: Containerized CI/CD**
   - Implementation: Docker container with ROS 2 Humble pre-installed + automated GitHub Actions pipeline
   - Rationale: Ensures consistent, reproducible testing across development environments; enables automated quality gates on commits/PRs
   - Requirements: Dockerfile defining ROS 2 Humble base image with required packages; GitHub Actions workflow executing all chapter code examples
   - **GitHub Actions Workflow Specification**:
     - **Trigger**: Pull requests targeting `main` branch
     - **Docker Image**: `ros:humble` (official ROS 2 image)
     - **Code Extraction**: Custom script parses MDX files for code blocks (```python, ```cpp) marked with `# test: true` comment
     - **Execution**: Each extracted code block runs in isolated container instance
     - **Failure Handling**: PR blocked if any code example fails; per-file results reported in PR comments
     - **Workflow File**: `.github/workflows/validate-code-examples.yml`

3. **Personalization Variant Granularity**: **DECISION: Section-level**
   - Implementation: Entire sections replaced based on user profile (e.g., beginner sees "Introduction to Transforms", advanced sees "Advanced Transform Trees")
   - Rationale: Easier content authoring, cleaner MDX structure, sufficient personalization granularity for educational content
   - MDX approach: Use custom MDX components wrapping section content with profile-based conditional rendering (e.g., `<PersonalizedSection level="beginner">...</PersonalizedSection>`)

## Dependencies

### External Systems
- **Qdrant Vector Database**: Required for content indexing and semantic search; ownership: Infrastructure team
  - **Collection**: `robotics_textbook_chapters`
  - **Vector Size**: 384 dimensions
  - **Distance Metric**: Cosine
  - **Index**: HNSW with m=16, ef_construct=100
  - **Payload Indexes**: `chapter_id` (keyword), `tags` (keyword), `difficulty_level` (keyword)
- **ROS 2 Humble Distribution**: Testing environment dependency; maintained by Open Robotics community
- **NVIDIA Isaac Sim**: Simulation platform for examples; provided by NVIDIA, requires GPU infrastructure

### Agent Skills
- **mdx-writer skill** (content-publisher-agent): Generates well-formed MDX with proper frontmatter
- **content-indexer skill** (content-publisher-agent): Embeds chapter content into Qdrant with metadata
- **content-adapter skill** (personalization-agent): Renders personalized variants based on user profile
- **urdu-translator skill** (personalization-agent): Translates prose while preserving technical terms
- **rag-pipeline skill** (rag-chatbot-agent): Executes semantic search and measures retrieval quality

### Internal Teams
- **Content Development Team**: Provides domain expertise for robotics curriculum design
- **Platform Engineering**: Maintains Qdrant infrastructure and embedding models
- **Quality Assurance**: Validates code examples in ROS 2 Humble environment

## Scope

### In Scope
- Generation of 10 complete MDX chapter files with frontmatter, content, code examples, and exercises
- Python and C++ code examples validated against ROS 2 Humble
- Exercise creation with beginner and intermediate difficulty levels including solutions
- MDX frontmatter schema design for personalization and indexing
- Content indexing into Qdrant vector database with embeddings
- Searchability validation using NDCG@10 metric on test queries
- Personalization marker implementation for experience level, ROS familiarity, and hardware access
- Technical terminology preservation markers for translation compatibility
- Readability validation using Flesch-Kincaid grade level scoring
- Quality gate implementation for code execution, readability, and search performance

### Out of Scope
- **Actual Urdu translation**: Only marking terms for preservation; translation execution is separate workflow
- **Interactive code execution environment**: Chapters provide code examples for local execution, not in-browser IDE
- **Video tutorials or multimedia content**: Text and code only; visual aids limited to static diagrams if needed
- **Advanced difficulty exercises**: Only beginner and intermediate levels required
- **ROS 1 compatibility**: Code examples target ROS 2 Humble exclusively
- **Custom ROS package development**: Examples use standard ROS 2 packages; no custom package creation tutorials
- **Physical robot hardware setup guides**: Assumes hardware is pre-configured; focus on software/simulation
- **Real-time content updates**: Chapters are static content, not dynamically updated based on ROS 2 releases
- **User progress tracking**: Content delivery only; no learner management system integration
- **Certification or assessment beyond exercises**: No formal testing or credential issuance
- **Community features**: No discussion forums, comments, or collaborative editing
- **Content versioning**: Initial version only; future updates not in scope for Phase 2
