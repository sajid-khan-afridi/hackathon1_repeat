# Implementation Plan: MDX Textbook Chapters for Physical AI & Humanoid Robotics

**Branch**: `001-mdx-textbook-chapters` | **Date**: 2025-12-12 | **Spec**: [specs/002-mdx-textbook-chapters/spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-mdx-textbook-chapters/spec.md`

**Note**: This plan follows Phase 2 (Content Creation) of the project constitution.

## Summary

Create 10 complete MDX textbook chapters covering ROS 2 fundamentals (Ch1-3), NVIDIA Isaac Sim (Ch4-6), and robotics applications (Ch7-10). Each chapter must include proper MDX frontmatter with learning objectives and personalization markers, executable Python/C++ code examples validated against ROS 2 Humble, exercises with solutions at beginner/intermediate levels, and technical terminology preservation markers for translation compatibility. Content will be indexed into Qdrant vector database for RAG chatbot search.

## Technical Context

**Language/Version**: TypeScript/JavaScript (Docusaurus), Python 3.10+ (ROS 2 examples), C++ 17+ (ROS 2 examples)
**Primary Dependencies**: Docusaurus v3, MDX 3.0, ROS 2 Humble, NVIDIA Isaac Sim 2023.1.0+, rclpy, rclcpp
**Storage**: MDX files in `docs/` directory, Qdrant vector database for content indexing
**Testing**: Jest (Docusaurus components), GitHub Actions CI (code example validation), textstat (readability scoring)
**Target Platform**: Web (Docusaurus static site on GitHub Pages), ROS 2 Humble on Ubuntu 22.04
**Project Type**: Web application with static content generation
**Performance Goals**: Flesch-Kincaid readability 12-14, NDCG@10 > 0.8 for search queries, code execution 100% pass rate
**Constraints**: Free tier limits (Qdrant 1GB, GitHub Pages 100GB/month), 20-30 min reading time per chapter
**Scale/Scope**: 10 chapters, 30+ code examples, 40+ exercises, 1000+ vector chunks

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Phase 2 Quality Gates (from Constitution)
<!-- Reference: .specify/memory/constitution.md lines 939-946 -->
- [ ] 10 chapters authored in MDX format
- [ ] All code examples execute without errors (CI test)
- [ ] Learning objectives defined per chapter
- [ ] Peer review completed by domain expert
- [ ] Readability Flesch-Kincaid score 12-14
- [ ] PHR created summarizing content milestones

### Additional Requirements (from Feature Spec)
- [ ] Each chapter MUST include structured frontmatter (title, learning_objectives, tags, difficulty_level, ros_version, prerequisites, estimated_time, personalization_variants)
- [ ] Each chapter MUST contain at least 3 executable code examples in Python and/or C++
- [ ] Each chapter MUST provide exercises at beginner and intermediate difficulty levels with complete solution code
- [ ] Code examples MUST execute successfully with 100% pass rate in clean ROS 2 Humble environment
- [ ] Technical terminology MUST be marked for preservation during translation
- [ ] All chapter content MUST be indexed into Qdrant vector database
- [ ] NDCG@10 score above 0.8 on predefined test queries

### Core Principles Alignment

**✅ Quality Over Speed**: Code examples tested via GitHub Actions CI with ros:humble Docker image. Readability validated with Flesch-Kincaid scoring. Search quality measured with NDCG@10.

**✅ Smallest Viable Change**: Focus on 10 chapters covering core curriculum. No custom ROS packages, no video content, no interactive IDE. Reuse existing Docusaurus infrastructure from Phase 1.

**✅ Security by Default**: No secrets in code examples. Code examples use standard ROS 2 packages without custom package creation requiring elevated permissions.

**✅ Observability & Measurability**:
- Code execution: 100% pass rate (measurable in CI)
- Readability: Flesch-Kincaid 12-14 (measurable with textstat)
- Search quality: NDCG@10 > 0.8 (measurable with test queries)
- Chapter count: 10 chapters (countable)
- Exercise count: 4+ per chapter (countable)

**✅ Accessibility & Inclusivity**: Alt text for all diagrams. Technical terminology marked for translation. PersonalizedSection components for different experience levels.

**✅ Free Tier Sustainability**: Content fits within Qdrant 1GB limit (~100k chunks). Static MDX files hosted on GitHub Pages.

### Agent Ownership
**Primary**: ContentWriter Agent (owns: `mdx-writer`)
**Support**: RAGArchitect Agent (owns: `content-indexer`, `qdrant-vectorstore`), PersonalizationEngine Agent (owns: `content-adapter`)
**Coordinator**: Orchestrator Agent (quality gate approval)

### Complexity Violations
*None identified - Phase follows YAGNI principle. Using existing Docusaurus infrastructure with MDX format. No new frameworks or abstractions introduced.*

## Project Structure

### Documentation (this feature)

```text
specs/002-mdx-textbook-chapters/
├── plan.md              # This file
├── research.md          # Phase 0: Technology research
├── data-model.md        # Phase 1: Entity schemas
├── quickstart.md        # Phase 1: Contributor onboarding
├── contracts/           # Phase 1: API contracts for content indexing
│   ├── frontmatter-schema.yaml    # MDX frontmatter schema
│   └── qdrant-payload-schema.yaml # Vector database payload schema
├── checklists/
│   └── requirements.md  # Requirements checklist
└── tasks.md             # Phase 2: Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
# Phase 2 Content (extending existing Docusaurus site)
docs/
├── module-1-ros2-fundamentals/      # Chapters 1-3
│   ├── chapter-1-publishers.mdx     # (existing sample)
│   ├── chapter-2-services.mdx       # ROS 2 Topics & Services
│   └── chapter-3-parameters.mdx     # ROS 2 Parameters & Launch
├── module-2-isaac-sim/              # Chapters 4-6
│   ├── chapter-1-introduction.mdx   # (existing sample)
│   ├── chapter-2-urdf.mdx           # Robot Models & URDF
│   └── chapter-3-sensors.mdx        # Sensors & Physics
├── module-3-applications/           # Chapters 7-10
│   ├── chapter-1-kinematics.mdx     # Kinematics
│   ├── chapter-2-navigation.mdx     # Navigation & Path Planning
│   ├── chapter-3-perception.mdx     # Perception & Sensor Fusion
│   └── chapter-4-humanoid.mdx       # Humanoid Control

# Supporting Infrastructure
src/
├── components/
│   └── PersonalizedSection.tsx      # Content personalization component

tests/
├── search-queries.json              # NDCG@10 test query set (20 queries)
└── code-examples/                   # Extracted code for CI testing

# CI/CD Configuration
.github/workflows/
└── validate-code-examples.yml       # ROS 2 code validation pipeline
```

**Structure Decision**: Extends existing Docusaurus structure from Phase 1. Chapters organized into 3 modules following spec's topic distribution. Test infrastructure added for code validation and search quality measurement.

## Phase 0: Research & Technical Context

### Research Tasks

1. **MDX Frontmatter Best Practices for Educational Content**
   - Decision: Use YAML frontmatter with strict schema (title, learning_objectives, tags, difficulty_level, ros_version, prerequisites, estimated_time, personalization_variants)
   - Rationale: Structured metadata enables content indexing, personalization, and curriculum navigation
   - Alternatives: JSON frontmatter (less readable), no frontmatter (loses indexing capability)

2. **ROS 2 Humble Code Example Patterns**
   - Decision: Use rclpy (Python) and rclcpp (C++) with standard message types (std_msgs, geometry_msgs, sensor_msgs)
   - Rationale: Standard packages available in ros:humble Docker image without custom package builds
   - Alternatives: Custom packages (requires complex CI setup), ROS 1 (outdated)

3. **Content Personalization Approach**
   - Decision: Section-level personalization via PersonalizedSection React component with props: level, rosFamiliarity, hardwareAccess
   - Rationale: Allows content variants within single MDX file, cleaner than separate chapter versions
   - Alternatives: Separate chapter files per variant (content duplication), inline conditional text (messy)

4. **Vector Database Chunking Strategy**
   - Decision: Chunk chapters by section (h2 headings), embed with all-MiniLM-L6-v2 (384 dimensions), store in Qdrant
   - Rationale: Section-level chunks balance context size with search precision
   - Alternatives: Paragraph-level (too granular), full chapter (too broad)

5. **Code Example Validation in CI**
   - Decision: GitHub Actions workflow extracts code blocks with `# test: true` marker, runs in ros:humble Docker container
   - Rationale: Automated validation prevents broken examples in production
   - Alternatives: Manual testing (error-prone), no validation (breaks learner experience)

6. **Readability Measurement**
   - Decision: Use textstat Python library for Flesch-Kincaid scoring during CI
   - Rationale: Automated readability checks ensure consistent college-level technical writing
   - Alternatives: Manual review (subjective), no measurement (uncontrolled quality)

### Dependencies Confirmed
- Docusaurus v3 with MDX support (from Phase 1)
- ROS 2 Humble Docker image (ros:humble-ros-base-jammy)
- all-MiniLM-L6-v2 embedding model (sentence-transformers)
- Qdrant Cloud free tier (1GB storage)
- textstat Python library for readability scoring

## Phase 1: Design & Contracts

### Data Model

#### Chapter Entity
```typescript
interface Chapter {
  id: string;                        // e.g., "ch01-ros2-publishers"
  module: 'ros2-fundamentals' | 'isaac-sim' | 'applications';
  chapterNumber: number;             // 1-10
  title: string;
  learningObjectives: string[];      // Min 3 items
  tags: string[];                    // For indexing
  difficultyLevel: 'beginner' | 'intermediate' | 'advanced';
  rosVersion: string;                // e.g., "humble"
  prerequisites: string[];           // Chapter IDs
  estimatedTime: number;             // Minutes
  personalizationVariants: {
    experienceLevel: boolean;
    rosFamiliarity: boolean;
    hardwareAccess: boolean;
  };
  sections: Section[];
  exercises: Exercise[];
  codeExamples: CodeExample[];
}
```

#### Section Entity
```typescript
interface Section {
  id: string;                        // e.g., "sec-1-2-creating-publisher"
  heading: string;
  level: 2 | 3 | 4;                  // h2, h3, h4
  content: string;                   // MDX content
  personalizedVariants?: PersonalizedVariant[];
}
```

#### CodeExample Entity
```typescript
interface CodeExample {
  id: string;                        // e.g., "ex-simple-publisher-py"
  chapterId: string;
  language: 'python' | 'cpp';
  code: string;
  rosVersion: string;
  requiredPackages: string[];        // e.g., ["rclpy", "std_msgs"]
  environment: 'simulation' | 'hardware' | 'both';
  expectedOutput?: string;
  validated: boolean;                // CI validation status
  testable: boolean;                 // Has # test: true marker
}
```

#### Exercise Entity
```typescript
interface Exercise {
  id: string;                        // e.g., "ex01-ch01-pub-sensor"
  chapterId: string;
  difficultyLevel: 'beginner' | 'intermediate';
  learningObjective: string;
  problemDescription: string;
  starterCode?: string;
  solution: string;
  expectedResult: string;
  troubleshootingSteps: string[];
}
```

#### PersonalizedVariant Entity
```typescript
interface PersonalizedVariant {
  variantType: 'experience_level' | 'ros_familiarity' | 'hardware_access';
  targetValue: string;               // e.g., "beginner", "expert", "true"
  replacementContent: string;
}
```

#### TechnicalTerm Entity
```typescript
interface TechnicalTerm {
  term: string;                      // e.g., "ROS 2"
  translationStatus: 'preserve' | 'transliterate';
  contextualExplanation?: string;    // Urdu explanation
}
```

#### SearchQuery Entity (for NDCG testing)
```typescript
interface SearchQuery {
  id: string;                        // e.g., "q01"
  query: string;
  type: 'exact-match' | 'conceptual';
  targetChapter: string;             // Primary relevant chapter
  relevanceGrades: Record<string, 0 | 1 | 2 | 3>;
}
```

### API Contracts

#### MDX Frontmatter Schema
See `contracts/frontmatter-schema.yaml`

#### Qdrant Payload Schema
See `contracts/qdrant-payload-schema.yaml`

### Chapter Outline

Based on spec clarification (session 2025-12-12):

| Chapter | Title | Module | Prerequisites | Key Topics |
|---------|-------|--------|---------------|------------|
| Ch1 | ROS 2 Nodes & Lifecycle | ROS 2 Fundamentals | None | Create nodes, lifecycle states, lifecycle callbacks |
| Ch2 | ROS 2 Topics & Services | ROS 2 Fundamentals | Ch1 | Publish/subscribe, services, QoS |
| Ch3 | ROS 2 Parameters & Launch | ROS 2 Fundamentals | Ch1, Ch2 | Parameter declaration, launch files, multi-node orchestration |
| Ch4 | Isaac Sim Environment Setup | Isaac Sim | None | Install, configure, UI navigation, ROS 2 bridge |
| Ch5 | Robot Models & URDF | Isaac Sim | Ch4 | Create URDF, import robots, configure joints/links |
| Ch6 | Sensors & Physics | Isaac Sim | Ch4, Ch5 | Add sensors (camera, lidar, IMU), physics properties, ROS 2 data |
| Ch7 | Kinematics | Applications | Ch1-3, Ch5 | Forward/inverse kinematics, IK solvers, arm positioning |
| Ch8 | Navigation & Path Planning | Applications | Ch1-3, Ch4-6 | Nav2 stack, path planning algorithms, obstacle avoidance |
| Ch9 | Perception & Sensor Fusion | Applications | Ch1-3, Ch6 | Camera/lidar processing, sensor fusion, object detection |
| Ch10 | Humanoid Control | Applications | Ch7-9 | Bipedal locomotion, balance control, full-body motion |

### Test Query Set (for NDCG@10)

20 queries (2 per chapter), mix of exact-match and conceptual:

| ID | Query | Type | Target | Relevance Grades |
|----|-------|------|--------|------------------|
| q01 | "how to create ROS 2 publisher" | exact-match | ch01 | ch01:3, ch02:2, ch03:1 |
| q02 | "ROS 2 node lifecycle states" | exact-match | ch01 | ch01:3, ch02:1 |
| q03 | "publish subscribe pattern ROS 2" | exact-match | ch02 | ch02:3, ch01:2 |
| q04 | "ROS 2 service client server" | exact-match | ch02 | ch02:3, ch03:1 |
| q05 | "ROS 2 launch file example" | exact-match | ch03 | ch03:3, ch02:1 |
| q06 | "ROS 2 parameters declaration" | exact-match | ch03 | ch03:3 |
| q07 | "Isaac Sim installation setup" | exact-match | ch04 | ch04:3 |
| q08 | "how to connect Isaac Sim to ROS 2" | conceptual | ch04 | ch04:3, ch06:2 |
| q09 | "create robot URDF model" | exact-match | ch05 | ch05:3, ch06:1 |
| q10 | "import robot into Isaac Sim" | exact-match | ch05 | ch05:3, ch04:1 |
| q11 | "add camera sensor Isaac Sim" | exact-match | ch06 | ch06:3, ch04:1 |
| q12 | "simulate lidar data ROS 2" | conceptual | ch06 | ch06:3, ch09:2 |
| q13 | "forward kinematics robot arm" | exact-match | ch07 | ch07:3 |
| q14 | "inverse kinematics solver" | exact-match | ch07 | ch07:3, ch10:1 |
| q15 | "ROS 2 Nav2 navigation" | exact-match | ch08 | ch08:3 |
| q16 | "path planning obstacle avoidance" | conceptual | ch08 | ch08:3, ch09:1 |
| q17 | "process camera data ROS 2" | conceptual | ch09 | ch09:3, ch06:2 |
| q18 | "sensor fusion techniques" | conceptual | ch09 | ch09:3, ch06:1 |
| q19 | "bipedal robot balance control" | exact-match | ch10 | ch10:3 |
| q20 | "humanoid full body motion planning" | exact-match | ch10 | ch10:3, ch07:2, ch08:1 |

## Post-Design Constitution Check (Phase 2 Design Complete)

### ✅ All Quality Gates Remain Valid
<!-- Reference: .specify/memory/constitution.md lines 939-946 -->
- [ ] 10 chapters authored in MDX format
- [ ] All code examples execute without errors (CI test)
- [ ] Learning objectives defined per chapter
- [ ] Peer review completed by domain expert
- [ ] Readability Flesch-Kincaid score 12-14
- [ ] PHR created summarizing content milestones

### ✅ Design Artifacts Generated
- ✅ `research.md` - Technology decisions for MDX, ROS 2, personalization, chunking, CI validation, readability
- ✅ `data-model.md` - Entity schemas for Chapter, Section, CodeExample, Exercise, PersonalizedVariant, TechnicalTerm, SearchQuery
- ✅ `quickstart.md` - Contributor onboarding guide for chapter authoring
- ✅ `contracts/` - frontmatter-schema.yaml, qdrant-payload-schema.yaml

### ✅ No New Complexity Violations

**Analysis**:
- **YAGNI Compliance**: Only infrastructure needed for 10 chapters. No custom ROS packages, no video content, no interactive IDE. Reuses Phase 1 Docusaurus.
- **Security**: No secrets in code examples. Standard ROS 2 packages only.
- **Performance**:
  - Flesch-Kincaid 12-14 validated via textstat
  - Code examples validated in CI (ros:humble Docker)
  - NDCG@10 > 0.8 measured with 20 test queries
- **Accessibility**: Alt text required for diagrams. Technical terms marked for translation.
- **Free Tier**: ~1000 chunks fits within Qdrant 1GB limit.

### ✅ Recommended ADRs

**Decision Detected**: Section-Level Personalization via React Components
- **Impact**: Determines how content adapts to user profiles (experience_level, ros_familiarity, hardware_access). Affects PersonalizationEngine Agent's content-adapter skill.
- **Alternatives**: Separate chapter files per variant (content duplication nightmare), inline conditional rendering (messy MDX)
- **Scope**: Cross-cutting - affects all 10 chapters, PersonalizationEngine Agent, and future translation workflow

📋 Architectural decision detected: **Section-Level Personalization via PersonalizedSection Component**
   Document reasoning and tradeoffs? Run `/sp.adr section-level-personalization`

**Decision Detected**: Code Example Validation Pipeline
- **Impact**: Determines how code examples are tested before merge. Affects CI/CD workflow and contributor experience.
- **Alternatives**: Manual testing (error-prone), no validation (broken examples in production)
- **Scope**: Cross-cutting - affects all 30+ code examples across 10 chapters

📋 Architectural decision detected: **GitHub Actions CI with ros:humble Docker for Code Validation**
   Document reasoning and tradeoffs? Run `/sp.adr code-validation-pipeline`

*Waiting for user consent before creating ADRs.*

---

## Next Steps

1. **Generate supporting artifacts**:
   - `research.md` - Consolidate technology decisions
   - `data-model.md` - Entity schemas
   - `contracts/` - API contract files
   - `quickstart.md` - Contributor onboarding

2. **Run `/sp.tasks`** to generate implementation task list from this plan

3. **Address ADR suggestions** (if approved by user)

4. **Begin Chapter Authoring** following task list priority order
