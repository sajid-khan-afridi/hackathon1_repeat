# Tasks: MDX Textbook Chapters for Physical AI & Humanoid Robotics

**Input**: Design documents from `/specs/002-mdx-textbook-chapters/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: This feature includes code validation tests via GitHub Actions CI

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **MDX Content**: `docs/module-{N}-{name}/chapter-{N}-{slug}.mdx`
- **Components**: `src/components/`
- **Tests**: `tests/`
- **CI**: `.github/workflows/`
- **Data**: `src/data/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create module directory structure for 10 chapters in docs/
- [X] T002 [P] Create PersonalizedSection React component in src/components/PersonalizedSection.tsx
- [X] T003 [P] Create TechnicalTerm React component in src/components/TechnicalTerm.tsx
- [X] T004 Create technical-terms.json glossary in src/data/technical-terms.json
- [X] T005 Create code extraction script in scripts/extract-code-examples.py
- [X] T006 Create code validation script in scripts/validate-code-examples.py
- [X] T007 Create readability validation script in scripts/validate-readability.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T008 Setup GitHub Actions workflow for code example validation in .github/workflows/validate-code-examples.yml
- [X] T009 Setup GitHub Actions workflow for readability validation in .github/workflows/validate-readability.yml
- [X] T010 Create test query set for NDCG@10 measurement in tests/search-queries.json
- [X] T011 Create Qdrant indexing script in scripts/index-content-to-qdrant.py
- [X] T012 Create NDCG measurement script in scripts/measure-ndcg.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Educational Content Discovery and Learning (Priority: P1) 🎯 MVP

**Goal**: Students access the robotics textbook, navigate chapters, read explanations, run code examples locally, and test understanding through exercises

**Independent Test**: Learner opens a chapter, reads content, copies code examples into ROS 2 Humble environment, executes them successfully, and completes exercises

### Implementation for User Story 1

- [X] T013 [US1] Create Chapter 1: ROS 2 Nodes & Lifecycle in docs/module-1-ros2-fundamentals/chapter-1-nodes-lifecycle.mdx
- [X] T014 [US1] Create Chapter 2: ROS 2 Topics & Services in docs/module-1-ros2-fundamentals/chapter-2-topics-services.mdx
- [X] T015 [US1] Create Chapter 3: ROS 2 Parameters & Launch in docs/module-1-ros2-fundamentals/chapter-3-parameters-launch.mdx

**For each chapter in User Story 1:**
- [X] T016 [P] [US1] Add frontmatter with learning objectives, tags, and metadata
- [X] T017 [P] [US1] Include minimum 3 executable code examples in Python/C++
- [X] T018 [P] [US1] Add 2 beginner and 2 intermediate exercises with solutions
- [X] T019 [P] [US1] Mark technical terms with TechnicalTerm component
- [X] T020 [P] [US1] Add PersonalizedSection components for experience levels
- [X] T021 [US1] Validate code examples execute 100% in ROS 2 Humble
- [X] T022 [US1] Validate Flesch-Kincaid readability 12-14

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Personalized Learning Experience (Priority: P2)

**Goal**: Learners with different backgrounds receive content adapted to their experience level and available resources

**Independent Test**: Configure user profiles with different experience levels and verify chapter content adapts appropriately

### Implementation for User Story 2

- [X] T023 [US2] Create Chapter 4: Isaac Sim Environment Setup in docs/module-2-isaac-sim/chapter-1-introduction-comprehensive.mdx
- [X] T024 [US2] Create Chapter 5: Robot Models & URDF in docs/module-2-isaac-sim/chapter-2-urdf.mdx
- [X] T026 [US2] Create Chapter 6: Sensors & Physics in docs/module-2-isaac-sim/chapter-3-sensors.mdx

**For each chapter in User Story 2:**
- [X] T027 [P] [US2] Add personalization variants for experience_level (beginner/intermediate/advanced)
- [X] T028 [P] [US2] Add personalization variants for hardware_access (simulation vs hardware)
- [X] T029 [P] [US2] Add personalization variants for ros_familiarity (novice/expert)
- [X] T030 [P] [US2] Include alternative code examples for different user profiles
- [X] T031 [US2] Validate content variants render correctly

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Multilingual Technical Content Access (Priority: P3)

**Goal**: Urdu-speaking students access technical robotics content in their native language while preserving critical technical terminology in English

**Independent Test**: Switch language preference to Urdu and verify that descriptive text is translated while technical terms remain in English

### Implementation for User Story 3

- [X] T032 [US3] Create Chapter 7: Kinematics in docs/module-3-applications/chapter-1-kinematics.mdx
- [X] T033 [US3] Create Chapter 8: Navigation & Path Planning in docs/module-3-applications/chapter-2-navigation.mdx

**For each chapter in User Story 3:**
- [X] T034 [P] [US3] Mark all technical terms with TechnicalTerm component
- [X] T035 [P] [US3] Add Urdu contextual explanations in technical-terms.json
- [X] T036 [US3] Validate technical terms are marked for translation preservation
- [X] T037 [US3] Test translation compatibility with urdu-translator skill

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Content Quality Assurance and Validation (Priority: P1)

**Goal**: Content creators and educators validate that published chapters meet educational quality standards

**Independent Test**: Run automated quality gates on completed chapters: execute code examples, measure readability, perform test searches

### Implementation for User Story 4

- [X] T038 [US4] Create Chapter 9: Perception & Sensor Fusion in docs/module-3-applications/chapter-3-perception.mdx
- [X] T039 [US4] Create Chapter 10: Humanoid Control in docs/module-3-applications/chapter-4-humanoid.mdx

**For each chapter in User Story 4:**
- [X] T040 [P] [US4] Validate all code examples with `# test: true` marker
- [X] T041 [P] [US4] Run CI pipeline to verify 100% code execution success
- [X] T042 [P] [US4] Validate Flesch-Kincaid scores between 12-14
- [X] T043 [P] [US4] Index chapter content into Qdrant vector database
- [X] T044 [US4] Run NDCG@10 measurement on test queries
- [X] T045 [US4] Verify NDCG@10 score > 0.8 for all chapters

**Checkpoint**: All chapters should pass quality gates and be indexed

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T046 [P] Index all 10 chapters into Qdrant vector database
- [X] T047 [P] Run comprehensive NDCG@10 validation on all test queries
- [X] T048 Create content contributor guide in docs/contributing/chapter-authoring.md
- [X] T049 [P] Add alt text to all diagrams for accessibility
- [X] T050 Verify all chapters follow consistent MDX structure
- [X] T051 Validate all code examples have comprehensive inline comments
- [X] T052 Run final quality gate validation across all chapters
- [X] T053 Update sidebar navigation configuration for all modules

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order: US1 (P1), US4 (P1), US2 (P2), US3 (P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable
- **User Story 4 (P1)**: Can start after Foundational (Phase 2) - Provides quality validation for all chapters

### Within Each User Story

- Content structure before code examples
- Code examples before exercises
- Quality validation after chapter completion
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- Different chapters within the same user story can be created in parallel
- Quality validation tasks can run in parallel for different chapters

---

## Parallel Example: User Story 1

```bash
# Create all three ROS 2 chapters in parallel:
Task: "Create Chapter 1: ROS 2 Nodes & Lifecycle in docs/module-1-ros2-fundamentals/chapter-1-nodes-lifecycle.mdx"
Task: "Create Chapter 2: ROS 2 Topics & Services in docs/module-1-ros2-fundamentals/chapter-2-topics-services.mdx"
Task: "Create Chapter 3: ROS 2 Parameters & Launch in docs/module-1-ros2-fundamentals/chapter-3-parameters-launch.mdx"

# Add content elements in parallel for each chapter:
Task: "Add frontmatter with learning objectives, tags, and metadata"
Task: "Include minimum 3 executable code examples in Python/C++"
Task: "Add 2 beginner and 2 intermediate exercises with solutions"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Chapters 1-3)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 4 → Test independently → Deploy/Demo (Quality Gates!)
4. Add User Story 2 → Test independently → Deploy/Demo (Personalization!)
5. Add User Story 3 → Test independently → Deploy/Demo (Translation Support!)
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Chapters 1-3)
   - Developer B: User Story 4 (Chapters 9-10 + Quality)
   - Developer C: User Story 2 (Chapters 4-6)
3. User Story 3 can be added after initial content is complete

---

## Quality Gates Checklist

Each chapter must pass:
- [ ] MDX frontmatter schema validation
- [ ] Code examples execute 100% in ROS 2 Humble
- [ ] Flesch-Kincaid readability 12-14
- [ ] Minimum 3 code examples included
- [ ] Minimum 4 exercises (2 beginner + 2 intermediate)
- [ ] Technical terms marked with TechnicalTerm component
- [ ] PersonalizedSection components where applicable
- [ ] Content indexed in Qdrant
- [ ] Search queries achieve NDCG@10 > 0.8

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Code examples must include `# test: true` marker for CI validation
- All ROS 2 code must target Humble LTS distribution
- Personalization is implemented at section level, not full chapter level
- Technical terms remain in English during translation
- Quality gates prevent broken content from reaching production
