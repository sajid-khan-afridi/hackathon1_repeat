---

description: "Task list for implementing Docusaurus book infrastructure feature"
---

# Tasks: Docusaurus Book Infrastructure

**Input**: Design documents from `/specs/002-docusaurus-book-infra/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Tests**: Tests are OPTIONAL for Phase 1 (static site infrastructure). Unit/component tests will be included for custom React components.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**Review Checkpoints**: After each phase, pause for human review and approval before proceeding to the next phase.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Static site structure**: `docs/`, `src/`, `static/`, `tests/` at repository root
- **Docusaurus config**: `docusaurus.config.ts`, `sidebars.ts`
- **Custom components**: `src/components/`
- **Styles**: `src/css/`
- **Tests**: `tests/unit/`, `tests/e2e/`, `tests/lighthouse/`

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize Docusaurus project and basic configuration

- [ ] T001 Initialize Docusaurus v3 project with TypeScript in docs/ directory
- [ ] T002 Configure package.json with required dependencies and scripts
- [ ] T003 [P] Create basic project directory structure (src/, docs/, static/, tests/)
- [ ] T004 [P] Configure TypeScript (tsconfig.json) with strict mode enabled
- [ ] T005 [P] Configure ESLint and Prettier for consistent code formatting

**🚀 REVIEW CHECKPOINT 1**: Verify project initializes correctly before proceeding

---

## Phase 2: Foundational (Core Docusaurus Configuration)

**Purpose**: Essential Docusaurus setup that MUST be complete before ANY user story implementation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create docusaurus.config.ts with basic configuration
- [ ] T007 [P] Configure docs plugin for MDX support and sidebar generation
- [ ] T008 [P] Configure theme plugin with custom CSS path
- [ ] T009 [P] Set up navigation structure and default routing
- [ ] T010 Create basic docs/intro.md as landing page
- [ ] T011 [P] Configure Prism.js syntax highlighting for Python, C++, and YAML
- [ ] T012 Create src/css/custom.css with logical properties for RTL readiness

**🚀 REVIEW CHECKPOINT 2**: Verify Docusaurus builds and runs with basic content before proceeding

---

## Phase 3: User Story 1 - Initial Content Publishing (Priority: P1) 🎯 MVP

**Goal**: Enable instructors to create MDX content that automatically publishes to the live site within 5 minutes

**Independent Test**: Create a sample MDX file, commit to GitHub, and verify automatic deployment to GitHub Pages with correct rendering

### Implementation for User Story 1

- [ ] T013 [US1] Create docs/module-1-ros2-fundamentals/_category_.json for module metadata
- [ ] T014 [US1] Create docs/module-1-ros2-fundamentals/chapter-1-publishers.mdx with ROS 2 content
- [ ] T015 [US1] Create docs/module-2-isaac-sim/_category_.json for second module
- [ ] T016 [US1] Create docs/module-2-isaac-sim/chapter-1-introduction.mdx with Isaac Sim content
- [ ] T017 [P] [US1] Test local development server with hot reload for MDX changes
- [ ] T018 [US1] Verify build process completes without errors

**🚀 REVIEW CHECKPOINT 3**: User Story 1 MVP should be fully functional locally

---

## Phase 4: User Story 2 - Responsive Learning Experience (Priority: P2)

**Goal**: Students experience consistent, optimized layouts on mobile (320px), tablet (768px), and desktop (1024px)

**Independent Test**: Access site at 320px, 768px, and 1024px widths, verify no horizontal scroll and touch targets ≥44x44px

### Implementation for User Story 2

- [ ] T019 [P] [US2] Create src/components/ResponsiveLayout with mobile-first CSS
- [ ] T020 [P] [US2] Update src/css/custom.css with responsive breakpoints and logical properties
- [ ] T021 [US2] Ensure all interactive elements meet 44x44px minimum touch target size
- [ ] T022 [US2] Test sidebar behavior on mobile (collapsible hamburger menu)
- [ ] T023 [P] [US2] Create tests/e2e/responsive.spec.ts for Playwright viewport testing

**🚀 REVIEW CHECKPOINT 4**: Verify responsive design works across all breakpoints

---

## Phase 5: User Story 3 - Personalized Reading Preference (Priority: P3)

**Goal**: Students can toggle dark mode that persists across pages and browser sessions

**Independent Test**: Toggle dark mode, navigate pages, refresh browser, and verify preference persists

### Implementation for User Story 3

- [ ] T024 [US3] Configure dark mode in docusaurus.config.ts with system preference detection
- [ ] T025 [US3] Customize dark theme colors in src/css/custom.css with WCAG AA contrast ratios
- [ ] T026 [US3] Create src/components/ThemeToggle for better UX (optional enhancement)
- [ ] T027 [US3] Test localStorage persistence and system preference detection
- [ ] T028 [US3] Verify WCAG 2.1 AA contrast ratios for both light and dark themes

**🚀 REVIEW CHECKPOINT 5**: Verify dark mode works and persists across sessions

---

## Phase 6: User Story 4 - Content Author Workflow (Priority: P2)

**Goal**: Instructors can write MDX with code examples, preview locally, and build for production

**Independent Test**: Run local dev server, edit MDX with code blocks, verify hot-reload and production build matches

### Implementation for User Story 4

- [ ] T029 [P] [US4] Configure MDX components for enhanced content rendering
- [ ] T030 [P] [US4] Test syntax highlighting for Python, C++, ROS 2, and YAML code blocks
- [ ] T031 [US4] Verify hot-reload works within 2 seconds for MDX changes
- [ ] T032 [US4] Test production build time stays under 5 minutes
- [ ] T033 [US4] Create sample code examples with syntax highlighting in chapters

**🚀 REVIEW CHECKPOINT 6**: Verify authoring workflow is efficient and reliable

---

## Phase 7: GitHub Pages Deployment Setup

**Purpose**: Configure CI/CD pipeline for automatic deployment from main branch

- [ ] T034 Create .github/workflows/deploy.yml for GitHub Actions CI/CD
- [ ] T035 Configure deployment permissions and GitHub Pages settings
- [ ] T036 [P] Add Lighthouse CI to workflow for quality gates
- [ ] T037 Test deployment workflow manually
- [ ] T038 Verify site deploys to GitHub Pages within 5 minutes of commit

**🚀 REVIEW CHECKPOINT 7**: Verify automated deployment works correctly

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final optimizations, documentation, and quality assurance

- [ ] T039 [P] Create comprehensive README.md with setup instructions
- [ ] T040 [P] Create tests/lighthouse/lighthouserc.json for performance monitoring
- [ ] T041 [P] Add favicon and logo customization
- [ ] T042 [P] Optimize images and static assets
- [ ] T043 [P] Create VS Code workspace settings and recommended extensions
- [ ] T044 Run final Lighthouse audit and verify scores meet requirements
- [ ] T045 Update quickstart.md with any final adjustments
- [ ] T046 [P] Add error handling for broken links and invalid MDX

**🚀 REVIEW CHECKPOINT 8**: Final review of complete implementation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P4 → P3)
- **Deployment Setup (Phase 7)**: Depends on all user stories being complete
- **Polish (Phase 8)**: Depends on deployment setup completion

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 content
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Applies site-wide

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- User Stories 2, 3, and selected US4 tasks can run in parallel once Foundational is complete
- All Polish phase tasks marked [P] can run in parallel

---

## Parallel Example: User Story Implementation

```bash
# After Foundational phase complete, these can run in parallel:
Task: "Create docs/module-1-ros2-fundamentals/_category_.json"
Task: "Create docs/module-2-isaac-sim/_category_.json"
Task: "Update src/css/custom.css with responsive breakpoints"
Task: "Configure dark mode in docusaurus.config.ts"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **🚀 STOP AND REVIEW**: Test User Story 1 independently
5. Deploy demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 4 → Test independently → Deploy/Demo
5. Add User Story 3 → Test independently → Deploy/Demo
6. Setup Deployment → Full automation
7. Polish and optimize

### Quality Gates Verification

After each user story completion:

- [ ] Docusaurus builds successfully (`npm run build`)
- [ ] Site runs locally without errors (`npm run start`)
- [ ] No console errors in browser DevTools
- [ ] Content renders correctly
- [ ] Responsive layout works (for US2)
- [ ] Theme toggle works (for US3)
- [ ] Code syntax highlighting works (for US4)

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- **Review checkpoints are mandatory** - wait for human approval before proceeding
- Build time must stay under 5 minutes throughout implementation
- All CSS should use logical properties for RTL readiness
- Lighthouse scores must meet requirements: Performance > 85, Accessibility > 90, Best Practices > 90