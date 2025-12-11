---

description: "Task list for book-infrastructure feature implementation"
---

# Tasks: Book Infrastructure (001-docusaurus-init)

**Input**: Design documents from `/specs/001-docusaurus-init/`
**Prerequisites**: plan.md (✓), spec.md (✓), research.md (✓), data-model.md, contracts/

**Tests**: Not requested in specification - tasks focus on implementation only

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Static Site**: Repository root (docs/, src/, static/)
- **Configuration**: Root level config files
- **Components**: `src/components/`
- **Pages**: `src/pages/`
- **Styles**: `src/css/` and `src/theme/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic Docusaurus structure

- [ ] T001 Initialize Docusaurus v3 project with TypeScript support via `docusaurus-init` skill
- [ ] T002 Configure tsconfig.json with strict mode enabled
- [ ] T003 Install core dependencies (React 18.x, @docusaurus/core, @docusaurus/preset-classic)
- [ ] T004 Create initial project structure per plan.md (docs/, src/, static/)
- [ ] T005 [P] Configure package.json scripts (start, build, serve)
- [ ] T006 [P] Initialize Git repository with .gitignore for Node.js and Docusaurus

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Configure docusaurus.config.ts with site metadata and organization settings
- [ ] T008 Set up CSS custom properties for theming design tokens in src/css/custom.css
- [ ] T009 [P] Create base layout component with semantic HTML structure in src/theme/Layout/
- [ ] T010 Configure ESLint and Prettier for code quality (.eslintrc.js, .prettierrc)
- [ ] T011 [P] Add Infima CSS framework imports and theme variable definitions
- [ ] T012 Configure MDX support in docusaurus.config.ts for educational content

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: US1: Local Development (Priority: P1) 🎯 MVP

**Goal**: Developer can initialize and run Docusaurus locally - MVP entry point

**Independent Test**: Developer runs `npm start` and sees documentation site at localhost:3000

### Implementation for US1

- [ ] T013 [US1] Create index.mdx homepage with hero section and introduction in docs/intro.mdx
- [ ] T014 [US1] Add sample chapter structure (chapter-01-introduction/index.mdx) in docs/chapter-01-introduction/
- [ ] T015 [US1] Configure sidebar navigation in docusaurus.config.ts for auto-generated structure
- [ ] T016 [US1] Verify `npm run start` works locally without errors
- [ ] T017 [US1] Verify `npm run build` produces static output in build/ directory

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: US2: Theme Toggle (Priority: P1)

**Goal**: User can toggle between dark and light themes with persistence

**Independent Test**: Toggle works, refreshes preserve choice, and system preference is detected on first visit

### Implementation for US2

- [ ] T018 [P] [US2] Implement ThemeToggle React component in src/components/ThemeToggle/index.tsx
- [ ] T019 [P] [US2] Create theme context provider in src/components/ThemeToggle/ThemeContext.tsx
- [ ] T020 [US2] Add system preference detection using `prefers-color-scheme` media query
- [ ] T021 [US2] Add localStorage persistence for manual theme toggle choice
- [ ] T022 [US2] Integrate ThemeToggle component in navbar via src/theme/NavbarItem/
- [ ] T023 [US2] Configure theme colors with 4.5:1 contrast ratio in both light and dark modes
- [ ] T024 [US2] Add keyboard accessibility (Enter/Space keys) to ThemeToggle component
- [ ] T025 [US2] Add smooth color transitions respecting `prefers-reduced-motion`
- [ ] T026 [US2] Add inline theme detection script to prevent FOUC (flash of unstyled content)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: US3: Responsive Layout (Priority: P2)

**Goal**: Content displays responsively across all device breakpoints

**Independent Test**: Site renders correctly on mobile (320px-767px), tablet (768px-1023px), and desktop (1024px+) without horizontal scroll

### Implementation for US3

- [ ] T027 [P] [US3] Define CSS breakpoints (320px, 768px, 1024px) in src/css/custom.css
- [ ] T028 [P] [US3] Create responsive navigation with hamburger menu on mobile in src/theme/Navbar/
- [ ] T029 [P] [US3] Ensure touch targets are ≥44x44px on mobile devices for all interactive elements
- [ ] T030 [US3] Test layout at all breakpoints to verify no horizontal scrolling
- [ ] T031 [US3] Configure mobile-first responsive design patterns in CSS custom properties
- [ ] T032 [US3] Add drawer-style sidebar for tablet view (collapsible navigation)
- [ ] T033 [US3] Implement 3-column desktop layout (sidebar 280px, content max-width 800px, TOC 280px)

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: US4: GitHub Pages Deployment (Priority: P2)

**Goal**: Site automatically builds and deploys to GitHub Pages on push to main

**Independent Test**: Push to main triggers successful deployment within 10 minutes

### Implementation for US4

- [ ] T034 [US4] Configure GitHub Pages deployment settings in repository
- [ ] T035 [US4] Create .github/workflows/deploy.yml with Node.js 20 and caching
- [ ] T036 [P] [US4] Add node_modules caching strategy for faster builds
- [ ] T037 [US4] Configure base URL for GitHub Pages in docusaurus.config.ts
- [ ] T038 [US4] Set up deployment notifications on build failure
- [ ] T039 [US4] Configure deployment to gh-pages branch with force-push
- [ ] T040 [US4] Add build artifact management and rollback strategy

---

## Phase 7: Polish & Quality Gate

**Purpose**: Quality assurance, performance optimization, and documentation

- [ ] T041 [P] Run Lighthouse audit and address any performance issues
- [ ] T042 [P] Run Lighthouse audit and address any accessibility issues
- [ ] T043 [P] Run Lighthouse audit and address any best practices issues
- [ ] T044 [P] Verify all Phase 1 exit criteria from constitution are met
- [ ] T045 Create PHR documenting infrastructure decisions in history/prompts/001-docusaurus-init/
- [ ] T046 [P] Add meta tags for SEO and social sharing
- [ ] T047 [P] Add favicon and PWA manifest files
- [ ] T048 Validate all user stories work end-to-end
- [ ] T049 Suggest ADR for theming architecture if not already created

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can proceed in parallel:
    - US2 (Theme) and US3 (Responsive) can run in parallel after Foundational
    - US4 (Deployment) can be configured in parallel with US2/US3
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational - Independent of US1
- **User Story 3 (P2)**: Can start after Foundational - Independent of US1/US2
- **User Story 4 (P2)**: Can start after Foundational - Independent of US1/US2/US3

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority
- Each story should be independently testable

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes:
  - US2 (Theme Toggle) and US3 (Responsive Layout) can run in parallel
  - US4 (Deployment) can run in parallel with US2/US3 after US1 is complete
- All Polish phase tasks marked [P] can run in parallel

---

## Parallel Example: User Stories 2 & 3

```bash
# Launch Theme Toggle (US2) tasks in parallel:
Task: T018 - Implement ThemeToggle React component in src/components/ThemeToggle/index.tsx
Task: T019 - Create theme context provider in src/components/ThemeToggle/ThemeContext.tsx
Task: T023 - Configure theme colors with 4.5:1 contrast ratio in both light and dark modes

# Launch Responsive Layout (US3) tasks in parallel:
Task: T027 - Define CSS breakpoints (320px, 768px, 1024px) in src/css/custom.css
Task: T028 - Create responsive navigation with hamburger menu on mobile in src/theme/Navbar/
Task: T029 - Ensure touch targets are ≥44x44px on mobile devices for all interactive elements
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (MVP)
   - Developer B: User Story 2 (Theme Toggle)
   - Developer C: User Story 3 (Responsive Layout)
   - Developer D: User Story 4 (Deployment Configuration)
3. Stories complete and integrate independently

---

## Success Criteria Validation

### Lighthouse Score Targets

- Performance: > 85 (target: 90-95)
- Accessibility: > 90 (WCAG 2.1 AA compliance)
- Best Practices: > 90
- SEO: > 90 (automatically included)

### Independent Test Criteria

- **US1**: `npm start` works locally without errors
- **US2**: Theme persists across browser sessions and respects system preference
- **US3**: No horizontal scroll at any breakpoint, touch targets ≥44x44px
- **US4**: Push to main triggers successful GitHub Pages deployment

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- US2 and US3 can run in parallel after Foundational phase
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence