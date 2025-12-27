# Tasks: Enhancing the UI

**Input**: Design documents from `/specs/007-enhance-ui/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, quickstart.md

**Tests**: Tests are NOT explicitly requested in the spec. E2E journey tests are defined in plan.md and will be created.

**Organization**: Tasks grouped by user story for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: US1-US6 mapping to user stories from spec.md

## Path Conventions

- **Web app**: `src/` at repository root (Docusaurus project)
- **Tests**: `tests/e2e/`, `tests/unit/`, `tests/component/`
- **CSS**: `src/css/` for global styles

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Animation infrastructure and shared utilities

- [X] T001 Install animation dependencies (framer-motion, lucide-react) in package.json
- [X] T002 [P] Create global animation keyframes in src/css/animations.css
- [X] T003 [P] Create reduced motion overrides in src/css/reduced-motion.css
- [X] T004 [P] Create animations directory structure at src/components/animations/
- [X] T005 [P] Create hooks directory structure at src/hooks/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core hooks and utilities that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 [P] Implement useReducedMotion hook in src/components/animations/useReducedMotion.ts
- [X] T007 [P] Implement usePersistedState hook in src/hooks/usePersistedState.ts
- [X] T008 [P] Implement useScrollReveal hook in src/components/animations/useScrollReveal.ts
- [X] T009 [P] Implement useAnimationState hook in src/components/animations/useAnimationState.ts
- [X] T010 [P] Create FadeIn component in src/components/animations/FadeIn.tsx
- [X] T011 [P] Create SlideIn component in src/components/animations/SlideIn.tsx
- [X] T012 Create animations index export in src/components/animations/index.ts
- [X] T013 Import animations.css and reduced-motion.css in docusaurus.config.ts

**Checkpoint**: Animation foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Chatbot Panel Interaction (Priority: P1) MVP

**Goal**: Modern docked chat panel with resize, minimize, unread badge, search, and smooth animations

**Independent Test**: Open chatbot, send message, use panel controls (minimize, resize, close)

### Implementation for User Story 1

- [X] T014 [US1] Implement useChatPanelState hook in src/hooks/useChatPanelState.ts
- [X] T015 [P] [US1] Create ChatPanel container in src/components/ChatPanel/ChatPanel.tsx
- [X] T016 [P] [US1] Create ChatPanel styles in src/components/ChatPanel/ChatPanel.module.css
- [X] T017 [US1] Create ChatPanelHeader in src/components/ChatPanel/ChatPanelHeader.tsx
- [X] T018 [US1] Create ResizeHandle in src/components/ChatPanel/ResizeHandle.tsx
- [X] T019 [US1] Create UnreadBadge in src/components/ChatPanel/UnreadBadge.tsx
- [X] T020 [US1] Create JumpToLatest in src/components/ChatPanel/JumpToLatest.tsx
- [X] T021 [US1] Create ChatPanelSearch in src/components/ChatPanel/ChatPanelSearch.tsx
- [X] T022 [US1] Create ChatPanel index export in src/components/ChatPanel/index.ts
- [X] T023 [US1] Integrate ChatPanel with existing FloatingChatPopup in src/components/FloatingChatPopup/GlobalFloatingChat.tsx
- [X] T024 [US1] Add panel open/close animations (300ms slide) in ChatPanel.module.css
- [X] T025 [US1] Add message fade-in and slide-up animations in ChatPanel.module.css
- [X] T026 [US1] Implement RTL support for panel docking (left side in RTL) in ChatPanel.module.css
- [X] T027 [US1] Add focus management (focus input on open, return focus on close) in ChatPanel.tsx
- [X] T028 [US1] Add Ctrl+F keyboard shortcut for chat search in ChatPanelSearch.tsx
- [X] T029 [US1] Add localStorage fallback (use in-memory defaults when unavailable) in useChatPanelState.ts
- [X] T030 [US1] Add debounce for rapid minimize/expand actions (prevent animation conflicts) in ChatPanel.tsx
- [X] T031 [US1] Preserve panel state across page navigation in useChatPanelState.ts

**Checkpoint**: User Story 1 complete - Chatbot panel fully functional with all controls

---

## Phase 4: User Story 2 - Home Page Discovery Experience (Priority: P2)

**Goal**: Engaging home page with hero animation, scroll reveals, and hover effects

**Independent Test**: Navigate to home page, scroll through content, hover over cards

### Implementation for User Story 2

- [X] T032 [P] [US2] Create CountUp animation component in src/components/animations/CountUp.tsx
- [X] T033 [US2] Add hero section fade-up animation in src/pages/index.tsx
- [X] T034 [US2] Add staggered text entrance to hero in src/pages/index.tsx
- [X] T035 [US2] Add scroll reveal to HomepageFeatures in src/components/HomepageFeatures/index.tsx
- [X] T036 [US2] Add hover lift effect to feature cards in src/components/HomepageFeatures/styles.module.css
- [X] T037 [US2] Add CTA button press animation in src/css/animations.css
- [X] T038 [US2] Add CountUp animation for statistics section in src/pages/index.tsx
- [X] T039 [US2] Export CountUp from animations index in src/components/animations/index.ts

**Checkpoint**: User Story 2 complete - Home page animations polished

---

## Phase 5: User Story 3 - Navbar Navigation Experience (Priority: P2)

**Goal**: Polished navbar with consistent icons, tooltips, and mobile drawer

**Independent Test**: Hover over navbar items, open dropdowns, test keyboard navigation

### Implementation for User Story 3

- [X] T040 [P] [US3] Create NavbarIcons component in src/components/Navbar/NavbarIcons.tsx
- [X] T041 [P] [US3] Create NavbarTooltip component in src/components/Navbar/NavbarTooltip.tsx
- [X] T042 [US3] Create MobileDrawer component in src/components/Navbar/MobileDrawer.tsx
- [X] T043 [US3] Create MobileDrawer styles in src/components/Navbar/MobileDrawer.module.css
- [X] T044 [US3] Create Navbar index export in src/components/Navbar/index.ts
- [X] T045 [US3] Swizzle Navbar component and integrate icons via docusaurus swizzle
- [X] T046 [US3] Add dropdown fade/slide animation in swizzled Navbar CSS
- [X] T047 [US3] Add keyboard navigation (Tab, Escape) to MobileDrawer in MobileDrawer.tsx
- [X] T048 [US3] Add focus indicators (3:1 contrast) to navbar items in Navbar CSS

**Checkpoint**: User Story 3 complete - Navbar polished with icons and tooltips

---

## Phase 6: User Story 4 - Reduced Motion Accessibility (Priority: P2)

**Goal**: All animations respect prefers-reduced-motion system preference

**Independent Test**: Enable reduced motion preference, verify all animations disabled/simplified

### Implementation for User Story 4

- [X] T049 [US4] Add reduced motion CSS overrides for ChatPanel in src/css/reduced-motion.css
- [X] T050 [US4] Add reduced motion CSS overrides for home page animations in src/css/reduced-motion.css
- [X] T051 [US4] Add reduced motion CSS overrides for navbar animations in src/css/reduced-motion.css
- [X] T052 [US4] Update TypingIndicator to use opacity pulse when reduced motion in src/components/SkeletonLoader/TypingIndicator.tsx
- [X] T053 [US4] Add ARIA live region for chatbot state changes in ChatPanel.tsx
- [X] T054 [US4] Test all animations with prefers-reduced-motion media query

**Checkpoint**: User Story 4 complete - Full reduced motion support

---

## Phase 7: User Story 5 - Loading State Feedback (Priority: P3)

**Goal**: Clear visual feedback for loading states (skeleton, typing, progress, spinner)

**Independent Test**: Observe skeleton screens on page load, typing indicator during chat

### Implementation for User Story 5

- [X] T055 [P] [US5] Create SkeletonLoader component in src/components/SkeletonLoader/SkeletonLoader.tsx
- [X] T056 [P] [US5] Create SkeletonLoader styles in src/components/SkeletonLoader/SkeletonLoader.module.css
- [X] T057 [P] [US5] Create TypingIndicator component in src/components/SkeletonLoader/TypingIndicator.tsx
- [X] T058 [P] [US5] Create ProgressBar component in src/components/SkeletonLoader/ProgressBar.tsx
- [X] T059 [US5] Create SkeletonLoader index export in src/components/SkeletonLoader/index.ts
- [X] T060 [US5] Integrate TypingIndicator in ChatPanel during AI response in ChatPanel.tsx
- [X] T061 [US5] Add shimmer animation to SkeletonLoader in SkeletonLoader.module.css

**Checkpoint**: User Story 5 complete - Loading states provide clear feedback

---

## Phase 8: User Story 6 - Responsive Layout Adaptation (Priority: P3)

**Goal**: UI adapts smoothly to all screen sizes without horizontal overflow

**Independent Test**: Resize browser through breakpoints, verify no layout breaks

### Implementation for User Story 6

- [X] T062 [US6] Add mobile full-screen overlay for ChatPanel at <768px in ChatPanel.module.css
- [X] T063 [US6] Add responsive breakpoints to navbar in Navbar CSS
- [X] T064 [US6] Add touch target sizing (44x44px minimum) in src/css/custom.css
- [X] T065 [US6] Verify no horizontal overflow at 320px-2560px in all components
- [X] T066 [US6] Add viewport resize handling during animations in ChatPanel.tsx

**Checkpoint**: User Story 6 complete - Responsive design verified

---

## Phase 9: E2E Testing & Polish

**Purpose**: E2E journey tests and final polish

### E2E Journey Tests

- [X] T067 [P] Create journey-01-first-visit.spec.ts in tests/e2e/
- [X] T068 [P] Create journey-02-chatbot-panel.spec.ts in tests/e2e/
- [X] T069 [P] Create journey-03-navbar-navigation.spec.ts in tests/e2e/
- [X] T070 [P] Create journey-04-dark-mode-rtl.spec.ts in tests/e2e/
- [X] T071 [P] Create journey-05-reduced-motion.spec.ts in tests/e2e/
- [X] T072 [P] Create journey-06-responsive.spec.ts in tests/e2e/
- [X] T073 [P] Create journey-07-learning-session.spec.ts in tests/e2e/
- [X] T074 [P] Create journey-08-error-recovery.spec.ts in tests/e2e/
- [X] T075 [P] Create journey-09-keyboard-navigation.spec.ts in tests/e2e/
- [X] T076 [P] Create journey-10-performance.spec.ts in tests/e2e/

### Unit Tests

- [X] T077 [P] Create useReducedMotion.test.ts in tests/unit/
- [X] T078 [P] Create useChatPanelState.test.ts in tests/unit/
- [X] T079 [P] Create animation-utils.test.ts in tests/unit/

### Component Tests

- [X] T080 [P] Create ChatPanel.test.tsx in tests/component/
- [X] T081 [P] Create SkeletonLoader.test.tsx in tests/component/
- [X] T082 [P] Create NavbarIcons.test.tsx in tests/component/

### Final Polish

- [ ] T083 Run WCAG 2.2 AA accessibility audit (axe-core)
- [ ] T084 Run Lighthouse CI and verify scores > 90
- [ ] T085 Verify Core Web Vitals: LCP <= 2.5s, INP <= 200ms, CLS <= 0.1
- [ ] T086 Verify bundle size increase <= 15KB gzipped
- [X] T087 Run all E2E tests and fix failures
- [X] T088 Code cleanup and remove unused imports
- [X] T089 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately
- **Phase 2 (Foundational)**: Depends on Phase 1 - BLOCKS all user stories
- **Phases 3-8 (User Stories)**: All depend on Phase 2 completion
  - Can proceed in parallel if staffed
  - Or sequentially in priority order (P1 -> P2 -> P3)
- **Phase 9 (Polish)**: Depends on all user stories being complete

### User Story Dependencies

- **US1 (Chatbot Panel - P1)**: Can start after Phase 2 - No dependencies on other stories
- **US2 (Home Page - P2)**: Can start after Phase 2 - No dependencies on other stories
- **US3 (Navbar - P2)**: Can start after Phase 2 - No dependencies on other stories
- **US4 (Reduced Motion - P2)**: Can start after Phase 2 - Benefits from US1-US3 existing for testing
- **US5 (Loading States - P3)**: Can start after Phase 2 - Integrates with US1 (ChatPanel)
- **US6 (Responsive - P3)**: Can start after Phase 2 - Tests components from US1-US3

### Within Each User Story

- Components before integration
- Styles alongside components (same file pair)
- Index exports after all components
- Integration with existing code last

### Parallel Opportunities

**Phase 1 (All [P] tasks):**
```bash
Task: "Create global animation keyframes in src/css/animations.css"
Task: "Create reduced motion overrides in src/css/reduced-motion.css"
Task: "Create animations directory structure"
Task: "Create hooks directory structure"
```

**Phase 2 (All hooks [P]):**
```bash
Task: "Implement useReducedMotion hook"
Task: "Implement usePersistedState hook"
Task: "Implement useScrollReveal hook"
Task: "Implement useAnimationState hook"
Task: "Create FadeIn component"
Task: "Create SlideIn component"
```

**User Stories 1-3 (if parallel team):**
```bash
Developer A: US1 (Chatbot Panel) - T014-T031
Developer B: US2 (Home Page) - T032-T039
Developer C: US3 (Navbar) - T040-T048
```

**All E2E tests (T067-T076):**
```bash
# All 10 journey tests can be written in parallel
Task: "journey-01-first-visit.spec.ts"
Task: "journey-02-chatbot-panel.spec.ts"
... (all 10 parallel)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1 (Chatbot Panel)
4. **STOP and VALIDATE**: Test chatbot independently
5. Deploy/demo if ready - users can interact with polished chatbot

### Incremental Delivery

1. Setup + Foundational -> Animation foundation ready
2. Add US1 (Chatbot) -> Test -> Deploy (MVP!)
3. Add US2 (Home Page) -> Test -> Deploy
4. Add US3 (Navbar) -> Test -> Deploy
5. Add US4 (Reduced Motion) -> Test -> Deploy
6. Add US5 (Loading States) -> Test -> Deploy
7. Add US6 (Responsive) -> Test -> Deploy
8. E2E tests + Polish -> Final release

### Single Developer Strategy

Follow priority order:
1. P1: US1 (Chatbot Panel) - highest value
2. P2: US2, US3, US4 (Home Page, Navbar, Accessibility)
3. P3: US5, US6 (Loading States, Responsive)
4. E2E tests can be written alongside or after implementation

---

## Summary

| Phase | Task Count | Parallelizable | Story |
|-------|------------|----------------|-------|
| Setup | 5 | 4 | - |
| Foundational | 8 | 6 | - |
| US1 Chatbot | 18 | 2 | P1 |
| US2 Home Page | 8 | 1 | P2 |
| US3 Navbar | 9 | 2 | P2 |
| US4 Reduced Motion | 6 | 0 | P2 |
| US5 Loading States | 7 | 4 | P3 |
| US6 Responsive | 5 | 0 | P3 |
| E2E/Unit/Component Tests | 16 | 16 | - |
| Final Polish | 7 | 0 | - |
| **Total** | **89** | **35** | - |

**MVP Scope**: Phases 1-3 (31 tasks) - Delivers polished chatbot panel
**Full Scope**: All phases (89 tasks) - Complete UI enhancement

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label (US1-US6) maps to user stories for traceability
- Each user story independently testable
- Animation bundle budget: <= 15KB gzipped
- All animations must respect prefers-reduced-motion
- Focus management required for accessibility
- RTL support required for chatbot panel
