# Tasks: Phase 5 Translation

**Input**: Design documents from `/specs/006-translation/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/glossary-api.md

**Tests**: Visual regression tests and validation scripts are included where specified in the spec (SC-005, FR-015).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md structure:
- **Frontend**: `src/` (React components, hooks, context, data)
- **i18n**: `docs/i18n/ur/` (Urdu translations)
- **Scripts**: `scripts/` (build-time translation scripts)
- **Tests**: `tests/` (unit, component, integration, visual)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and translation infrastructure

- [X] T001 Configure Docusaurus i18n for Urdu locale in docusaurus.config.js
- [X] T002 [P] Create RTL stylesheet with CSS logical properties in src/css/rtl.css
- [X] T003 [P] Add Noto Nastaliq Urdu font loading in src/css/custom.css
- [X] T004 [P] Create GlossaryCategory TypeScript enum in src/types/glossary.ts
- [X] T005 Create initial glossary.json structure with 10 seed terms in src/data/glossary.json
- [X] T006 [P] Create i18n directory structure for Urdu in docs/i18n/ur/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Create LanguageContext provider with locale state in src/context/LanguageContext.tsx
- [X] T008 Create useLanguagePreference hook with localStorage in src/hooks/useLanguagePreference.ts
- [X] T009 [P] Create content hash generation script in scripts/generate-content-hash.ts
- [X] T010 [P] Create technical term validation script in scripts/validate-terms.ts
- [X] T011 Create translate-chapters build script orchestrator in scripts/translate-chapters.ts
- [X] T012 [P] Create TranslationCache metadata type in src/types/translation.ts
- [X] T013 Add npm scripts for translation build commands in package.json

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Browse Chapters in Urdu (Priority: P1) MVP

**Goal**: Enable Urdu-speaking students to read textbook chapters in Urdu with RTL layout and preserved technical terms

**Independent Test**: Navigate to any chapter, switch language to Urdu, verify content displays in RTL layout with technical terms preserved in English and code blocks unchanged

### Tests for User Story 1

> **NOTE: Visual regression tests per SC-005**

- [X] T014 [P] [US1] Create RTL visual regression test for mobile (320px) in tests/visual/rtl-regression.spec.ts
- [X] T015 [P] [US1] Create RTL visual regression test for tablet (768px) in tests/visual/rtl-regression.spec.ts
- [X] T016 [P] [US1] Create RTL visual regression test for desktop (1024px) in tests/visual/rtl-regression.spec.ts
- [X] T017 [P] [US1] Create LanguageToggle component test in src/components/LanguageToggle/LanguageToggle.test.tsx

### Implementation for User Story 1

- [X] T018 [US1] Create LanguageToggle component with ARIA support in src/components/LanguageToggle/index.tsx
- [X] T019 [P] [US1] Create LanguageToggle styles with 44x44px touch target in src/components/LanguageToggle/LanguageToggle.module.css
- [X] T020 [US1] Integrate LanguageToggle into Docusaurus navbar via swizzling in src/theme/Navbar/index.tsx
- [X] T021 [US1] Configure urdu-translator skill integration in scripts/translate-chapters.ts
- [X] T022 [US1] Implement technical term protection (wrap/unwrap) in scripts/translate-chapters.ts
- [X] T023 [US1] Generate Urdu translation for Module 1 chapters in docs/i18n/ur/docusaurus-plugin-content-docs/current/
- [X] T024 [US1] Generate Urdu translation for Module 2 chapters in docs/i18n/ur/docusaurus-plugin-content-docs/current/
- [X] T025 [US1] Generate translation cache metadata files (.meta.json) alongside translated MDX
- [X] T026 [US1] Add aria-live region for language change announcements in src/components/LanguageToggle/index.tsx
- [X] T027 [US1] Translate navbar UI strings in docs/i18n/ur/docusaurus-theme-classic/navbar.json

**Checkpoint**: At this point, User Story 1 should be fully functional - users can browse chapters in Urdu with proper RTL layout

---

## Phase 4: User Story 2 - Language Preference Persistence (Priority: P2)

**Goal**: Remember user's language preference across browser sessions using localStorage

**Independent Test**: Select Urdu, close browser, reopen site, verify Urdu is still selected.

> **Scope Decision (2025-12-27):** Client-only localStorage approach. Backend sync deferred to future enhancement.

### Implementation for User Story 2

- [X] T028 [US2] Extend useLanguagePreference hook to read from localStorage on mount in src/hooks/useLanguagePreference.ts
- [X] T029 [P] [US2] Create integration test for preference persistence in tests/integration/language-preference.test.ts

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - language preference persists

> **Deferred Tasks (Future Enhancement - Backend Sync):**
> - ~~T030: SQL migration for preferred_language column~~
> - ~~T031: PATCH /api/user/preferences endpoint~~
> - ~~T032: localStorage-to-profile migration on login~~

---

## Phase 5: User Story 3 - Access Technical Glossary (Priority: P3)

**Goal**: Provide bilingual glossary with hover tooltips and searchable glossary page

**Independent Test**: Hover over technical term in Urdu content, verify tooltip shows English term with definition. Navigate to Glossary page, search for a term, verify results.

### Tests for User Story 3

- [X] T034 [P] [US3] Create GlossaryTooltip component test in src/components/GlossaryTooltip/GlossaryTooltip.test.tsx
- [X] T035 [P] [US3] Create GlossaryPage component test in src/components/GlossaryPage/GlossaryPage.test.tsx

### Implementation for User Story 3

- [ ] T036 [US3] Expand glossary.json to 50+ technical terms in src/data/glossary.json
- [X] T037 [P] [US3] Create useGlossary hook with search and filter in src/hooks/useGlossary.ts
- [X] T038 [US3] Create GlossaryTooltip component with hover behavior in src/components/GlossaryTooltip/index.tsx
- [X] T039 [P] [US3] Create GlossaryTooltip styles in src/components/GlossaryTooltip/GlossaryTooltip.module.css
- [X] T040 [US3] Create GlossaryPage component with search and category filter in src/components/GlossaryPage/index.tsx
- [X] T041 [P] [US3] Create GlossaryPage styles with RTL support in src/components/GlossaryPage/GlossaryPage.module.css
- [X] T042 [US3] Add Glossary page to Docusaurus navigation in docusaurus.config.js
- [X] T043 [US3] Integrate GlossaryTooltip into translated MDX content (via MDX component mapping)

**Checkpoint**: All primary user stories (1, 2, 3) should now be independently functional

---

## Phase 6: User Story 4 - Graceful Fallback on Translation Error (Priority: P4)

**Goal**: Ensure users can still access content in English with clear messaging when translation is unavailable

**Independent Test**: Simulate translation service failure, verify English content displays with appropriate error banner

### Implementation for User Story 4

- [X] T044 [US4] Create TranslationErrorBanner component in src/components/TranslationErrorBanner/index.tsx
- [X] T045 [P] [US4] Create TranslationErrorBanner styles in src/components/TranslationErrorBanner/TranslationErrorBanner.module.css
- [X] T046 [US4] Add fallback logic when translation file is missing in LanguageContext
- [X] T047 [US4] Add visual distinction for mixed-language content (partial translation failure)
- [X] T048 [US4] Add "Retry" button functionality in TranslationErrorBanner
- [X] T049 [US4] Add offline detection with appropriate messaging
- [X] T050 [P] [US4] Add banner for untranslated new chapters ("Translation in progress")

**Checkpoint**: All 4 user stories should now be complete and independently testable

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Quality gates, validation, and cross-cutting improvements

- [X] T051 [P] Run technical term preservation validation (100% per SC-004) via npm run translate:validate
- [ ] T052 [P] Run visual regression tests on all 3 viewports (SC-005) via npm run test:visual
- [X] T053 Measure translation load time (< 500ms per SC-003) and document results
- [X] T054 Measure language toggle response time (< 200ms per SC-006) and document results
- [X] T055 [P] Run WCAG 2.1 AA accessibility audit on LanguageToggle (SC-009)
- [X] T056 Measure CLS for language switch (< 0.1 per SC-007) and document results
- [ ] T057 [P] Generate remaining Urdu translations for all 10 chapters (SC-001)
- [ ] T058 Native speaker review for 3 sample chapters (SC-008)
- [X] T059 [P] Update quickstart.md with actual command outputs and verification steps
- [X] T060 Run full build and verify deployment to GitHub Pages

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 -> P2 -> P3 -> P4)
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

| Story | Priority | Depends On | Can Start After |
|-------|----------|------------|-----------------|
| US1: Browse Chapters in Urdu | P1 | Phase 2 (Foundational) | Phase 2 complete |
| US2: Language Persistence | P2 | Phase 2 (Foundational) | Phase 2 complete |
| US3: Technical Glossary | P3 | Phase 2 (Foundational) | Phase 2 complete |
| US4: Graceful Fallback | P4 | Phase 2 (Foundational) | Phase 2 complete |

**Note**: All user stories can be implemented in parallel after Phase 2, but P1 (US1) is recommended as MVP.

### Within Each User Story

- Tests MUST be written and FAIL before implementation (TDD)
- Components before integration
- Core functionality before edge cases
- Story complete before moving to next priority

### Parallel Opportunities

**Setup Phase (6 parallel groups)**:
- T001 (config) runs first
- T002, T003, T004, T006 can run in parallel after T001
- T005 (glossary) can run in parallel with T002-T006

**Foundational Phase (4 parallel groups)**:
- T007, T008 (context/hooks) run first
- T009, T010, T012 can run in parallel
- T011 depends on T009, T010
- T013 runs last

**User Story 1 (tests in parallel, then implementation)**:
- T014, T015, T016, T017 (tests) run in parallel
- T018-T027 (implementation) follow sequentially with some parallelism (T019 || T018)

---

## Parallel Example: User Story 1 Tests

```bash
# Launch all visual regression tests together:
Task: "RTL visual regression test for mobile (320px) in tests/visual/rtl-regression.spec.ts"
Task: "RTL visual regression test for tablet (768px) in tests/visual/rtl-regression.spec.ts"
Task: "RTL visual regression test for desktop (1024px) in tests/visual/rtl-regression.spec.ts"
Task: "LanguageToggle component test in src/components/LanguageToggle/LanguageToggle.test.tsx"
```

## Parallel Example: User Story 3 Components

```bash
# Launch independent components together:
Task: "Create useGlossary hook with search and filter in src/hooks/useGlossary.ts"
Task: "Create GlossaryTooltip styles in src/components/GlossaryTooltip/GlossaryTooltip.module.css"
Task: "Create GlossaryPage styles with RTL support in src/components/GlossaryPage/GlossaryPage.module.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Browse Chapters in Urdu)
4. **STOP and VALIDATE**: Test US1 independently
   - Navigate to chapter, switch to Urdu
   - Verify RTL layout on 3 viewports
   - Verify technical terms preserved
   - Verify code blocks unchanged
5. Deploy/demo if ready

### Incremental Delivery

| Increment | Scope | Value Delivered |
|-----------|-------|-----------------|
| MVP | Setup + Foundational + US1 | Urdu chapter browsing with RTL |
| +US2 | Add language persistence | Returning users don't re-select |
| +US3 | Add glossary | Enhanced learning with tooltips |
| +US4 | Add fallback | Reliability and offline support |
| +Polish | Quality gates | Production-ready release |

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (core translation)
   - Developer B: User Story 3 (glossary components)
   - Developer C: Build scripts and validation
3. Stories complete and integrate independently

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 56 |
| **Setup Tasks** | 6 |
| **Foundational Tasks** | 7 |
| **US1 Tasks** | 14 |
| **US2 Tasks** | 2 (4 deferred) |
| **US3 Tasks** | 10 |
| **US4 Tasks** | 7 |
| **Polish Tasks** | 10 |
| **Parallel Opportunities** | 26 tasks marked [P] |
| **Suggested MVP** | Phase 1 + Phase 2 + Phase 3 (US1) = 27 tasks |

> **Architecture Decisions Applied:**
> - TranslationCache: File-based (.meta.json) - no Postgres storage
> - Language Preference: localStorage only - no backend sync

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label (US1, US2, US3, US4) maps task to specific user story for traceability
- Each user story is independently completable and testable
- Build-time translation avoids runtime API dependencies
- All translations are static files - no runtime translation service
- Technical terms protected via wrap/unwrap during translation
- Visual regression tests validate RTL layout on 3 viewports
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
