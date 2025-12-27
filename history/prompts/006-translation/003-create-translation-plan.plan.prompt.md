---
id: 003
title: Create Translation Implementation Plan
stage: plan
date: 2025-12-27
surface: agent
model: claude-opus-4-5-20251101
feature: 006-translation
branch: 006-translation
user: Claude Code
command: /sp.plan
labels: ["planning", "translation", "rtl", "i18n", "architecture"]
links:
  spec: specs/006-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-translation/plan.md
  - specs/006-translation/research.md
  - specs/006-translation/data-model.md
  - specs/006-translation/quickstart.md
  - specs/006-translation/contracts/glossary-api.md
tests:
  - null
---

## Prompt

/sp.plan

## Response snapshot

Created comprehensive implementation plan for Phase 5 Translation feature. Completed both Phase 0 (Research) and Phase 1 (Design & Contracts).

**Technical Context Established:**
- TypeScript 5.x frontend, Python 3.11+ build scripts
- Docusaurus v3 with native i18n support
- Build-time translation generation (not runtime)
- Static file storage for translations, Neon Postgres for user preferences

**Phase 0 Research Completed:**
- Docusaurus i18n integration pattern
- RTL layout with CSS logical properties
- urdu-translator skill integration with term protection
- JSON glossary storage strategy
- React Context for language state
- Playwright visual regression testing
- MD5 content hashing for cache invalidation
- WCAG 2.1 AA accessibility implementation

**Phase 1 Design Artifacts Generated:**
- `data-model.md`: 4 entities (GlossaryTerm, TranslationCache, LanguagePreference, GlossaryCategory)
- `quickstart.md`: Local development setup guide
- `contracts/glossary-api.md`: Static JSON API contract

**Constitution Compliance:**
- All 6 Phase 5 quality gates documented
- Core principles alignment verified (Quality, YAGNI, Security, Accessibility, Free Tier)
- Agent ownership: TranslationService Agent (primary)

**ADR Detected:**
- Build-time translation vs runtime translation service
- Recommended: `/sp.adr build-time-translation-strategy`

## Outcome

- Impact: Complete implementation plan ready for task generation
- Tests: N/A (planning phase)
- Files: 5 files created (plan.md, research.md, data-model.md, quickstart.md, glossary-api.md)
- Next prompts: /sp.tasks to generate implementation tasks, optionally /sp.adr for build-time translation decision
- Reflection: Build-time translation is optimal for free tier sustainability and simplifies architecture

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution Check PASS, Research PASS, Design PASS
- Prompt variant (if applicable): Standard /sp.plan command
- Next experiment (smallest change to try): None needed - plan complete
