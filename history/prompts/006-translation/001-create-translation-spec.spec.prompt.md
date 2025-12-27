---
id: 001
title: Create Translation Spec
stage: spec
date: 2025-12-27
surface: agent
model: claude-opus-4-5-20251101
feature: 006-translation
branch: 006-translation
user: Claude Code
command: /sp.specify phase-5 Translation
labels: ["translation", "urdu", "rtl", "i18n", "phase-5"]
links:
  spec: specs/006-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-translation/spec.md
  - specs/006-translation/checklists/requirements.md
tests:
  - null
---

## Prompt

phase-5 Translation

## Response snapshot

Created Phase 5 Translation specification for the Physical AI & Humanoid Robotics Textbook project. The spec defines Urdu translation with RTL support as the core feature, aligned with constitution Phase 5 requirements.

**Key deliverables documented:**
- 4 prioritized user stories (P1-P4): Browse in Urdu, Language Persistence, Technical Glossary, Error Fallback
- 14 functional requirements covering translation, RTL layout, language toggle, glossary, caching, and accessibility
- 10 measurable success criteria with specific metrics (< 5s translation, 100% term preservation, CLS < 0.1)
- 4 key entities: Translation, Technical Term, Language Preference, Translation Cache
- Dependencies on Phase 2 content and `urdu-translator` skill
- 3 identified risks with mitigation strategies and kill switches

**Files created:**
- `specs/006-translation/spec.md` - Full feature specification
- `specs/006-translation/checklists/requirements.md` - Quality validation checklist (all items pass)

## Outcome

- Impact: Created complete Phase 5 Translation specification ready for planning phase
- Tests: N/A (specification phase)
- Files: 2 files created (spec.md, requirements.md)
- Next prompts: /sp.clarify or /sp.plan to proceed with feature development
- Reflection: Spec aligns well with constitution Phase 5 detailed requirements including RTL, glossary, caching, and accessibility

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Content Quality PASS, Requirement Completeness PASS, Feature Readiness PASS
- Prompt variant (if applicable): Standard /sp.specify command
- Next experiment (smallest change to try): None needed - spec validated successfully
