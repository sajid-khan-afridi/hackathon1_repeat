---
id: 004
title: Create ADR Build-Time Translation
stage: plan
date: 2025-12-27
surface: agent
model: claude-opus-4-5-20251101
feature: 006-translation
branch: 006-translation
user: Claude Code
command: /sp.adr build-time-translation-strategy
labels: ["adr", "translation", "architecture", "build-time", "caching"]
links:
  spec: specs/006-translation/spec.md
  ticket: null
  adr: history/adr/013-build-time-translation-strategy.md
  pr: null
files:
  - history/adr/013-build-time-translation-strategy.md
tests:
  - null
---

## Prompt

/sp.adr build-time-translation-strategy

## Response snapshot

Created ADR-013: Build-Time Translation Strategy documenting the architectural decision to generate all Urdu translations at build/deploy time rather than runtime.

**Decision Cluster Components:**
- Timing: CI/CD pipeline translation during build
- Storage: Static MDX files in Docusaurus i18n directory
- Caching: Indefinite cache with content hash invalidation
- Security: API keys in CI/CD only, never in client
- Deployment: Pre-translated content to GitHub Pages
- Updates: Rebuild on content changes

**Significance Test Results:**
- Impact: ✅ Affects deployment, caching, security architecture
- Alternatives: ✅ 3 alternatives analyzed (on-demand, hybrid, external service)
- Scope: ✅ Cross-cutting (CI/CD, frontend, caching, security)

**Alternatives Documented:**
1. On-demand runtime translation - rejected (latency, cost, security)
2. Hybrid pre-translate + on-demand - rejected (complexity, inconsistent UX)
3. External translation service - rejected (less control, external dependency)

**Acceptance Criteria Grader: PASS**
- ✅ Documents decision affecting multiple components (CI/CD, frontend, deployment)
- ✅ Lists 3 alternative approaches with rationale
- ✅ Clear pros (8) and cons (5) for chosen approach
- ✅ Concise but detailed for future reference

## Outcome

- Impact: Documented architectural decision for Phase 5 Translation
- Tests: N/A (documentation phase)
- Files: 1 ADR created (013-build-time-translation-strategy.md)
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: Build-time translation is well-justified for free tier sustainability and simplicity

## Evaluation notes (flywheel)

- Failure modes observed: None - decision is appropriately clustered
- Graders run and results (PASS/FAIL): Significance Test PASS, Acceptance Criteria PASS
- Prompt variant (if applicable): Standard /sp.adr command with decision title
- Next experiment (smallest change to try): None needed - ADR complete
