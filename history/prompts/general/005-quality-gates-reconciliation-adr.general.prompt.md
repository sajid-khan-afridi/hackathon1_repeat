---
id: 005
title: Quality Gates Reconciliation ADR
stage: general
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5
feature: none
branch: 002-docusaurus-book-infra
user: sajid-khan-afridi
command: /sp.adr
labels: ["adr", "quality-gates", "constitution", "planning-workflow", "governance"]
links:
  spec: null
  ticket: null
  adr: history/adr/003-quality-gates-reconciliation.md
  pr: null
files:
  - history/adr/003-quality-gates-reconciliation.md
  - history/prompts/general/005-quality-gates-reconciliation-adr.general.prompt.md
tests:
  - N/A (documentation work, no code changes)
---

## Prompt

/sp.adr

a lot of improvements are suggest, conflicts identified between the constitution and plan phase

## Response snapshot

**ADR-003 Created: Quality Gates and Acceptance Criteria Reconciliation**

**Key Decision**: Establish three-tier quality gate hierarchy to resolve conflicts between constitution and plan phase:

1. **Constitution Detailed Exit Criteria** (Source of Truth)
   - Location: `.specify/memory/constitution.md` lines 927-990
   - Format: Detailed checklists (e.g., "Lighthouse Performance > 85 AND Accessibility > 90")

2. **Constitution Summary** (Quick Reference Only)
   - Location: `.specify/memory/constitution.md` lines 85-92
   - Format: One-line summaries per phase
   - NOT authoritative source

3. **Plan Phase Constitution Check** (Implementation View)
   - Location: `specs/<feature>/plan.md`
   - MUST exactly match tier 1 (Constitution Detailed Exit Criteria)

**Conflicts Identified**:

1. **Lighthouse Score Mismatch**:
   - Constitution summary says: "Lighthouse scores > 85"
   - Constitution detailed says: "Performance > 85 AND Accessibility > 90"
   - Plan 002 says: "all > 85"
   - **Resolution**: Plan must match detailed (Performance 85, Accessibility 90)

2. **Test Coverage Ambiguity**:
   - Constitution principle: "80% test coverage for core functionality"
   - Phase 1 detailed criteria: No mention of coverage
   - **Resolution**: Phase 1 has no business logic, coverage N/A (clarified in ADR)

**Reconciliation Process**:
- When conflict detected: Constitution detailed criteria wins
- Agent auto-corrects plan to match Constitution
- Logs correction in PHR

**Follow-Up Actions**:
- Update plan 002 to resolve Lighthouse conflict
- Update plan template with three-tier hierarchy explanation
- Add validation to `/sp.plan` command
- Create this PHR (completed)

## Outcome

- ✅ Impact: Established single source of truth for quality gates, preventing quality drift across features
- 🧪 Tests: N/A (ADR documentation, no code)
- 📁 Files: Created ADR-003 and PHR-005
- 🔁 Next prompts: Update plan 002 to resolve conflicts, audit plan 001, update plan template
- 🧠 Reflection: Three-tier system adds clarity but requires documentation to prevent confusion. Alternative C (two-tier system) may be worth revisiting if summary tier causes issues in practice.

## Evaluation notes (flywheel)

- Failure modes observed: Constitution had two sources of quality gate definitions (summary vs detailed) with slight mismatches, causing ambiguity during plan phase. Plan templates copied summary instead of detailed criteria.
- Graders run and results (PASS/FAIL): MANUAL_REVIEW - ADR significance checklist PASS (Impact: long-term, Alternatives: 4 considered, Scope: cross-cutting all phases)
- Prompt variant (if applicable): N/A (first ADR using this workflow)
- Next experiment (smallest change to try): Monitor if three-tier hierarchy causes confusion; if yes, collapse to two-tier system (remove constitution summary)
