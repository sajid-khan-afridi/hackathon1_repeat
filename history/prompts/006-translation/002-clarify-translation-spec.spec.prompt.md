---
id: 002
title: Clarify Translation Spec
stage: spec
date: 2025-12-27
surface: agent
model: claude-opus-4-5-20251101
feature: 006-translation
branch: 006-translation
user: Claude Code
command: /sp.clarify
labels: ["clarification", "translation", "caching", "build-time"]
links:
  spec: specs/006-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-translation/spec.md
tests:
  - null
---

## Prompt

/sp.clarify

## Response snapshot

Performed structured ambiguity scan on Phase 5 Translation spec across 10 taxonomy categories. Identified 2 high-impact clarification questions that materially affect architecture and task decomposition.

**Questions Asked & Resolved:**

1. **Cache Expiry Strategy**: When should cached translations expire or be invalidated?
   - Answer: Never expire; invalidate only on source content change
   - Impact: Updated FR-010, Translation Cache entity definition

2. **Translation Timing Strategy**: When should translations be generated (build-time vs on-demand)?
   - Answer: Pre-translate all chapters at build/deploy time
   - Impact: Added FR-015, updated SC-002 to specify build-time context

**Spec Updates:**
- Added `## Clarifications` section with `### Session 2025-12-27`
- Updated FR-010: Cache invalidation based on content hash comparison
- Added FR-015: Build-time translation generation requirement
- Updated SC-002: Clarified 5-second metric applies to build/deploy time
- Updated Translation Cache entity: Added source content hash, removed expiry field

**Coverage Summary:**
- All 10 taxonomy categories now at Clear status
- No Outstanding or Deferred items

## Outcome

- Impact: Resolved 2 architectural ambiguities affecting caching and deployment strategy
- Tests: N/A (specification phase)
- Files: 1 file updated (spec.md)
- Next prompts: /sp.plan to begin architecture planning
- Reflection: Build-time translation aligns with free tier sustainability and eliminates runtime API dependency

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Clarification integration PASS, Spec validation PASS
- Prompt variant (if applicable): Standard /sp.clarify command
- Next experiment (smallest change to try): None needed - clarification complete
