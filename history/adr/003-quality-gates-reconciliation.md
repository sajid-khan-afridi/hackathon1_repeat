# ADR-003: Quality Gates and Acceptance Criteria Reconciliation

> **Scope**: Standardize quality gate definitions across constitution summary, detailed exit criteria, and plan phase templates to eliminate conflicts and ensure consistent enforcement.

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** Project Infrastructure (applies to all phases)
- **Context:** Cross-phase quality assurance workflow

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Long-term consequence affecting all development phases and quality enforcement
     2) Alternatives: YES - Multiple approaches to defining quality gates (summary vs detailed, constitution vs plan)
     3) Scope: YES - Cross-cutting concern affecting all agents and all phases
-->

## Decision

**Establish a three-tier quality gate hierarchy with explicit precedence and reconciliation rules:**

1. **Constitution Detailed Exit Criteria (Source of Truth)**
   - Location: `.specify/memory/constitution.md` Section "QUALITY GATES" (lines 927-990)
   - Format: Detailed checklists with specific metrics
   - Example: Phase 1 has 7 criteria including "Lighthouse Performance > 85" AND "Lighthouse Accessibility > 90"

2. **Constitution Summary Quality Gates (High-Level Reference)**
   - Location: `.specify/memory/constitution.md` Section "Core Principles > Quality Over Speed" (lines 85-92)
   - Format: Concise one-line summaries per phase
   - Example: Phase 1 summarized as "Lighthouse scores > 85, build success, dark mode functional"
   - **Purpose**: Quick reference only; NOT definitive source

3. **Plan Phase Constitution Check (Implementation View)**
   - Location: `specs/<feature>/plan.md` Section "Constitution Check"
   - Format: Copy of relevant detailed exit criteria from Constitution
   - **Rule**: MUST exactly match Constitution Detailed Exit Criteria; any deviation is a conflict

**Reconciliation Process:**
- When conflicts detected between tiers 1 and 3, Constitution Detailed Exit Criteria (tier 1) wins
- Tier 2 summaries are informational only and may omit details for brevity
- Plan phase templates MUST reference Constitution by line number to prevent drift

**Conflict Resolution Workflow:**
1. Agent detects mismatch during plan phase constitution check
2. Agent reports: "Quality gate conflict detected: Plan says X, Constitution detailed criteria says Y"
3. Agent auto-corrects plan to match Constitution detailed criteria
4. Agent logs correction in PHR with explanation

## Consequences

### Positive

- **Single Source of Truth**: Constitution Detailed Exit Criteria is unambiguous authority
- **Automated Conflict Detection**: Plan templates can programmatically validate against Constitution
- **Consistency Across Features**: All plan phases use identical quality gates for same phase number
- **Audit Trail**: PHRs capture any reconciliation performed during planning
- **Prevents Quality Erosion**: Cannot accidentally lower standards by modifying plan templates
- **Clear Escalation**: When Constitution needs updating, explicit governance process required

### Negative

- **Initial Reconciliation Effort**: Must audit existing plans (001, 002) and update to match Constitution
- **Maintenance Overhead**: Constitution changes require cascading updates to all plan templates
- **Potential Confusion**: Three tiers may seem redundant to newcomers
- **Template Verbosity**: Plan constitution check sections will be longer (copying detailed criteria)
- **Risk of Stale References**: If Constitution line numbers change, plan templates may reference wrong sections

### Mitigations for Negatives

- **One-Time Audit**: Use grep to find all plan.md files and validate quality gates (automated script possible)
- **Template Automation**: Use `/sp.plan` command to auto-populate constitution check from source
- **Documentation**: Add to `.specify/templates/plan-template.md` explaining three-tier hierarchy
- **Validation Script**: CI job to validate plan quality gates match Constitution (optional enhancement)

## Alternatives Considered

### Alternative A: Plan Phase as Source of Truth
- **Description**: Each feature's plan.md defines its own quality gates, Constitution provides guidance only
- **Why Rejected**:
  - Allows quality drift across features (Feature A has strict gates, Feature B relaxes them)
  - No consistency enforcement
  - Violates "Quality Over Speed" principle (agents could lower standards)
  - Makes cross-phase quality comparisons impossible

### Alternative B: Flat Single-Tier System
- **Description**: Constitution has ONLY summary quality gates (like tier 2), plans define details
- **Why Rejected**:
  - Ambiguity in what "Lighthouse > 85" means (Performance? Accessibility? All scores?)
  - Plans become source of truth by necessity, leading to drift (same as Alternative A)
  - Constitution loses enforceability

### Alternative C: Two-Tier System (Remove Constitution Summary)
- **Description**: Eliminate tier 2 (summary gates in "Quality Over Speed" section), keep only detailed exit criteria
- **Why Considered**: Reduces redundancy, single source is clearest
- **Why Rejected for Now**:
  - Summary is helpful for quick reference when reading constitution principles
  - Can be kept as informational if clearly labeled "Summary - see QUALITY GATES for details"
  - Low cost to maintain (8 lines total across 6 phases)
  - **Future Option**: May revisit if summaries cause confusion in practice

### Alternative D: Dynamic References (Programmatic Lookup)
- **Description**: Plan templates don't copy quality gates; instead reference Constitution via script that injects content
- **Example**: `<!-- INJECT: Constitution Phase 1 Exit Criteria -->`
- **Why Rejected**:
  - Requires custom tooling (script to process templates)
  - Harder to review plans (must run script to see criteria)
  - Adds complexity to plan authoring workflow
  - GitHub markdown preview wouldn't show injected content
  - **Future Option**: Could implement as `/sp.plan` command enhancement if manual copying becomes burden

## Identified Conflicts (Current Codebase)

### Conflict 1: Phase 1 Lighthouse Scores
- **Constitution Summary (Tier 2)**: "Lighthouse scores > 85" (line 86)
- **Constitution Detailed (Tier 1)**: "Lighthouse Performance > 85" AND "Lighthouse Accessibility > 90" (lines 931-932)
- **Plan 002 (Tier 3)**: "Lighthouse scores all > 85" (line 139 in plan.md)
- **Resolution**: Plan 002 MUST change to match tier 1 (Performance > 85, Accessibility > 90)
- **Impact**: Accessibility bar is higher than initially stated in plan

### Conflict 2: PHR Requirement Placement
- **Constitution Detailed (Tier 1)**: "PHR created documenting infrastructure decisions" (line 935)
- **Plan 002 (Tier 3)**: Missing this criterion in initial draft
- **Resolution**: Plan 002 added PHR criterion (line 36)
- **Status**: Already reconciled in current plan.md

### Conflict 3: Test Coverage Ambiguity
- **Constitution Summary (Tier 2)**: "build success" (line 86)
- **Constitution Detailed (Tier 1)**: "Docusaurus builds successfully (`npm run build`)" (line 929) - no mention of test coverage for Phase 1
- **Constitution Principles**: "Test coverage minimum: 80% for all core functionality" (line 78)
- **Resolution**: Phase 1 has NO core business logic (static site setup only), so 80% coverage applies to future phases when logic added
- **Clarification**: Add to Phase 1 exit criteria: "Test coverage: N/A (no business logic in Phase 1)"

## Implementation Plan

### Immediate Actions (This ADR)
1. ✅ Document three-tier hierarchy and precedence rules
2. ✅ Identify conflicts in existing plans (001, 002)
3. ⏳ Update plan 002 to resolve Conflict 1 (Lighthouse scores)
4. ⏳ Update plan 002 to add Conflict 3 clarification (test coverage N/A)

### Follow-Up Actions (Next Sprint)
1. Audit plan 001 for quality gate conflicts
2. Update `.specify/templates/plan-template.md` to include:
   - Comment explaining three-tier system
   - Instruction to copy exact wording from Constitution Detailed Exit Criteria
   - Line number reference format: `(Constitution lines 927-935)`
3. Add validation step to `/sp.plan` command: compare plan quality gates to Constitution
4. Create PHR documenting this ADR (per constitution requirement)

### Long-Term Enhancements (Optional)
- CI script to validate plan quality gates match Constitution (GitHub Actions)
- `/sp.plan` command auto-populates constitution check from source file (reduces manual copying)
- Quality gate dashboard showing compliance across all features

## References

- Feature Spec: N/A (project-wide infrastructure decision)
- Implementation Plan: `specs/002-docusaurus-book-infra/plan.md` (example affected by this ADR)
- Related ADRs: None (first ADR addressing planning workflow)
- Constitution: `.specify/memory/constitution.md` (lines 85-92 summary, lines 927-990 detailed exit criteria)
- Plan Template: `.specify/templates/plan-template.md` (to be updated per this ADR)

## Acceptance Criteria for This ADR

- [ ] ADR file created at `history/adr/003-quality-gates-reconciliation.md`
- [ ] Plan 002 updated to resolve Lighthouse score conflict (Performance > 85, Accessibility > 90)
- [ ] Plan 002 updated to clarify test coverage N/A for Phase 1
- [ ] PHR created documenting this ADR work
- [ ] Constitution summary (tier 2) labeled as "Summary - see QUALITY GATES section for complete criteria" to prevent future confusion
