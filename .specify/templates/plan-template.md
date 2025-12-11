# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

[Extract from feature spec: primary requirement + technical approach from research]

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: [e.g., Python 3.11, Swift 5.9, Rust 1.75 or NEEDS CLARIFICATION]  
**Primary Dependencies**: [e.g., FastAPI, UIKit, LLVM or NEEDS CLARIFICATION]  
**Storage**: [if applicable, e.g., PostgreSQL, CoreData, files or N/A]  
**Testing**: [e.g., pytest, XCTest, cargo test or NEEDS CLARIFICATION]  
**Target Platform**: [e.g., Linux server, iOS 15+, WASM or NEEDS CLARIFICATION]
**Project Type**: [single/web/mobile - determines source structure]  
**Performance Goals**: [domain-specific, e.g., 1000 req/s, 10k lines/sec, 60 fps or NEEDS CLARIFICATION]  
**Constraints**: [domain-specific, e.g., <200ms p95, <100MB memory, offline-capable or NEEDS CLARIFICATION]  
**Scale/Scope**: [domain-specific, e.g., 10k users, 1M LOC, 50 screens or NEEDS CLARIFICATION]

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

<!--
  QUALITY GATES THREE-TIER HIERARCHY (per ADR-003):

  Tier 1: Constitution Detailed Exit Criteria (SOURCE OF TRUTH)
    Location: .specify/memory/constitution.md Section "QUALITY GATES"
    Reference: Lines 927-990 (Phase 1-6 Exit Criteria)

  Tier 2: Constitution Summary (INFORMATIONAL ONLY)
    Location: .specify/memory/constitution.md Section "Core Principles > Quality Over Speed"
    Reference: Lines 85-92
    Note: High-level summaries; NOT authoritative source

  Tier 3: Plan Phase Constitution Check (IMPLEMENTATION VIEW - THIS FILE)
    Rule: MUST exactly match Tier 1 (Constitution Detailed Exit Criteria)
    Any deviation is a conflict; Tier 1 always wins

  INSTRUCTIONS:
  1. Copy the EXACT checklist from Constitution Detailed Exit Criteria for your phase
  2. Paste below under "Phase [N] Quality Gates (from Constitution)"
  3. Add reference comment: (Constitution lines XXX-YYY)
  4. Do NOT summarize or paraphrase; use exact wording
  5. If conflict detected, auto-correct to match Constitution and log in PHR
-->

### Phase [N] Quality Gates (from Constitution)
<!-- Reference: .specify/memory/constitution.md lines XXX-YYY -->
- [ ] [Copy exact checklist from Constitution Detailed Exit Criteria]
- [ ] [Example: Docusaurus builds successfully (`npm run build`)]
- [ ] [Example: Lighthouse Performance > 85]
- [ ] [Example: Lighthouse Accessibility > 90]
- [ ] [Example: Test coverage: N/A (no business logic in Phase 1) OR 80% for core functionality]
- [ ] [Example: PHR created documenting infrastructure decisions]

### Core Principles Alignment

**✅ Quality Over Speed**: [How this phase enforces quality gates and testing]
**✅ Smallest Viable Change**: [How this phase avoids over-engineering]
**✅ Security by Default**: [Security measures for this phase]
**✅ Observability & Measurability**: [How success criteria are measurable]
**✅ Accessibility & Inclusivity**: [Accessibility compliance for this phase]
**✅ Free Tier Sustainability**: [How this phase respects free tier limits]

### Agent Ownership
**Primary**: [Agent name] (owns: [list of skills])
**Support**: [Supporting agents if any]
**Coordinator**: Orchestrator Agent (quality gate approval)

### Complexity Violations
*[None identified - Phase follows YAGNI principle] OR [Document violations with justification]*

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
# [REMOVE IF UNUSED] Option 1: Single project (DEFAULT)
src/
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# [REMOVE IF UNUSED] Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

# [REMOVE IF UNUSED] Option 3: Mobile + API (when "iOS/Android" detected)
api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure: feature modules, UI flows, platform tests]
```

**Structure Decision**: [Document the selected structure and reference the real
directories captured above]

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Post-Design Constitution Check (Phase [N] Complete)

### ✅ All Quality Gates Remain Valid
<!-- Copy same checklist from Constitution Check above -->
- [ ] [Same as Phase [N] Quality Gates above]
- [ ] [Confirms no drift during design phase]

### ✅ Design Artifacts Generated
- ✅ `research.md` - [Brief description of research findings]
- ✅ `data-model.md` - [Brief description of entities defined]
- ✅ `quickstart.md` - [Brief description of setup guide]
- ⏳ `contracts/` - [N/A for static phases OR list of API contracts defined]

### ✅ No New Complexity Violations

**Analysis**:
- **YAGNI Compliance**: [How design avoids over-engineering]
- **Security**: [Security measures confirmed]
- **Performance**: [Performance strategy confirmed]
- **Accessibility**: [Accessibility compliance confirmed]
- **Free Tier**: [Free tier sustainability confirmed]

### ✅ Recommended ADRs

**[If architectural decisions were made during planning, list them here]**

**Decision Detected**: [Brief title]
- **Impact**: [Long-term consequences]
- **Alternatives**: [Other options considered]
- **Scope**: [Cross-cutting nature]

📋 Architectural decision detected: **[decision title]**
   Document reasoning and tradeoffs? Run `/sp.adr [decision-slug]`

*Waiting for user consent before creating ADRs.*

---
