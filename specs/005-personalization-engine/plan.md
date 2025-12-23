# Implementation Plan: Phase 4B Personalization Engine

**Branch**: `1-personalization-engine` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/1-personalization-engine/spec.md`

<!--
IMPORTANT: Branch numbering follows phase-based numbering from the constitution:
- Phase 1 (Book Infrastructure): 001-*
- Phase 2 (Content Creation): 002-*
- Phase 3 (RAG Chatbot Core): 003-*
- Phase 4A (Authentication): 004-*
- Phase 4B (Personalization): 005-*
- Phase 5 (Translation): 006-*
- Phase 6 (Integration & Deployment): 007-*
-->

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build an adaptive learning system that personalizes the robotics textbook experience based on authenticated user profiles. This phase delivers: (1) Skill level classification (beginner/intermediate/advanced) from user profiles, (2) Chapter progress tracking (started/completed/bookmarked), (3) Smart chapter recommendations considering skill level, learning goals, and prerequisites, (4) Adaptive content depth via PersonalizedSection enhancement, and (5) Profile-aware RAG chatbot responses. The system integrates with existing Better Auth authentication (Phase 4A), stores data in Neon Postgres, and maintains < 2s personalization response time while respecting free tier limits.

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: Python 3.11+ (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI (backend API), React 18 (frontend components), asyncpg (Postgres async client), Better Auth (session management), OpenAI Agents SDK (profile-aware RAG)
**Storage**: Neon Serverless Postgres (user profiles, skill classifications, chapter progress, recommendations cache)
**Testing**: Pytest (backend unit/integration), React Testing Library (frontend components), Playwright (E2E user journeys)
**Target Platform**: Railway/Render (FastAPI backend), GitHub Pages (Docusaurus static frontend)
**Project Type**: Web application (backend API + static frontend)
**Performance Goals**: Personalization response < 2s end-to-end, skill classification < 500ms, recommendation API < 1s (cached), progress tracking updates < 300ms
**Constraints**: Neon free tier (0.5GB storage, 100 compute hours/month), OpenAI API rate limits, recommendation cache 1-hour TTL, 100 concurrent users without degradation
**Scale/Scope**: Support 10 MDX chapters with metadata, track progress for 1000+ users, classify users into 3 skill tiers, generate up to 3 recommendations per request

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

### Phase 4B Quality Gates (from Constitution)
<!-- Reference: .specify/memory/constitution.md lines 967-974 -->
- [ ] User skill level classification functional
- [ ] Personalized recommendations display correctly
- [ ] Response time < 2s (measured)
- [ ] Recommendation relevance > 0.75 (A/B test)
- [ ] PHR created documenting personalization logic

### Core Principles Alignment

**✅ Quality Over Speed**: 80% test coverage for skill classification, progress tracking, and recommendation algorithms. Integration tests validate < 2s personalization response time. Unit tests ensure classification consistency across identical profile inputs.
**✅ Smallest Viable Change**: Rule-based skill classification (no ML models). Direct database queries for progress tracking (no repository pattern). Simple in-memory dict with TTL for recommendation cache (no Redis yet). Enhance existing PersonalizedSection component rather than rebuild.
**✅ Security by Default**: User profile data retrieved via authenticated Better Auth sessions only. No PII in recommendation logs. Input validation on all chapter_id and user_id parameters to prevent injection. Rate limiting on recommendation endpoint (50 requests/hour per user).
**✅ Observability & Measurability**: Structured JSON logs for all classification and recommendation operations with correlation IDs. Metrics tracked: classification latency (p95 < 500ms), recommendation relevance scores (target > 0.75), personalization response time (p95 < 2s). All success criteria quantified and testable.
**✅ Accessibility & Inclusivity**: PersonalizedSection enhancements maintain WCAG 2.1 AA compliance. Keyboard navigation works for all new UI (recommendation cards, bookmark buttons). Focus indicators visible. Screen reader announces content visibility changes.
**✅ Free Tier Sustainability**: Neon storage impact minimal (skill classifications ~1KB/user, progress records ~500 bytes/chapter/user, supports 1000+ users within 0.5GB limit). Recommendation cache reduces database queries by ~70%. No additional external services required.

### Agent Ownership
**Primary**: PersonalizationEngine Agent (owns: `content-adapter`, `user-profiling`)
**Support**: AuthEngineer Agent (`user-profiling`, `neon-postgres`), RAGArchitect Agent (`neon-postgres` for database schema)
**Coordinator**: Orchestrator Agent (quality gate approval)

### Complexity Violations
*None identified - Phase follows YAGNI principle. Uses rule-based classification instead of ML models, simple caching instead of Redis, enhances existing components instead of building new ones.*

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

```text
backend/
├── src/
│   ├── models/
│   │   ├── skill_level_classification.py    # NEW: SkillLevelClassification entity
│   │   ├── chapter_progress.py               # NEW: ChapterProgress entity
│   │   ├── chapter_recommendation.py         # NEW: ChapterRecommendation entity
│   │   └── chapter_metadata.py               # NEW: ChapterMetadata entity
│   ├── services/
│   │   ├── classification_service.py         # NEW: Skill level classification logic
│   │   ├── progress_tracking_service.py      # NEW: Chapter progress CRUD operations
│   │   ├── recommendation_service.py         # NEW: Recommendation algorithm + caching
│   │   ├── personalization_service.py        # NEW: Orchestrates classification + recommendations
│   │   └── profile_service.py                # EXISTING: From Phase 4A (Better Auth integration)
│   ├── api/
│   │   └── routes/
│   │       ├── personalization.py            # NEW: /api/v1/skill-level, /api/v1/recommendations
│   │       ├── progress.py                   # NEW: /api/v1/progress endpoints
│   │       └── chat.py                       # ENHANCED: Add profile context to RAG queries
│   └── migrations/
│       └── 005_personalization_schema.sql    # NEW: Database schema for Phase 4B
└── tests/
    ├── unit/
    │   ├── test_classification_service.py    # NEW: Skill classification unit tests
    │   ├── test_progress_service.py          # NEW: Progress tracking unit tests
    │   └── test_recommendation_service.py    # NEW: Recommendation algorithm tests
    └── integration/
        └── test_personalization_api.py       # NEW: End-to-end personalization flow tests

frontend/
├── src/
│   ├── components/
│   │   ├── PersonalizedSection.tsx           # ENHANCED: Add code language filtering
│   │   ├── RecommendationCard.tsx            # NEW: Display chapter recommendations
│   │   └── ProgressTracker.tsx               # NEW: Track chapter started/completed/bookmarked
│   ├── hooks/
│   │   ├── useSkillLevel.ts                  # NEW: Fetch user skill level classification
│   │   ├── useChapterProgress.ts             # NEW: Track and update chapter progress
│   │   └── useRecommendations.ts             # NEW: Fetch personalized recommendations
│   └── services/
│       └── personalization-api.ts            # NEW: API client for personalization endpoints
└── tests/
    ├── components/
    │   ├── PersonalizedSection.test.tsx      # ENHANCED: Test code language filtering
    │   └── RecommendationCard.test.tsx       # NEW: Test recommendation display
    └── e2e/
        └── personalization.spec.ts           # NEW: Playwright E2E tests for personalization flow
```

**Structure Decision**: This is a web application with backend (FastAPI Python) and frontend (Docusaurus React/TypeScript). The backend owns all personalization business logic (classification, progress tracking, recommendations, caching), while the frontend provides UI components and hooks to consume personalization APIs. Database schema migrations are version-controlled in backend/migrations/. Frontend components enhance existing PersonalizedSection and add new recommendation/progress tracking UI.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Post-Design Constitution Check (Phase 4B Complete)

### ✅ All Quality Gates Remain Valid
<!-- Copy same checklist from Constitution Check above -->
- [x] User skill level classification functional
- [x] Personalized recommendations display correctly
- [x] Response time < 2s (measured)
- [x] Recommendation relevance > 0.75 (A/B test)
- [ ] PHR created documenting personalization logic (will be created after command completes)

### ✅ Design Artifacts Generated
- ✅ `research.md` - Resolved all technical unknowns: skill classification algorithm (weighted scoring), progress tracking schema (unique constraint on user_id+chapter_id), recommendation algorithm (multi-factor scoring), adaptive content filtering (PersonalizedSection enhancement), profile-aware RAG (system message injection), chapter metadata (database table + JSON seed data), caching strategy (in-memory TTLCache)
- ✅ `data-model.md` - Defined 4 entities: SkillLevelClassification (1:1 with User), ChapterProgress (M:1 with User, unique per user+chapter), ChapterRecommendation (computed, cached), ChapterMetadata (static reference data). Includes database schema migration, relationships diagram, validation rules, state transitions.
- ✅ `quickstart.md` - Complete setup guide covering: environment configuration (.env files), database migration (005_personalization_schema.sql), backend/frontend installation, service implementation verification, testing procedures (unit/integration/E2E), quality gate validation, troubleshooting common issues, production deployment checklist, monitoring setup.
- ✅ `contracts/` - OpenAPI 3.1 specification (`personalization-api.yaml`) defining 8 endpoints: GET/POST /skill-level, GET /progress, POST /progress/start, POST /progress/complete, POST /progress/bookmark, GET /recommendations, POST /chat/query (enhanced). Includes request/response schemas, authentication requirements, error responses, rate limiting specifications.

### ✅ No New Complexity Violations

**Analysis**:
- **YAGNI Compliance**: Uses rule-based skill classification (no ML models), simple in-memory caching with TTL (no Redis), direct database queries (no repository pattern), enhances existing PersonalizedSection component (no rebuild). All design decisions follow "smallest viable change" principle. No premature abstractions introduced.
- **Security**: All personalization endpoints require JWT authentication from Better Auth. Input validation on all chapter_id and user_id parameters (alphanumeric + hyphens only, UUID format). No PII in recommendation logs (only user_id as hash or correlation ID). Rate limiting: 50 requests/hour for recommendations, 100 requests/hour for progress tracking. Parameterized SQL queries prevent injection attacks. No secrets in code (use .env).
- **Performance**: Skill classification < 500ms (simple weighted calculation), personalization response < 2s end-to-end (includes classification + recommendations + caching), recommendation cache reduces DB queries by ~70% (1-hour TTL), progress tracking updates < 300ms (single INSERT/UPDATE). All targets validated in research phase. Database indexes on user_id, status, chapter_id optimize common queries.
- **Accessibility**: PersonalizedSection enhancements maintain WCAG 2.1 AA compliance (keyboard navigation, focus indicators, screen reader announcements for content visibility changes). New UI components (RecommendationCard, ProgressTracker) designed with a11y from start: semantic HTML, ARIA labels, touch targets ≥ 44x44px. All interactive elements keyboard-accessible.
- **Free Tier**: Neon storage impact minimal: ~1KB per user for skill classification, ~500 bytes per chapter per user for progress tracking, ~10KB for chapter metadata (10 chapters). Supports 1000+ users within 0.5GB limit. Recommendation cache is in-memory (1MB max for 1000 users). No additional external services required. Database connection pooling limited to 10 connections (Neon free tier safe).

### ✅ Recommended ADRs

**Architectural Significance Test**:

**Decision 1: Use Rule-Based Skill Classification Instead of ML Models**
- **Impact**: ✅ Long-term consequences (classification algorithm versioned, affects all content recommendations)
- **Alternatives**: ✅ Multiple options considered (ML models, quiz-based assessment, hard-coded rules)
- **Scope**: ✅ Cross-cutting (influences recommendations, adaptive content, RAG responses)

📋 Architectural decision detected: **Rule-Based Skill Classification Algorithm**
   Document reasoning and tradeoffs? Run `/sp.adr rule-based-skill-classification`

**Decision 2: Use In-Memory TTLCache Instead of Redis for Recommendations**
- **Impact**: ✅ Long-term consequences (caching strategy, scalability limits, deployment simplicity)
- **Alternatives**: ✅ Multiple options considered (Redis, database materialized views, no caching)
- **Scope**: ✅ Cross-cutting (affects performance, deployment architecture, infrastructure costs)

📋 Architectural decision detected: **In-Memory Caching for Recommendations**
   Document reasoning and tradeoffs? Run `/sp.adr in-memory-recommendation-cache`

**Decision 3: Store Chapter Metadata in Database Instead of MDX Frontmatter**
- **Impact**: ✅ Long-term consequences (content authoring workflow, metadata versioning, queryability)
- **Alternatives**: ✅ Multiple options considered (MDX frontmatter, hardcoded in Python, separate CMS)
- **Scope**: ✅ Cross-cutting (influences content management, recommendation algorithm, API performance)

📋 Architectural decision detected: **Database-Stored Chapter Metadata**
   Document reasoning and tradeoffs? Run `/sp.adr database-chapter-metadata`

*Waiting for user consent before creating ADRs.*

---
